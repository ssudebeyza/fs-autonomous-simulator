import math
import sys
import time

import pygame

from camera import SimulatedCamera
from controller import (
    PurePursuitController,
    PIDSpeedController,
)

from graphics import draw_scene
from laptimer import LapTimer
from mpccontroller import MPCController
from pathgen import LocalPathGenerator
from performance import LapPerformanceTracker
from speedplanner import CurvatureSpeedPlanner
from telemetry import TelemetryLogger

from track import (
    create_centerline,
    create_oval_track,
    create_racing_line,
)

from vehicle import (
    FormulaStudentCar,
    PIXELS_PER_METER,
)


# ============================================================
# SETTINGS
# ============================================================

SCREEN_WIDTH = 1400
SCREEN_HEIGHT = 900

FPS = 60

LOCAL_PATH_HOLD_TIME = 1.2
RECOVERY_POINTS = 20


# ============================================================
# BASIC DISTANCE HELPERS
# ============================================================

def point_to_segment_distance(
    px,
    py,
    x1,
    y1,
    x2,
    y2,
):
    dx = x2 - x1
    dy = y2 - y1

    length_squared = (
        dx * dx
        + dy * dy
    )

    if length_squared == 0.0:
        return math.hypot(
            px - x1,
            py - y1,
        )

    t = (
        (px - x1) * dx
        + (py - y1) * dy
    ) / length_squared

    t = max(
        0.0,
        min(
            1.0,
            t,
        ),
    )

    closest_x = (
        x1 + t * dx
    )

    closest_y = (
        y1 + t * dy
    )

    return math.hypot(
        px - closest_x,
        py - closest_y,
    )


def point_to_line_distance(
    px,
    py,
    x1,
    y1,
    x2,
    y2,
):
    dx = x2 - x1
    dy = y2 - y1

    length = math.hypot(
        dx,
        dy,
    )

    if length < 1e-6:
        return math.hypot(
            px - x1,
            py - y1,
        )

    cross_product = abs(
        dx * (y1 - py)
        - (x1 - px) * dy
    )

    return (
        cross_product
        / length
    )


# ============================================================
# REFERENCE PATH ERROR
# ============================================================

def calculate_path_error_metres(
    car,
    path,
    closed_path=False,
):
    if len(path) < 2:
        return 0.0

    minimum_distance = (
        float("inf")
    )

    if closed_path:
        segment_count = len(path)
    else:
        segment_count = len(path) - 1

    for index in range(
        segment_count
    ):
        point_1 = path[index]

        if closed_path:
            point_2 = path[
                (index + 1)
                % len(path)
            ]
        else:
            point_2 = path[
                index + 1
            ]

        distance = (
            point_to_segment_distance(
                car.x,
                car.y,
                point_1[0],
                point_1[1],
                point_2[0],
                point_2[1],
            )
        )

        minimum_distance = min(
            minimum_distance,
            distance,
        )

    return (
        minimum_distance
        / PIXELS_PER_METER
    )


# ============================================================
# LOCAL PATH TRACKING ERROR
# ============================================================

def calculate_local_tracking_error_metres(
    car,
    path,
):
    if len(path) < 2:
        return math.nan

    first_point = path[0]
    second_point = path[1]

    minimum_distance = (
        point_to_line_distance(
            car.x,
            car.y,
            first_point[0],
            first_point[1],
            second_point[0],
            second_point[1],
        )
    )

    for index in range(
        1,
        len(path) - 1,
    ):
        point_1 = path[index]

        point_2 = path[
            index + 1
        ]

        distance = (
            point_to_segment_distance(
                car.x,
                car.y,
                point_1[0],
                point_1[1],
                point_2[0],
                point_2[1],
            )
        )

        minimum_distance = min(
            minimum_distance,
            distance,
        )

    return (
        minimum_distance
        / PIXELS_PER_METER
    )


# ============================================================
# ANGLE
# ============================================================

def normalize_angle(
    angle,
):
    return (
        angle + math.pi
    ) % (
        2.0 * math.pi
    ) - math.pi


# ============================================================
# LOCAL PATH TURN
# ============================================================

def calculate_local_path_curvature(
    path,
):
    if len(path) < 3:
        return 0.0

    maximum_angle = 0.0

    for index in range(
        1,
        len(path) - 1,
    ):
        previous_point = (
            path[index - 1]
        )

        current_point = (
            path[index]
        )

        next_point = (
            path[index + 1]
        )

        heading_1 = math.atan2(
            -(
                current_point[1]
                - previous_point[1]
            ),
            (
                current_point[0]
                - previous_point[0]
            ),
        )

        heading_2 = math.atan2(
            -(
                next_point[1]
                - current_point[1]
            ),
            (
                next_point[0]
                - current_point[0]
            ),
        )

        difference = (
            normalize_angle(
                heading_2
                - heading_1
            )
        )

        maximum_angle = max(
            maximum_angle,
            abs(difference),
        )

    return maximum_angle


# ============================================================
# CENTERLINE RECOVERY
# ============================================================

def create_recovery_path(
    car,
    centerline,
    controller,
    point_count,
):
    nearest_index = (
        controller.find_nearest_index(
            car,
            centerline,
        )
    )

    recovery_path = []

    for offset in range(
        point_count
    ):
        index = (
            nearest_index
            + offset
        ) % len(centerline)

        recovery_path.append(
            centerline[index]
        )

    return recovery_path


# ============================================================
# PYGAME
# ============================================================

pygame.init()

screen = pygame.display.set_mode(
    (
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
    )
)

pygame.display.set_caption(
    "Formula Student Autonomous Simulator"
)

clock = pygame.time.Clock()

font = pygame.font.SysFont(
    "Arial",
    21,
)


# ============================================================
# TRACK
# ============================================================

blue_cones, yellow_cones = (
    create_oval_track(
        width=SCREEN_WIDTH,
        height=SCREEN_HEIGHT,
    )
)

centerline = (
    create_centerline(
        blue_cones,
        yellow_cones,
    )
)

# Reference only.
# Vehicle does NOT use this for steering or speed planning.
racing_line = (
    create_racing_line(
        centerline,
        offset_strength=18.0,
        smoothing_passes=4,
    )
)


# ============================================================
# START
# ============================================================

start_point = (
    centerline[0]
)

next_point = (
    centerline[1]
)

start_yaw = math.atan2(
    -(
        next_point[1]
        - start_point[1]
    ),
    (
        next_point[0]
        - start_point[0]
    ),
)


# ============================================================
# VEHICLE
# ============================================================

car = FormulaStudentCar(
    x=start_point[0],
    y=start_point[1],
    yaw=start_yaw,
)


# ============================================================
# SIMULATED FRONT CAMERA
# ============================================================

camera = SimulatedCamera(
    image_width=640,
    image_height=360,
    horizontal_fov_deg=120.0,
    max_range_m=35.0,
    camera_height_m=0.75,
)


# ============================================================
# LOCAL PATH GENERATOR
# ============================================================

path_generator = LocalPathGenerator(
    min_track_width_m=3.0,
    max_track_width_m=11.0,
    max_forward_distance_m=35.0,
    min_forward_distance_m=-4.0,
    max_lateral_distance_m=20.0,
    max_forward_pair_difference_m=10.0,
    max_midpoint_gap_m=14.0
)


# ============================================================
# STEERING
# ============================================================

pure_pursuit_controller = PurePursuitController(
    lookahead_distance=18.0,
    wheelbase=12.8,
    max_steering=0.60,
    dynamic_lookahead=True,
    lookahead_gain=0.55,
    min_lookahead=10.0,
    max_lookahead=30.0,
    search_backward=4,
    search_forward=18,
)

mpc_controller = MPCController(
    wheelbase=12.8,
    max_steering=0.60,
    prediction_horizon=18,
    prediction_dt=0.08,
    steering_samples=17,
    pixels_per_meter=PIXELS_PER_METER,
    path_weight=8.0,
    heading_weight=5.0,
    steering_weight=0.05,
    steering_change_weight=0.25,
    progress_weight=0.05,
)

active_controller = pure_pursuit_controller
controller_name = "PURE PURSUIT"
controller_compute_ms = 0.0


# ============================================================
# SPEED CONTROL
# ============================================================

speed_controller = PIDSpeedController(
    kp=3.0,
    ki=0.0,
    kd=0.15,
    target_speed=8.0,
)

speed_planner = CurvatureSpeedPlanner(
    max_speed=15.0,
    min_speed=5.0,
    lateral_acceleration_limit=6.0,
    braking_acceleration=7.0,
    preview_points=12,
    acceleration_rate=3.0,
    deceleration_rate=8.0,
)


# ============================================================
# TELEMETRY
# ============================================================

telemetry = TelemetryLogger(
    filename="telemetry.csv",
)

telemetry.start()


# ============================================================
# LAP TIMER
# ============================================================

lap_timer = LapTimer(
    start_x=start_point[0],
    start_y=start_point[1],
    trigger_radius=25.0,
    minimum_lap_time=5.0,
)


# ============================================================
# PERFORMANCE
# ============================================================

performance_tracker = (
    LapPerformanceTracker()
)


# ============================================================
# STATE
# ============================================================

running = True
autonomous_mode = False

detected_blue = []
detected_yellow = []

local_path = []
last_valid_local_path = []
control_path = []

local_path_timeout = 0.0

target_point = None
target_speed = 0.0

nearest_index = 0

path_source = "NONE"
control_status = "MANUAL"

local_turn_angle = 0.0

reference_cte = 0.0
local_path_error = math.nan

camera_surface = None
camera_detections = []


# ============================================================
# MAIN LOOP
# ============================================================

while running:

    delta_time = (
        clock.tick(FPS)
        / 1000.0
    )


    # ========================================================
    # EVENTS
    # ========================================================

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:

            if event.key == pygame.K_ESCAPE:
                running = False

            if event.key == pygame.K_m:

                autonomous_mode = (
                    not autonomous_mode
                )

                pure_pursuit_controller.reset()
                mpc_controller.reset()
                speed_controller.reset()
                speed_planner.reset()

            if event.key == pygame.K_c:

                if controller_name == "PURE PURSUIT":
                    active_controller = mpc_controller
                    controller_name = "MPC"
                else:
                    active_controller = pure_pursuit_controller
                    controller_name = "PURE PURSUIT"

                pure_pursuit_controller.reset()
                mpc_controller.reset()
                car.set_steering_angle(
                    desired_steering_angle=0.0
                )

            if event.key == pygame.K_r:

                car.reset(
                    x=start_point[0],
                    y=start_point[1],
                    yaw=start_yaw,
                )

                pure_pursuit_controller.reset()
                mpc_controller.reset()
                speed_controller.reset()
                speed_planner.reset()

                local_path = []
                last_valid_local_path = []
                control_path = []

                local_path_timeout = 0.0

                target_point = None
                target_speed = 0.0

                reference_cte = 0.0
                local_path_error = math.nan

                path_source = "NONE"
                control_status = "RESET"

                lap_timer = LapTimer(
                    start_x=start_point[0],
                    start_y=start_point[1],
                    trigger_radius=25.0,
                    minimum_lap_time=5.0,
                )

                performance_tracker = (
                    LapPerformanceTracker()
                )


    # ========================================================
    # CAMERA PERCEPTION
    #
    # The controller no longer uses ConePerception. The simulated
    # camera produces forward/lateral detections, which are then
    # reconstructed into world coordinates for LocalPathGenerator.
    # ========================================================

    camera_surface, camera_detections = camera.render(
        car=car,
        blue_cones=blue_cones,
        yellow_cones=yellow_cones,
    )

    detected_blue, detected_yellow = camera.detections_to_world(
        car=car,
        detections=camera_detections,
    )


    # ========================================================
    # CAMERA-BASED LOCAL PATH GENERATION
    # ========================================================

    local_path = (
        path_generator.generate_local_path(
            car=car,
            detected_blue=detected_blue,
            detected_yellow=detected_yellow,
        )
    )


    # ========================================================
    # CONTROL PATH SELECTION
    # ========================================================

    if len(local_path) >= 2:

        control_path = (
            local_path.copy()
        )

        last_valid_local_path = (
            local_path.copy()
        )

        local_path_timeout = 0.0

        path_source = (
            "CAMERA PATH"
        )

    else:

        local_path_timeout += (
            delta_time
        )

        if (
            len(last_valid_local_path) >= 2
            and
            local_path_timeout
            <= LOCAL_PATH_HOLD_TIME
        ):

            control_path = (
                last_valid_local_path.copy()
            )

            path_source = (
                "PATH MEMORY"
            )

        else:

            control_path = (
                create_recovery_path(
                    car,
                    centerline,
                    pure_pursuit_controller,
                    RECOVERY_POINTS,
                )
            )

            path_source = (
                "CENTERLINE RECOVERY"
            )


    # ========================================================
    # LOCAL TURN
    # ========================================================

    if len(local_path) >= 3:

        local_turn_angle = (
            calculate_local_path_curvature(
                local_path
            )
        )

    elif len(control_path) >= 3:

        local_turn_angle = (
            calculate_local_path_curvature(
                control_path
            )
        )

    else:

        local_turn_angle = 0.0


    # ========================================================
    # AUTONOMOUS CONTROL
    # ========================================================

    if autonomous_mode:

        # ----------------------------------------------------
        # STEERING
        # ----------------------------------------------------

        if len(control_path) >= 2:

            controller_start_time = time.perf_counter()

            steering_angle, target_point = (
                active_controller.calculate_steering(
                    car,
                    control_path,
                )
            )

            controller_compute_ms = (
                time.perf_counter()
                - controller_start_time
            ) * 1000.0

            car.set_steering_angle(
                desired_steering_angle=(
                    steering_angle
                )
            )

            control_status = (
                path_source
            )

        else:

            car.set_steering_angle(
                desired_steering_angle=0.0
            )

            target_point = None

            control_status = (
                "NO PATH"
            )

            controller_compute_ms = 0.0


        # ----------------------------------------------------
        # LOCAL-PATH SPEED PLANNING
        #
        # No racing line here.
        # ----------------------------------------------------

        if len(control_path) >= 3:

            target_speed = (
                speed_planner.calculate_local_target_speed(
                    local_path=control_path,
                    current_speed=car.speed,
                    delta_time=delta_time,
                    pixels_per_meter=PIXELS_PER_METER,
                )
            )

        else:

            target_speed = 5.0


        # ----------------------------------------------------
        # SHORT PATH SAFETY LIMIT
        # ----------------------------------------------------

        if len(local_path) < 5:

            target_speed = min(
                target_speed,
                7.0,
            )

        elif len(local_path) < 8:

            target_speed = min(
                target_speed,
                10.0,
            )


        # ----------------------------------------------------
        # HAIRPIN / TURN SAFETY LIMIT
        # ----------------------------------------------------

        if (
            local_turn_angle
            > math.radians(55.0)
        ):

            target_speed = min(
                target_speed,
                5.0,
            )

        elif (
            local_turn_angle
            > math.radians(35.0)
        ):

            target_speed = min(
                target_speed,
                6.5,
            )

        elif (
            local_turn_angle
            > math.radians(20.0)
        ):

            target_speed = min(
                target_speed,
                8.5,
            )


        # ----------------------------------------------------
        # PATH MEMORY SAFETY
        # ----------------------------------------------------

        if (
            path_source
            == "PATH MEMORY"
        ):

            target_speed = min(
                target_speed,
                6.0,
            )


        # ----------------------------------------------------
        # RECOVERY SAFETY
        # ----------------------------------------------------

        if (
            path_source
            == "CENTERLINE RECOVERY"
        ):

            target_speed = min(
                target_speed,
                5.0,
            )


        # ----------------------------------------------------
        # PID SPEED CONTROL
        # ----------------------------------------------------

        speed_controller.target_speed = (
            target_speed
        )

        acceleration = (
            speed_controller.calculate_acceleration(
                current_speed=car.speed,
                delta_time=delta_time,
            )
        )

        car.update_speed_automatic(
            longitudinal_acceleration=acceleration,
            delta_time=delta_time,
        )


    # ========================================================
    # MANUAL
    # ========================================================

    else:

        keys = pygame.key.get_pressed()

        throttle = 0.0
        brake = 0.0
        steering = 0.0

        if keys[pygame.K_w]:
            throttle = 1.0

        if keys[pygame.K_s]:
            brake = 1.0

        if keys[pygame.K_a]:
            steering = 0.45

        if keys[pygame.K_d]:
            steering = -0.45

        car.set_steering_angle(
            desired_steering_angle=(
                steering
            )
        )

        car.update_speed_manual(
            throttle=throttle,
            brake=brake,
            delta_time=delta_time,
        )

        target_speed = 0.0
        target_point = None

        control_status = (
            "MANUAL"
        )

        controller_compute_ms = 0.0


    # ========================================================
    # VEHICLE UPDATE
    # ========================================================

    car.update_position(
        delta_time=delta_time
    )


    # ========================================================
    # REFERENCE INDEX
    #
    # Reference only for telemetry.
    # Not used for vehicle control.
    # ========================================================

    nearest_index = (
        pure_pursuit_controller.find_nearest_index(
            car,
            racing_line,
        )
    )


    # ========================================================
    # REFERENCE CTE
    # ========================================================

    reference_cte = (
        calculate_path_error_metres(
            car,
            racing_line,
            closed_path=True,
        )
    )


    # ========================================================
    # LOCAL TRACKING ERROR
    # ========================================================

    if (
        path_source == "CAMERA PATH"
        and
        len(local_path) >= 2
    ):

        local_path_error = (
            calculate_local_tracking_error_metres(
                car,
                local_path,
            )
        )

    else:

        local_path_error = (
            math.nan
        )


    # ========================================================
    # PERFORMANCE
    # ========================================================

    performance_tracker.add_sample(
        speed=car.speed,
        cross_track_error=(
            reference_cte
        ),
    )


    # ========================================================
    # LAP TIMER
    # ========================================================

    (
        lap_completed,
        completed_lap_time,
    ) = lap_timer.update(
        car
    )

    if lap_completed:

        completed_lap_number = (
            lap_timer.current_lap
            - 1
        )

        result = (
            performance_tracker.complete_lap(
                lap_number=(
                    completed_lap_number
                ),
                lap_time=(
                    completed_lap_time
                ),
            )
        )

        print()

        print(
            f"Lap {result['lap']} Performance"
        )

        print(
            "----------------------------"
        )

        print(
            f"Lap Time: "
            f"{result['lap_time']:.3f} s"
        )

        print(
            f"Average Speed: "
            f"{result['average_speed']:.2f} m/s"
        )

        print(
            f"Maximum Speed: "
            f"{result['maximum_speed']:.2f} m/s"
        )

        print(
            f"Mean Reference CTE: "
            f"{result['mean_cte']:.3f} m"
        )

        print(
            f"Reference CTE RMSE: "
            f"{result['cte_rmse']:.3f} m"
        )

        print()


    # ========================================================
    # TELEMETRY
    # ========================================================

    telemetry.log(
        car=car,
        target_speed=target_speed,
        nearest_index=nearest_index,
        reference_cte=reference_cte,
        local_path_error=(
            local_path_error
        ),
        path_source=path_source,
    )


    # ========================================================
    # DRAW BASE SCENE
    # ========================================================

    draw_scene(
        surface=screen,
        car=car,
        blue_cones=blue_cones,
        yellow_cones=yellow_cones,
        centerline=centerline,
        width=SCREEN_WIDTH,
        height=SCREEN_HEIGHT,
        font=font,
        autonomous_mode=(
            autonomous_mode
        ),
        target_point=target_point,
        racing_line=None,
        lap_timer=lap_timer,
    )


    # ========================================================
    # SIMULATED FRONT CAMERA PREVIEW
    #
    # The same detections shown here are now used to generate
    # the controller local path.
    # ========================================================

    camera_preview = pygame.transform.smoothscale(
        camera_surface,
        (320, 180),
    )

    camera_x = SCREEN_WIDTH - camera_preview.get_width() - 15
    camera_y = 15

    screen.blit(
        camera_preview,
        (camera_x, camera_y),
    )

    pygame.draw.rect(
        screen,
        (255, 255, 255),
        (
            camera_x - 2,
            camera_y - 2,
            camera_preview.get_width() + 4,
            camera_preview.get_height() + 4,
        ),
        2,
    )


    # ========================================================
    # DETECTED BLUE CONES
    # ========================================================

    for cone in detected_blue:

        pygame.draw.circle(
            screen,
            (0, 255, 255),
            (
                int(cone[0]),
                int(cone[1]),
            ),
            11,
            3,
        )


    # ========================================================
    # DETECTED YELLOW CONES
    # ========================================================

    for cone in detected_yellow:

        pygame.draw.circle(
            screen,
            (255, 0, 255),
            (
                int(cone[0]),
                int(cone[1]),
            ),
            11,
            3,
        )


    # ========================================================
    # LOCAL PATH
    # ========================================================

    if len(local_path) >= 2:

        pygame.draw.lines(
            screen,
            (255, 0, 255),
            False,
            [
                (
                    int(point[0]),
                    int(point[1]),
                )
                for point in local_path
            ],
            6,
        )

        for point in local_path:

            pygame.draw.circle(
                screen,
                (255, 255, 255),
                (
                    int(point[0]),
                    int(point[1]),
                ),
                6,
            )


    # ========================================================
    # MEMORY / RECOVERY
    # ========================================================

    if (
        len(control_path) >= 2
        and
        path_source != "CAMERA PATH"
    ):

        pygame.draw.lines(
            screen,
            (255, 165, 0),
            False,
            [
                (
                    int(point[0]),
                    int(point[1]),
                )
                for point in control_path
            ],
            3,
        )


    # ========================================================
    # HUD
    # ========================================================

    panel = pygame.Rect(
        15,
        15,
        710,
        310,
    )

    pygame.draw.rect(
        screen,
        (0, 0, 0),
        panel,
    )


    if math.isnan(
        local_path_error
    ):

        local_error_text = (
            "N/A"
        )

    else:

        local_error_text = (
            f"{local_path_error:.3f} m"
        )


    hud_lines = [
        (
            f"BLUE {len(detected_blue)}   "
            f"YELLOW {len(detected_yellow)}   "
            f"PATH {len(local_path)}"
        ),

        (
            f"CAMERA VISIBLE: {len(camera_detections)}   "
            f"CONTROL: {control_status}"
        ),

        (
            f"CONTROLLER: {controller_name}   "
            f"COMPUTE {controller_compute_ms:.2f} ms"
        ),

        (
            "C = SWITCH CONTROLLER   M = AUTO/MANUAL   R = RESET"
        ),

        (
            f"CONTROL POINTS: "
            f"{len(control_path)}"
        ),

        (
            f"SPEED "
            f"{car.speed:.2f} m/s   "
            f"TARGET "
            f"{target_speed:.2f} m/s"
        ),

        (
            f"STEER "
            f"{car.steering_angle:.3f} rad"
        ),

        (
            f"LOCAL TURN "
            f"{math.degrees(local_turn_angle):.1f} deg"
        ),

        (
            f"REFERENCE CTE "
            f"{reference_cte:.3f} m"
        ),

        (
            f"LOCAL TRACKING ERROR "
            f"{local_error_text}"
        ),

        (
            "SPEED SOURCE: CAMERA LOCAL PATH"
        ),
    ]


    for index, text in enumerate(
        hud_lines
    ):

        text_surface = (
            font.render(
                text,
                True,
                (255, 255, 255),
            )
        )

        screen.blit(
            text_surface,
            (
                30,
                25
                + index * 26,
            ),
        )


    pygame.display.flip()


# ============================================================
# SHUTDOWN
# ============================================================

telemetry.close()

pygame.quit()

sys.exit()