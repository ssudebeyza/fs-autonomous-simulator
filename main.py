import math
import sys

import pygame
from telemetry import TelemetryLogger
from controller import (
    PurePursuitController,
    PIDSpeedController,
)

from graphics import draw_scene

from speedplanner import CurvatureSpeedPlanner

from track import (
    create_centerline,
    create_oval_track,
    create_racing_line,
)

from vehicle import  FormulaStudentCar


# --------------------------------------------------
# PYGAME
# --------------------------------------------------

pygame.init()

SCREEN_WIDTH = 1400
SCREEN_HEIGHT = 900

screen = pygame.display.set_mode(
    (
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
    )
)

pygame.display.set_caption(
    "Formula Student Path Tracking Simulator"
)

clock = pygame.time.Clock()

font = pygame.font.SysFont(
    "Arial",
    20,
)


# --------------------------------------------------
# TRACK
# --------------------------------------------------

blue_cones, yellow_cones = create_oval_track(width=SCREEN_WIDTH, height=SCREEN_HEIGHT,)

centerline = create_centerline(
    blue_cones,
    yellow_cones,
)

racing_line = create_racing_line(
    centerline,
    offset_strength=16.0,
    smoothing_passes=6,
)


# --------------------------------------------------
# VEHICLE START
# --------------------------------------------------

START_INDEX = 0

start_x, start_y = racing_line[
    START_INDEX
]

next_x, next_y = racing_line[
    (START_INDEX + 1)
    % len(racing_line)
]

initial_yaw = math.atan2(
    -(next_y - start_y),
    next_x - start_x,
)

car = FormulaStudentCar(
    x=start_x,
    y=start_y,
    yaw=initial_yaw,
)


# --------------------------------------------------
# CONTROLLERS
# --------------------------------------------------

controller = PurePursuitController(
    lookahead_distance=70.0,
)

speed_controller = PIDSpeedController(
    kp=3.0,
    ki=0.0,
    kd=0.15,
    target_speed=120.0,
)

speed_planner = CurvatureSpeedPlanner()
telemetry = TelemetryLogger(
    filename="telemetry.csv",
)

telemetry.start()

# --------------------------------------------------
# SIMULATION
# --------------------------------------------------

autonomous_mode = False
target_speed = 0.0
nearest_index = 0
running = True
while running:

    delta_time = clock.tick(60) / 1000.0

    # ---------------------------------------------
    # EVENTS
    # ---------------------------------------------

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:

            if event.key == pygame.K_ESCAPE:
                running = False

            elif event.key == pygame.K_m:
                autonomous_mode = not autonomous_mode

                speed_controller.reset()

            elif event.key == pygame.K_r:

                car.x = start_x
                car.y = start_y
                car.yaw = initial_yaw

                car.speed = 0.0
                car.steering_angle = 0.0

                speed_controller.reset()

    # ---------------------------------------------
    # MANUAL CONTROL
    # ---------------------------------------------

    keys = pygame.key.get_pressed()

    if not autonomous_mode:

        throttle = 0.0

        if keys[pygame.K_w]:
            throttle = 4.0

        elif keys[pygame.K_s]:
            throttle = -6.0

        steering = 0.0

        if keys[pygame.K_a]:
            steering = 0.60

        elif keys[pygame.K_d]:
            steering = -0.60

        car.set_steering_angle(
            desired_steering_angle=steering,
        )

        car.update_speed_automatic(
            longitudinal_acceleration=throttle,
            delta_time=delta_time,
        )

    # ---------------------------------------------
    # AUTONOMOUS
    # ---------------------------------------------

    else:

        steering_angle, target_point = (
            controller.calculate_steering(
                car,
                racing_line,
            )
        )

        car.set_steering_angle(
            desired_steering_angle=steering_angle,
        )

        nearest_index = (
            controller.find_nearest_index(
                car,
                racing_line,
            )
        )

        target_speed = (
            speed_planner.calculate_target_speed(
                racing_line,
                nearest_index,
            )
        )

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

    # ---------------------------------------------
    # VEHICLE UPDATE
    # ---------------------------------------------

    car.update_position(
        delta_time,
    )
    telemetry.log(
    car=car,
    target_speed=target_speed,
    nearest_index=nearest_index,
)
    # ---------------------------------------------
    # DRAW
    # ---------------------------------------------

    draw_scene(
        surface=screen,
        car=car,
        blue_cones=blue_cones,
        yellow_cones=yellow_cones,
        centerline=centerline,
        width=SCREEN_WIDTH,
        height=SCREEN_HEIGHT,
        font=font,
        autonomous_mode=autonomous_mode,
        target_point=target_point if autonomous_mode else None,
        racing_line=racing_line,
    )

    pygame.display.flip()


# --------------------------------------------------
# EXIT
# --------------------------------------------------
telemetry.close()
pygame.quit()
sys.exit()