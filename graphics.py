import math

import pygame


# --------------------------------------------------
# COLOURS
# --------------------------------------------------

BACKGROUND = (22, 24, 27)
TRACK_COLOUR = (55, 58, 62)

BLUE_CONE = (40, 120, 255)
YELLOW_CONE = (255, 210, 40)

CENTERLINE_COLOUR = (110, 110, 110)
RACING_LINE_COLOUR = (0, 220, 210)

WHITE = (245, 245, 245)
BLACK = (15, 15, 15)

CAR_BODY = (35, 35, 38)
CAR_DETAIL = (0, 220, 190)
TYRE_COLOUR = (12, 12, 12)

HUD_BACKGROUND = (0, 0, 0, 150)


# --------------------------------------------------
# GEOMETRY HELPERS
# --------------------------------------------------

def transform_car_point(
    car,
    local_x,
    local_y,
):
    """
    Convert a point from local vehicle coordinates
    into screen coordinates.
    """

    cos_yaw = math.cos(
        car.yaw
    )

    sin_yaw = math.sin(
        car.yaw
    )

    screen_x = (
        car.x
        + local_x * cos_yaw
        + local_y * sin_yaw
    )

    screen_y = (
        car.y
        - local_x * sin_yaw
        + local_y * cos_yaw
    )

    return (
        int(screen_x),
        int(screen_y),
    )


def draw_rotated_rectangle(
    surface,
    car,
    center_x,
    center_y,
    length,
    width,
    colour,
):
    half_length = (
        length / 2
    )

    half_width = (
        width / 2
    )

    local_points = [
        (
            center_x - half_length,
            center_y - half_width,
        ),
        (
            center_x + half_length,
            center_y - half_width,
        ),
        (
            center_x + half_length,
            center_y + half_width,
        ),
        (
            center_x - half_length,
            center_y + half_width,
        ),
    ]

    screen_points = [
        transform_car_point(
            car,
            point[0],
            point[1],
        )
        for point in local_points
    ]

    pygame.draw.polygon(
        surface,
        colour,
        screen_points,
    )


# --------------------------------------------------
# TRACK DRAWING
# --------------------------------------------------

def draw_centerline(
    surface,
    centerline,
):
    if (
        centerline is None
        or len(centerline) < 2
    ):
        return

    pygame.draw.lines(
        surface,
        CENTERLINE_COLOUR,
        True,
        centerline,
        1,
    )


def draw_racing_line(
    surface,
    racing_line,
):
    if (
        racing_line is None
        or len(racing_line) < 2
    ):
        return

    pygame.draw.lines(
        surface,
        RACING_LINE_COLOUR,
        True,
        racing_line,
        2,
    )


def draw_cones(
    surface,
    cones,
    colour,
):
    for cone in cones:
        pygame.draw.circle(
            surface,
            colour,
            (
                int(cone[0]),
                int(cone[1]),
            ),
            5,
        )

        pygame.draw.circle(
            surface,
            BLACK,
            (
                int(cone[0]),
                int(cone[1]),
            ),
            5,
            1,
        )


# --------------------------------------------------
# FORMULA-STYLE CAR
# --------------------------------------------------

def draw_car(
    surface,
    car,
):
    """
    Draw a simplified top-view Formula-style car.
    """

    # Rear wing
    draw_rotated_rectangle(
        surface,
        car,
        -23,
        0,
        7,
        32,
        CAR_BODY,
    )

    # Rear tyres
    draw_rotated_rectangle(
        surface,
        car,
        -13,
        -14,
        13,
        7,
        TYRE_COLOUR,
    )

    draw_rotated_rectangle(
        surface,
        car,
        -13,
        14,
        13,
        7,
        TYRE_COLOUR,
    )

    # Main chassis
    chassis_points = [
        (-18, -6),
        (8, -7),
        (21, -4),
        (26, 0),
        (21, 4),
        (8, 7),
        (-18, 6),
    ]

    chassis_screen = [
        transform_car_point(
            car,
            x,
            y,
        )
        for x, y in chassis_points
    ]

    pygame.draw.polygon(
        surface,
        CAR_BODY,
        chassis_screen,
    )

    # Sidepods
    draw_rotated_rectangle(
        surface,
        car,
        0,
        -8,
        17,
        7,
        CAR_BODY,
    )

    draw_rotated_rectangle(
        surface,
        car,
        0,
        8,
        17,
        7,
        CAR_BODY,
    )

    # Front tyres
    draw_rotated_rectangle(
        surface,
        car,
        14,
        -13,
        11,
        6,
        TYRE_COLOUR,
    )

    draw_rotated_rectangle(
        surface,
        car,
        14,
        13,
        11,
        6,
        TYRE_COLOUR,
    )

    # Front wing
    draw_rotated_rectangle(
        surface,
        car,
        27,
        0,
        5,
        29,
        CAR_BODY,
    )

    # Cockpit
    cockpit = transform_car_point(
        car,
        -2,
        0,
    )

    pygame.draw.circle(
        surface,
        BLACK,
        cockpit,
        5,
    )

    # Halo / central detail
    halo_front = transform_car_point(
        car,
        3,
        0,
    )

    halo_back = transform_car_point(
        car,
        -6,
        0,
    )

    pygame.draw.line(
        surface,
        CAR_DETAIL,
        halo_back,
        halo_front,
        2,
    )

    # Formula-style turquoise details
    nose_start = transform_car_point(
        car,
        6,
        0,
    )

    nose_end = transform_car_point(
        car,
        24,
        0,
    )

    pygame.draw.line(
        surface,
        CAR_DETAIL,
        nose_start,
        nose_end,
        2,
    )

    rear_detail_left = transform_car_point(
        car,
        -18,
        -8,
    )

    rear_detail_right = transform_car_point(
        car,
        -18,
        8,
    )

    pygame.draw.line(
        surface,
        CAR_DETAIL,
        rear_detail_left,
        rear_detail_right,
        2,
    )


# --------------------------------------------------
# HUD
# --------------------------------------------------

def draw_hud(
    surface,
    car,
    font,
    autonomous_mode,
    lap_timer=None,
):
    hud_surface = pygame.Surface(
        (
            310,
            245,
        ),
        pygame.SRCALPHA,
    )

    hud_surface.fill(
        HUD_BACKGROUND
    )

    if autonomous_mode:
        mode_text = "AUTOMATIC"
    else:
        mode_text = "MANUAL"

    speed_text = (
        f"Speed: "
        f"{car.speed:.2f} m/s"
    )

    steering_text = (
        f"Steering: "
        f"{car.steering_angle:.3f} rad"
    )

    mode_surface = font.render(
        f"Mode: {mode_text}",
        True,
        WHITE,
    )

    speed_surface = font.render(
        speed_text,
        True,
        WHITE,
    )

    steering_surface = font.render(
        steering_text,
        True,
        WHITE,
    )

    hud_surface.blit(
        mode_surface,
        (15, 15),
    )

    hud_surface.blit(
        speed_surface,
        (15, 45),
    )

    hud_surface.blit(
        steering_surface,
        (15, 75),
    )

    if lap_timer is not None:
        current_lap_time = (
            lap_timer.get_current_lap_time()
        )

        lap_surface = font.render(
            f"Lap: {lap_timer.current_lap}",
            True,
            WHITE,
        )

        current_surface = font.render(
            (
                f"Current Lap: "
                f"{current_lap_time:.2f} s"
            ),
            True,
            WHITE,
        )

        if (
            lap_timer.best_lap
            is None
        ):
            best_lap_text = "--"
        else:
            best_lap_text = (
                f"{lap_timer.best_lap:.2f} s"
            )

        best_surface = font.render(
            (
                f"Best Lap: "
                f"{best_lap_text}"
            ),
            True,
            WHITE,
        )

        average_lap = (
            lap_timer.get_average_lap_time()
        )

        if average_lap is None:
            average_text = "--"
        else:
            average_text = (
                f"{average_lap:.2f} s"
            )

        average_surface = font.render(
            (
                f"Average Lap: "
                f"{average_text}"
            ),
            True,
            WHITE,
        )

        hud_surface.blit(
            lap_surface,
            (15, 115),
        )

        hud_surface.blit(
            current_surface,
            (15, 145),
        )

        hud_surface.blit(
            best_surface,
            (15, 175),
        )

        hud_surface.blit(
            average_surface,
            (15, 205),
        )

    surface.blit(
        hud_surface,
        (15, 15),
    )


# --------------------------------------------------
# MAIN SCENE
# --------------------------------------------------

def draw_scene(
    surface,
    car,
    blue_cones,
    yellow_cones,
    centerline,
    width,
    height,
    font,
    autonomous_mode,
    target_point=None,
    racing_line=None,
    lap_timer=None,
):
    surface.fill(
        BACKGROUND
    )

    # Track area
    pygame.draw.rect(
        surface,
        TRACK_COLOUR,
        pygame.Rect(
            0,
            0,
            width,
            height,
        ),
    )

    # Reference lines
    draw_centerline(
        surface,
        centerline,
    )

    draw_racing_line(
        surface,
        racing_line,
    )

    # Track cones
    draw_cones(
        surface,
        blue_cones,
        BLUE_CONE,
    )

    draw_cones(
        surface,
        yellow_cones,
        YELLOW_CONE,
    )

    # Vehicle
    draw_car(
        surface,
        car,
    )

    # HUD
    draw_hud(
        surface,
        car,
        font,
        autonomous_mode,
        lap_timer=lap_timer,
    )