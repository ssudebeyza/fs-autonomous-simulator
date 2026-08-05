import math
import os

import pygame


# --------------------------------------------------
# COLOURS
# --------------------------------------------------

BACKGROUND_COLOUR = (35, 120, 60)
TRACK_COLOUR = (65, 65, 65)

WHITE = (255, 255, 255)
BLACK = (20, 20, 20)

BLUE = (40, 100, 255)
YELLOW = (255, 220, 30)
ORANGE = (255, 130, 20)

HUD_BACKGROUND = (15, 15, 15, 180)


# --------------------------------------------------
# CAR IMAGE
# --------------------------------------------------

BASE_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

CAR_IMAGE_PATH = os.path.join(
    BASE_DIR,
    "assets",
    "mercedes_f1.png",
)

CAR_IMAGE = None


def load_car_image():
    """
    Loads and resizes the Mercedes-style car image.
    """

    global CAR_IMAGE

    if CAR_IMAGE is not None:
        return CAR_IMAGE

    if not os.path.exists(CAR_IMAGE_PATH):
        raise FileNotFoundError(
            "Car image could not be found:\n"
            f"{CAR_IMAGE_PATH}\n\n"
            "Create an 'assets' folder and place "
            "'mercedes_f1.png' inside it."
        )

    image = pygame.image.load(
        CAR_IMAGE_PATH
    ).convert_alpha()

    CAR_IMAGE = pygame.transform.smoothscale(
        image,
        (38, 68),
    )

    return CAR_IMAGE


# --------------------------------------------------
# CONE DRAWING
# --------------------------------------------------

def draw_cone(
    surface: pygame.Surface,
    position: tuple[float, float],
    colour: tuple[int, int, int],
) -> None:
    """
    Draws a single track cone.
    """

    x, y = position

    pygame.draw.circle(
        surface,
        BLACK,
        (int(x), int(y)),
        8,
    )

    pygame.draw.circle(
        surface,
        colour,
        (int(x), int(y)),
        6,
    )

    pygame.draw.circle(
        surface,
        WHITE,
        (int(x), int(y)),
        2,
    )


# --------------------------------------------------
# TRACK DRAWING
# --------------------------------------------------

def draw_track(
    surface: pygame.Surface,
    blue_cones: list[tuple[float, float]],
    yellow_cones: list[tuple[float, float]],
) -> None:
    """
    Draws the asphalt and both cone boundaries.
    """

    if len(blue_cones) >= 3:
        pygame.draw.polygon(
            surface,
            TRACK_COLOUR,
            blue_cones,
        )

    if len(yellow_cones) >= 3:
        pygame.draw.polygon(
            surface,
            BACKGROUND_COLOUR,
            yellow_cones,
        )

    for cone_position in blue_cones:
        draw_cone(
            surface,
            cone_position,
            BLUE,
        )

    for cone_position in yellow_cones:
        draw_cone(
            surface,
            cone_position,
            YELLOW,
        )


def draw_centerline(
    surface: pygame.Surface,
    centerline: list[tuple[float, float]],
) -> None:
    """
    Draws a subtle dashed centreline.
    """

    if len(centerline) < 2:
        return

    for index in range(
        0,
        len(centerline),
        2,
    ):
        next_index = (
            index + 1
        ) % len(centerline)

        pygame.draw.line(
            surface,
            (190, 190, 190),
            centerline[index],
            centerline[next_index],
            1,
        )


def draw_start_gate(
    surface: pygame.Surface,
    centerline: list[tuple[float, float]],
) -> None:
    """
    Draws the start gate near the first centreline point.
    """

    if not centerline:
        return

    start_x, start_y = centerline[0]

    draw_cone(
        surface,
        (start_x - 42, start_y),
        ORANGE,
    )

    draw_cone(
        surface,
        (start_x + 42, start_y),
        ORANGE,
    )


# --------------------------------------------------
# VEHICLE DRAWING
# --------------------------------------------------
def transform_car_point(
    car,
    local_x: float,
    local_y: float,
) -> tuple[int, int]:
    """
    Converts a point from the car's local coordinate system
    to screen coordinates.
    """

    cosine_yaw = math.cos(car.yaw)
    sine_yaw = math.sin(car.yaw)

    rotated_x = (
        local_x * cosine_yaw
        - local_y * sine_yaw
    )

    rotated_y = (
        local_x * sine_yaw
        + local_y * cosine_yaw
    )

    screen_x = car.x + rotated_x
    screen_y = car.y - rotated_y

    return (
        int(screen_x),
        int(screen_y),
    )


def draw_rotated_rectangle(
    surface: pygame.Surface,
    car,
    centre_x: float,
    centre_y: float,
    length: float,
    width: float,
    colour: tuple[int, int, int],
    extra_rotation: float = 0.0,
    border_colour=None,
) -> None:
    """
    Draws a rectangle attached to the car.
    """

    half_length = length / 2
    half_width = width / 2

    cosine_rotation = math.cos(extra_rotation)
    sine_rotation = math.sin(extra_rotation)

    local_corners = [
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
        (-half_length, -half_width),
    ]

    transformed_corners = []

    for corner_x, corner_y in local_corners:
        rotated_x = (
            corner_x * cosine_rotation
            - corner_y * sine_rotation
        )

        rotated_y = (
            corner_x * sine_rotation
            + corner_y * cosine_rotation
        )

        transformed_corners.append(
            transform_car_point(
                car,
                centre_x + rotated_x,
                centre_y + rotated_y,
            )
        )

    pygame.draw.polygon(
        surface,
        colour,
        transformed_corners,
    )

    if border_colour is not None:
        pygame.draw.polygon(
            surface,
            border_colour,
            transformed_corners,
            1,
        )


def draw_car(
    surface: pygame.Surface,
    car,
) -> None:
    """
    Draws a Mercedes-inspired Formula-style car.
    """

    body_black = (18, 20, 22)
    body_silver = (180, 185, 190)
    dark_silver = (75, 80, 85)
    turquoise = (0, 220, 210)
    tyre_colour = (8, 8, 8)
    cockpit_colour = (3, 3, 3)
    suspension_colour = (45, 45, 45)

    # --------------------------------------------------
    # REAR WING
    # --------------------------------------------------

    rear_wing_points = [
        transform_car_point(car, -37, -19),
        transform_car_point(car, -31, -19),
        transform_car_point(car, -31, 19),
        transform_car_point(car, -37, 19),
    ]

    pygame.draw.polygon(
        surface,
        body_black,
        rear_wing_points,
    )

    pygame.draw.line(
        surface,
        turquoise,
        transform_car_point(car, -34, -17),
        transform_car_point(car, -34, 17),
        2,
    )

    # Rear-wing supports
    pygame.draw.line(
        surface,
        suspension_colour,
        transform_car_point(car, -30, -7),
        transform_car_point(car, -22, -5),
        3,
    )

    pygame.draw.line(
        surface,
        suspension_colour,
        transform_car_point(car, -30, 7),
        transform_car_point(car, -22, 5),
        3,
    )

    # --------------------------------------------------
    # REAR WHEELS
    # --------------------------------------------------

    draw_rotated_rectangle(
        surface,
        car,
        centre_x=-20,
        centre_y=-17,
        length=15,
        width=8,
        colour=tyre_colour,
        border_colour=(80, 80, 80),
    )

    draw_rotated_rectangle(
        surface,
        car,
        centre_x=-20,
        centre_y=17,
        length=15,
        width=8,
        colour=tyre_colour,
        border_colour=(80, 80, 80),
    )

    # --------------------------------------------------
    # MAIN BODY
    # --------------------------------------------------

    main_body = [
        transform_car_point(car, 28, 0),
        transform_car_point(car, 18, -6),
        transform_car_point(car, 5, -10),
        transform_car_point(car, -17, -9),
        transform_car_point(car, -27, -5),
        transform_car_point(car, -27, 5),
        transform_car_point(car, -17, 9),
        transform_car_point(car, 5, 10),
        transform_car_point(car, 18, 6),
    ]

    pygame.draw.polygon(
        surface,
        body_black,
        main_body,
    )

    pygame.draw.polygon(
        surface,
        body_silver,
        main_body,
        2,
    )

    # Sidepods
    left_sidepod = [
        transform_car_point(car, 7, -8),
        transform_car_point(car, -9, -13),
        transform_car_point(car, -21, -10),
        transform_car_point(car, -14, -6),
    ]

    right_sidepod = [
        transform_car_point(car, 7, 8),
        transform_car_point(car, -9, 13),
        transform_car_point(car, -21, 10),
        transform_car_point(car, -14, 6),
    ]

    pygame.draw.polygon(
        surface,
        dark_silver,
        left_sidepod,
    )

    pygame.draw.polygon(
        surface,
        dark_silver,
        right_sidepod,
    )

    pygame.draw.line(
        surface,
        turquoise,
        transform_car_point(car, 8, -7),
        transform_car_point(car, -18, -9),
        2,
    )

    pygame.draw.line(
        surface,
        turquoise,
        transform_car_point(car, 8, 7),
        transform_car_point(car, -18, 9),
        2,
    )

    # --------------------------------------------------
    # ENGINE COVER
    # --------------------------------------------------

    engine_cover = [
        transform_car_point(car, 3, -4),
        transform_car_point(car, -22, -5),
        transform_car_point(car, -28, 0),
        transform_car_point(car, -22, 5),
        transform_car_point(car, 3, 4),
    ]

    pygame.draw.polygon(
        surface,
        body_silver,
        engine_cover,
    )

    pygame.draw.line(
        surface,
        turquoise,
        transform_car_point(car, -24, 0),
        transform_car_point(car, 7, 0),
        2,
    )

    # --------------------------------------------------
    # COCKPIT AND HALO
    # --------------------------------------------------

    cockpit_position = transform_car_point(
        car,
        2,
        0,
    )

    pygame.draw.ellipse(
        surface,
        cockpit_colour,
        (
            cockpit_position[0] - 6,
            cockpit_position[1] - 8,
            12,
            16,
        ),
    )

    pygame.draw.ellipse(
        surface,
        turquoise,
        (
            cockpit_position[0] - 6,
            cockpit_position[1] - 8,
            12,
            16,
        ),
        2,
    )

    halo_front = transform_car_point(car, 8, 0)
    halo_left = transform_car_point(car, 1, -5)
    halo_right = transform_car_point(car, 1, 5)

    pygame.draw.line(
        surface,
        body_silver,
        halo_front,
        halo_left,
        2,
    )

    pygame.draw.line(
        surface,
        body_silver,
        halo_front,
        halo_right,
        2,
    )

    # --------------------------------------------------
    # NOSE
    # --------------------------------------------------

    nose = [
        transform_car_point(car, 40, -2),
        transform_car_point(car, 40, 2),
        transform_car_point(car, 16, 5),
        transform_car_point(car, 12, -5),
    ]

    pygame.draw.polygon(
        surface,
        body_silver,
        nose,
    )

    pygame.draw.line(
        surface,
        turquoise,
        transform_car_point(car, 39, 0),
        transform_car_point(car, 13, 0),
        2,
    )

    # --------------------------------------------------
    # FRONT SUSPENSION
    # --------------------------------------------------

    pygame.draw.line(
        surface,
        suspension_colour,
        transform_car_point(car, 17, -5),
        transform_car_point(car, 23, -15),
        2,
    )

    pygame.draw.line(
        surface,
        suspension_colour,
        transform_car_point(car, 17, 5),
        transform_car_point(car, 23, 15),
        2,
    )

    pygame.draw.line(
        surface,
        suspension_colour,
        transform_car_point(car, 29, -3),
        transform_car_point(car, 23, -15),
        2,
    )

    pygame.draw.line(
        surface,
        suspension_colour,
        transform_car_point(car, 29, 3),
        transform_car_point(car, 23, 15),
        2,
    )

    # --------------------------------------------------
    # FRONT WHEELS
    # --------------------------------------------------

    front_wheel_rotation = getattr(
        car,
        "steering_angle",
        0.0,
    )

    draw_rotated_rectangle(
        surface,
        car,
        centre_x=24,
        centre_y=-17,
        length=14,
        width=7,
        colour=tyre_colour,
        extra_rotation=front_wheel_rotation,
        border_colour=(80, 80, 80),
    )

    draw_rotated_rectangle(
        surface,
        car,
        centre_x=24,
        centre_y=17,
        length=14,
        width=7,
        colour=tyre_colour,
        extra_rotation=front_wheel_rotation,
        border_colour=(80, 80, 80),
    )

    # --------------------------------------------------
    # FRONT WING
    # --------------------------------------------------

    front_wing = [
        transform_car_point(car, 42, -22),
        transform_car_point(car, 47, -22),
        transform_car_point(car, 47, 22),
        transform_car_point(car, 42, 22),
    ]

    pygame.draw.polygon(
        surface,
        body_black,
        front_wing,
    )

    pygame.draw.line(
        surface,
        turquoise,
        transform_car_point(car, 45, -20),
        transform_car_point(car, 45, 20),
        2,
    )

    # Front wing endplates
    pygame.draw.line(
        surface,
        body_silver,
        transform_car_point(car, 42, -22),
        transform_car_point(car, 48, -22),
        3,
    )

    pygame.draw.line(
        surface,
        body_silver,
        transform_car_point(car, 42, 22),
        transform_car_point(car, 48, 22),
        3,
    )

# --------------------------------------------------
# HUD
# --------------------------------------------------

def draw_hud(
    surface: pygame.Surface,
    car,
    font: pygame.font.Font,
    height: int,
    autonomous_mode: bool,
) -> None:
    """
    Draws the speed, steering and mode information.
    """

    hud_surface = pygame.Surface(
        (315, 125),
        pygame.SRCALPHA,
    )

    hud_surface.fill(
        HUD_BACKGROUND
    )

    speed_text = font.render(
        f"Speed: {car.speed:.1f}",
        True,
        WHITE,
    )

    steering_text = font.render(
        (
            "Steering: "
            f"{math.degrees(car.steering_angle):.1f}°"
        ),
        True,
        WHITE,
    )

    if autonomous_mode:
        mode_name = "AUTONOMOUS"
        mode_colour = (0, 230, 210)
    else:
        mode_name = "MANUAL"
        mode_colour = (255, 210, 40)

    mode_text = font.render(
        f"Mode: {mode_name}",
        True,
        mode_colour,
    )

    hud_surface.blit(
        speed_text,
        (15, 12),
    )

    hud_surface.blit(
        steering_text,
        (15, 45),
    )

    hud_surface.blit(
        mode_text,
        (15, 78),
    )

    surface.blit(
        hud_surface,
        (15, 15),
    )

    controls_text = font.render(
        "WASD: drive | M: mode | R: reset | ESC: quit",
        True,
        WHITE,
    )

    controls_background = pygame.Surface(
        (
            controls_text.get_width() + 24,
            controls_text.get_height() + 14,
        ),
        pygame.SRCALPHA,
    )

    controls_background.fill(
        HUD_BACKGROUND
    )

    controls_background.blit(
        controls_text,
        (12, 7),
    )

    surface.blit(
        controls_background,
        (
            15,
            height
            - controls_background.get_height()
            - 15,
        ),
    )


# --------------------------------------------------
# FULL SCENE
# --------------------------------------------------

def draw_scene(
    surface: pygame.Surface,
    car,
    blue_cones: list[tuple[float, float]],
    yellow_cones: list[tuple[float, float]],
    centerline: list[tuple[float, float]],
    width: int,
    height: int,
    font: pygame.font.Font,
    autonomous_mode: bool,
    target_point=None,
    racing_line=None,
) -> None:
    """
    Draws the full simulation scene.
    """

    surface.fill(
        BACKGROUND_COLOUR
    )

    draw_track(
        surface,
        blue_cones,
        yellow_cones,
    )

    draw_centerline(
        surface,
        centerline,
    )

    draw_start_gate(
        surface,
        centerline,
    )

    # target_point is intentionally not drawn.
    # Pure Pursuit still uses it internally.

    draw_car(
        surface,
        car,
    )

    draw_hud(
        surface,
        car,
        font,
        height,
        autonomous_mode,
    )