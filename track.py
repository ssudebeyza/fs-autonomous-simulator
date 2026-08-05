import math


def create_oval_track(
    width: int,
    height: int,
    cone_count: int = 48,
) -> tuple[
    list[tuple[float, float]],
    list[tuple[float, float]],
]:
    """
    Creates an oval Formula Student track.

    Blue cones define the outer boundary.
    Yellow cones define the inner boundary.
    """

    blue_cones = []
    yellow_cones = []

    centre_x = width / 2
    centre_y = height / 2

    outer_radius_x = 430
    outer_radius_y = 270

    inner_radius_x = 320
    inner_radius_y = 170

    for index in range(cone_count):
        angle = (
            2.0
            * math.pi
            * index
            / cone_count
        )

        outer_x = (
            centre_x
            + outer_radius_x
            * math.cos(angle)
        )

        outer_y = (
            centre_y
            + outer_radius_y
            * math.sin(angle)
        )

        inner_x = (
            centre_x
            + inner_radius_x
            * math.cos(angle)
        )

        inner_y = (
            centre_y
            + inner_radius_y
            * math.sin(angle)
        )

        blue_cones.append(
            (outer_x, outer_y)
        )

        yellow_cones.append(
            (inner_x, inner_y)
        )

    return blue_cones, yellow_cones


def create_centerline(
    blue_cones: list[tuple[float, float]],
    yellow_cones: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """
    Creates a centerline by taking the midpoint
    between corresponding blue and yellow cones.
    """

    centerline = []

    for blue_cone, yellow_cone in zip(
        blue_cones,
        yellow_cones,
    ):
        blue_x, blue_y = blue_cone
        yellow_x, yellow_y = yellow_cone

        centre_x = (
            blue_x + yellow_x
        ) / 2.0

        centre_y = (
            blue_y + yellow_y
        ) / 2.0

        centerline.append(
            (centre_x, centre_y)
        )

    return centerline


def find_nearest_centerline_index(
    x: float,
    y: float,
    centerline: list[tuple[float, float]],
) -> int:
    """
    Finds the index of the centerline point
    closest to a given position.
    """

    if not centerline:
        raise ValueError(
            "Centerline cannot be empty."
        )

    nearest_index = 0
    nearest_distance = float("inf")

    for index, point in enumerate(centerline):
        point_x, point_y = point

        distance = math.hypot(
            point_x - x,
            point_y - y,
        )

        if distance < nearest_distance:
            nearest_distance = distance
            nearest_index = index

    return nearest_index

def create_racing_line(
    centerline,
    offset_strength=22.0,
    smoothing_passes=6,
):
    """
    Creates a simple racing line from the centreline.

    The line moves slightly toward the inside of corners
    and is smoothed afterward.
    """

    if len(centerline) < 5:
        return centerline.copy()

    racing_line = []

    point_count = len(centerline)

    for index in range(point_count):
        previous_point = centerline[
            (index - 2) % point_count
        ]

        current_point = centerline[index]

        next_point = centerline[
            (index + 2) % point_count
        ]

        incoming_angle = math.atan2(
            current_point[1] - previous_point[1],
            current_point[0] - previous_point[0],
        )

        outgoing_angle = math.atan2(
            next_point[1] - current_point[1],
            next_point[0] - current_point[0],
        )

        angle_difference = normalize_angle_local(
            outgoing_angle - incoming_angle
        )

        curvature_amount = min(
            abs(angle_difference),
            1.0,
        )

        tangent_x = (
            next_point[0]
            - previous_point[0]
        )

        tangent_y = (
            next_point[1]
            - previous_point[1]
        )

        tangent_length = math.hypot(
            tangent_x,
            tangent_y,
        )

        if tangent_length == 0:
            racing_line.append(
                current_point
            )
            continue

        tangent_x /= tangent_length
        tangent_y /= tangent_length

        normal_x = -tangent_y
        normal_y = tangent_x

        turn_direction = 1.0

        if angle_difference < 0:
            turn_direction = -1.0

        offset = (
            offset_strength
            * curvature_amount
            * turn_direction
        )

        racing_point = (
            current_point[0]
            + normal_x * offset,
            current_point[1]
            + normal_y * offset,
        )

        racing_line.append(
            racing_point
        )

    for _ in range(smoothing_passes):
        smoothed_line = []

        for index in range(point_count):
            previous_point = racing_line[
                (index - 1) % point_count
            ]

            current_point = racing_line[index]

            next_point = racing_line[
                (index + 1) % point_count
            ]

            smoothed_x = (
                previous_point[0]
                + 2.0 * current_point[0]
                + next_point[0]
            ) / 4.0

            smoothed_y = (
                previous_point[1]
                + 2.0 * current_point[1]
                + next_point[1]
            ) / 4.0

            smoothed_line.append(
                (
                    smoothed_x,
                    smoothed_y,
                )
            )

        racing_line = smoothed_line

    return racing_line


def normalize_angle_local(angle):
    """
    Keeps an angle between -pi and pi.
    """

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle