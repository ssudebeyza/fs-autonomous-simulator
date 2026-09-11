import math


# --------------------------------------------------
# BASIC HELPERS
# --------------------------------------------------

def distance(point_a, point_b):
    return math.hypot(
        point_b[0] - point_a[0],
        point_b[1] - point_a[1],
    )


def interpolate_catmull_rom(
    p0,
    p1,
    p2,
    p3,
    t,
):
    t2 = t * t
    t3 = t2 * t

    x = 0.5 * (
        (2.0 * p1[0])
        + (-p0[0] + p2[0]) * t
        + (
            2.0 * p0[0]
            - 5.0 * p1[0]
            + 4.0 * p2[0]
            - p3[0]
        ) * t2
        + (
            -p0[0]
            + 3.0 * p1[0]
            - 3.0 * p2[0]
            + p3[0]
        ) * t3
    )

    y = 0.5 * (
        (2.0 * p1[1])
        + (-p0[1] + p2[1]) * t
        + (
            2.0 * p0[1]
            - 5.0 * p1[1]
            + 4.0 * p2[1]
            - p3[1]
        ) * t2
        + (
            -p0[1]
            + 3.0 * p1[1]
            - 3.0 * p2[1]
            + p3[1]
        ) * t3
    )

    return x, y


# --------------------------------------------------
# TRACK CENTRE SHAPE
# --------------------------------------------------

def create_track_control_points(
    width,
    height,
):
    """
    Smooth Formula Student-style technical circuit.

    Includes:
    - start/finish straight
    - fast sweepers
    - slalom
    - hairpin
    - technical S section
    - long lower sweeper
    """

    return [
        # Start / finish
        (0.76 * width, 0.50 * height),

        # Upper-right sweeper
        (0.80 * width, 0.39 * height),
        (0.76 * width, 0.27 * height),
        (0.67 * width, 0.18 * height),

        # Upper technical section
        (0.56 * width, 0.16 * height),
        (0.48 * width, 0.22 * height),
        (0.40 * width, 0.17 * height),
        (0.31 * width, 0.22 * height),

        # Left hairpin
        (0.20 * width, 0.20 * height),
        (0.13 * width, 0.29 * height),
        (0.12 * width, 0.41 * height),
        (0.18 * width, 0.51 * height),

        # Middle S
        (0.29 * width, 0.55 * height),
        (0.38 * width, 0.49 * height),
        (0.47 * width, 0.55 * height),
        (0.40 * width, 0.65 * height),

        # Lower-left transition
        (0.29 * width, 0.69 * height),
        (0.22 * width, 0.77 * height),

        # Long smooth lower sweeper
        (0.27 * width, 0.84 * height),
        (0.40 * width, 0.87 * height),
        (0.55 * width, 0.86 * height),
        (0.69 * width, 0.82 * height),

        # Lower-right sweeper
        (0.79 * width, 0.74 * height),
        (0.82 * width, 0.64 * height),

        # Final corner
        (0.78 * width, 0.56 * height),
    ]
    return points


# --------------------------------------------------
# SMOOTH CLOSED CENTERLINE
# --------------------------------------------------

def create_smooth_closed_path(
    control_points,
    samples_per_segment=12,
):
    path = []

    count = len(control_points)

    for i in range(count):

        p0 = control_points[
            (i - 1) % count
        ]

        p1 = control_points[
            i
        ]

        p2 = control_points[
            (i + 1) % count
        ]

        p3 = control_points[
            (i + 2) % count
        ]

        for sample in range(
            samples_per_segment
        ):

            t = (
                sample
                / samples_per_segment
            )

            point = (
                interpolate_catmull_rom(
                    p0,
                    p1,
                    p2,
                    p3,
                    t,
                )
            )

            path.append(point)

    return path


# --------------------------------------------------
# CONE GENERATION
# --------------------------------------------------

def create_cones_from_centerline(
    centerline,
    track_half_width=28.0,
    cone_spacing=3,
):
    blue_cones = []
    yellow_cones = []

    count = len(centerline)

    for i in range(
        0,
        count,
        cone_spacing,
    ):

        previous_point = centerline[
            (i - 1) % count
        ]

        next_point = centerline[
            (i + 1) % count
        ]

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
            continue

        tangent_x /= tangent_length
        tangent_y /= tangent_length

        # Normal to the centreline
        normal_x = -tangent_y
        normal_y = tangent_x

        centre_x = centerline[i][0]
        centre_y = centerline[i][1]

        blue_point = (
            centre_x
            + normal_x * track_half_width,
            centre_y
            + normal_y * track_half_width,
        )

        yellow_point = (
            centre_x
            - normal_x * track_half_width,
            centre_y
            - normal_y * track_half_width,
        )

        blue_cones.append(
            blue_point
        )

        yellow_cones.append(
            yellow_point
        )

    return (
        blue_cones,
        yellow_cones,
    )


# --------------------------------------------------
# PUBLIC TRACK FUNCTION
# --------------------------------------------------

def create_oval_track(
    width,
    height,
    cone_count=48,
):
    """
    Function name kept for compatibility with main.py.

    It now creates a technical Formula Student-style
    circuit instead of a simple oval.
    """

    control_points = (
        create_track_control_points(
            width,
            height,
        )
    )

    smooth_centerline = (
        create_smooth_closed_path(
            control_points,
            samples_per_segment=14,
        )
    )

    blue_cones, yellow_cones = (
        create_cones_from_centerline(
            smooth_centerline,
            track_half_width=30.0,
            cone_spacing=4,
        )
    )

    return (
        blue_cones,
        yellow_cones,
    )


# --------------------------------------------------
# CENTERLINE FROM CONES
# --------------------------------------------------

def create_centerline(
    blue_cones,
    yellow_cones,
):
    centerline = []

    pair_count = min(
        len(blue_cones),
        len(yellow_cones),
    )

    for i in range(pair_count):

        center_x = (
            blue_cones[i][0]
            + yellow_cones[i][0]
        ) / 2.0

        center_y = (
            blue_cones[i][1]
            + yellow_cones[i][1]
        ) / 2.0

        centerline.append(
            (
                center_x,
                center_y,
            )
        )

    return centerline


# --------------------------------------------------
# NEAREST POINT
# --------------------------------------------------

def find_nearest_centerline_index(
    x,
    y,
    centerline,
):
    nearest_index = 0
    nearest_distance = float(
        "inf"
    )

    for index, point in enumerate(
        centerline
    ):

        point_distance = math.hypot(
            x - point[0],
            y - point[1],
        )

        if (
            point_distance
            < nearest_distance
        ):
            nearest_distance = (
                point_distance
            )

            nearest_index = index

    return nearest_index


# --------------------------------------------------
# RACING LINE
# --------------------------------------------------

def _signed_turn_angle(
    previous_point,
    current_point,
    next_point,
):
    v1_x = current_point[0] - previous_point[0]
    v1_y = current_point[1] - previous_point[1]

    v2_x = next_point[0] - current_point[0]
    v2_y = next_point[1] - current_point[1]

    length_1 = math.hypot(
        v1_x,
        v1_y,
    )

    length_2 = math.hypot(
        v2_x,
        v2_y,
    )

    if length_1 < 0.001 or length_2 < 0.001:
        return 0.0

    v1_x /= length_1
    v1_y /= length_1

    v2_x /= length_2
    v2_y /= length_2

    cross = (
        v1_x * v2_y
        - v1_y * v2_x
    )

    dot = (
        v1_x * v2_x
        + v1_y * v2_y
    )

    return math.atan2(
        cross,
        dot,
    )


def _interpolate_profile(
    relative_position,
):
    """
    Racing-line profile around a corner.

    -1.0  = outside on entry
     0.0  = inside at apex
    +1.0  = outside on exit
    """

    x = max(
        -1.0,
        min(
            1.0,
            relative_position,
        ),
    )

    if x < -0.5:

        t = (
            x + 1.0
        ) / 0.5

        return (
            -0.55
            + 0.55 * t
        )

    if x < 0.0:

        t = (
            x + 0.5
        ) / 0.5

        return t

    if x < 0.5:

        t = (
            x
            / 0.5
        )

        return (
            1.0 - t
        )

    t = (
        x - 0.5
    ) / 0.5

    return (
        -0.55 * t
    )


def create_racing_line(
    centerline,
    offset_strength=14.0,
    smoothing_passes=4,
):
    """
    Generate an approximate racing line:

    outside -> apex -> outside

    The line remains close enough to the
    centreline to stay inside the cone boundaries.
    """

    count = len(centerline)

    if count < 7:
        return list(centerline)

    # --------------------------------------------------
    # CURVATURE
    # --------------------------------------------------

    curvature = []

    for i in range(count):

        previous_point = centerline[
            (i - 1) % count
        ]

        current_point = centerline[i]

        next_point = centerline[
            (i + 1) % count
        ]

        angle = _signed_turn_angle(
            previous_point,
            current_point,
            next_point,
        )

        curvature.append(angle)


    # --------------------------------------------------
    # SMOOTH CURVATURE
    # --------------------------------------------------

    for _ in range(2):

        smoothed = []

        for i in range(count):

            value = (
                curvature[
                    (i - 1) % count
                ]
                + 2.0
                * curvature[i]
                + curvature[
                    (i + 1) % count
                ]
            ) / 4.0

            smoothed.append(value)

        curvature = smoothed


    # --------------------------------------------------
    # FIND CORNER APEXES
    # --------------------------------------------------

    apexes = []

    curvature_threshold = 0.055
    minimum_apex_gap = 4

    for i in range(count):

        current_strength = abs(
            curvature[i]
        )

        previous_strength = abs(
            curvature[
                (i - 1) % count
            ]
        )

        next_strength = abs(
            curvature[
                (i + 1) % count
            ]
        )

        if (
            current_strength
            > curvature_threshold
            and current_strength
            >= previous_strength
            and current_strength
            >= next_strength
        ):

            if apexes:

                previous_apex = apexes[-1]

                gap = (
                    i - previous_apex
                ) % count

                if gap < minimum_apex_gap:

                    if (
                        current_strength
                        > abs(
                            curvature[
                                previous_apex
                            ]
                        )
                    ):
                        apexes[-1] = i

                    continue

            apexes.append(i)


    # --------------------------------------------------
    # OFFSET PROFILE
    # --------------------------------------------------

    offsets = [
        0.0
        for _ in range(count)
    ]

    corner_window = 6

    for apex_index in apexes:

        apex_curvature = curvature[
            apex_index
        ]

        if abs(apex_curvature) < 0.001:
            continue

        turn_direction = (
            1.0
            if apex_curvature > 0.0
            else -1.0
        )

        strength = min(
            1.0,
            abs(apex_curvature)
            / 0.18,
        )

        for relative_index in range(
            -corner_window,
            corner_window + 1,
        ):

            index = (
                apex_index
                + relative_index
            ) % count

            relative_position = (
                relative_index
                / corner_window
            )

            profile = _interpolate_profile(
                relative_position
            )

            desired_offset = (
                turn_direction
                * profile
                * offset_strength
                * strength
            )

            offsets[index] += (
                desired_offset
            )


    # --------------------------------------------------
    # LIMIT OFFSETS
    # --------------------------------------------------

    maximum_offset = (
        offset_strength
    )

    for i in range(count):

        offsets[i] = max(
            -maximum_offset,
            min(
                maximum_offset,
                offsets[i],
            ),
        )


    # --------------------------------------------------
    # SMOOTH OFFSET TRANSITIONS
    # --------------------------------------------------

    for _ in range(
        smoothing_passes
    ):

        smoothed_offsets = []

        for i in range(count):

            value = (
                offsets[
                    (i - 1) % count
                ]
                + 3.0
                * offsets[i]
                + offsets[
                    (i + 1) % count
                ]
            ) / 5.0

            smoothed_offsets.append(
                value
            )

        offsets = smoothed_offsets


    # --------------------------------------------------
    # MOVE CENTERLINE ALONG LOCAL NORMAL
    # --------------------------------------------------

    racing_line = []

    for i in range(count):

        previous_point = centerline[
            (i - 1) % count
        ]

        next_point = centerline[
            (i + 1) % count
        ]

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

        if tangent_length < 0.001:

            racing_line.append(
                centerline[i]
            )

            continue

        tangent_x /= tangent_length
        tangent_y /= tangent_length

        normal_x = -tangent_y
        normal_y = tangent_x

        racing_x = (
            centerline[i][0]
            + normal_x
            * offsets[i]
        )

        racing_y = (
            centerline[i][1]
            + normal_y
            * offsets[i]
        )

        racing_line.append(
            (
                racing_x,
                racing_y,
            )
        )


    # --------------------------------------------------
    # FINAL LIGHT SMOOTHING
    # --------------------------------------------------

    for _ in range(2):

        smoothed_line = []

        for i in range(count):

            previous_point = racing_line[
                (i - 1) % count
            ]

            current_point = racing_line[i]

            next_point = racing_line[
                (i + 1) % count
            ]

            x = (
                previous_point[0]
                + 6.0
                * current_point[0]
                + next_point[0]
            ) / 8.0

            y = (
                previous_point[1]
                + 6.0
                * current_point[1]
                + next_point[1]
            ) / 8.0

            smoothed_line.append(
                (
                    x,
                    y,
                )
            )

        racing_line = smoothed_line


    return racing_line