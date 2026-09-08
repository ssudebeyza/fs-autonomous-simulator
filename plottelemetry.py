import csv
import math
import os

import matplotlib.pyplot as plt

from track import (
    create_oval_track,
    create_centerline,
    create_racing_line,
)


# --------------------------------------------------
# SETTINGS
# --------------------------------------------------

TRACK_WIDTH = 1400
TRACK_HEIGHT = 900

OUTPUT_FOLDER = "telemetry_plots"


# --------------------------------------------------
# HELPER FUNCTION
# Point-to-segment distance
# --------------------------------------------------

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

    segment_length_squared = (
        dx * dx
        + dy * dy
    )

    if segment_length_squared == 0:
        return math.hypot(
            px - x1,
            py - y1,
        )

    t = (
        (
            (px - x1) * dx
            + (py - y1) * dy
        )
        / segment_length_squared
    )

    t = max(
        0.0,
        min(
            1.0,
            t,
        ),
    )

    closest_x = (
        x1
        + t * dx
    )

    closest_y = (
        y1
        + t * dy
    )

    return math.hypot(
        px - closest_x,
        py - closest_y,
    )


def calculate_cross_track_error(
    px,
    py,
    racing_line,
):
    if len(racing_line) < 2:
        return 0.0

    minimum_distance = float(
        "inf"
    )

    for index in range(
        len(racing_line)
    ):
        point_1 = racing_line[
            index
        ]

        point_2 = racing_line[
            (index + 1)
            % len(racing_line)
        ]

        distance = (
            point_to_segment_distance(
                px,
                py,
                point_1[0],
                point_1[1],
                point_2[0],
                point_2[1],
            )
        )

        if (
            distance
            < minimum_distance
        ):
            minimum_distance = (
                distance
            )

    return minimum_distance


# --------------------------------------------------
# CREATE OUTPUT FOLDER
# --------------------------------------------------

os.makedirs(
    OUTPUT_FOLDER,
    exist_ok=True,
)


# --------------------------------------------------
# RECREATE TRACK AND RACING LINE
# --------------------------------------------------

blue_cones, yellow_cones = create_oval_track(
    width=TRACK_WIDTH,
    height=TRACK_HEIGHT,
)

centerline = create_centerline(
    blue_cones,
    yellow_cones,
)

racing_line = create_racing_line(
    centerline,
)


# --------------------------------------------------
# TELEMETRY DATA
# --------------------------------------------------

time_data = []
speed_data = []
target_speed_data = []
steering_data = []
x_data = []
y_data = []

cross_track_error_data = []


# --------------------------------------------------
# READ TELEMETRY CSV
# --------------------------------------------------

with open(
    "telemetry.csv",
    "r",
    newline="",
) as file:

    reader = csv.DictReader(
        file
    )

    for row in reader:
        time_value = float(
            row["time"]
        )

        speed_value = float(
            row["speed"]
        )

        target_speed_value = float(
            row["target_speed"]
        )

        steering_value = float(
            row["steering_angle"]
        )

        x_value = float(
            row["x"]
        )

        y_value = float(
            row["y"]
        )

        time_data.append(
            time_value
        )

        speed_data.append(
            speed_value
        )

        target_speed_data.append(
            target_speed_value
        )

        steering_data.append(
            steering_value
        )

        x_data.append(
            x_value
        )

        y_data.append(
            y_value
        )

        cross_track_error = (
            calculate_cross_track_error(
                x_value,
                y_value,
                racing_line,
            )
        )

        cross_track_error_data.append(
            cross_track_error
        )


# --------------------------------------------------
# FIGURE 1
# SPEED TRACKING
# --------------------------------------------------

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    time_data,
    speed_data,
    label="Actual Speed",
)

plt.plot(
    time_data,
    target_speed_data,
    label="Target Speed",
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Speed (m/s)"
)

plt.title(
    "Formula Student Speed Tracking"
)

plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig(
    os.path.join(
        OUTPUT_FOLDER,
        "speed_tracking.png",
    ),
    dpi=300,
)


# --------------------------------------------------
# FIGURE 2
# STEERING RESPONSE
# --------------------------------------------------

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    time_data,
    steering_data,
    label="Steering Angle",
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Steering Angle (rad)"
)

plt.title(
    "Formula Student Steering Response"
)

plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig(
    os.path.join(
        OUTPUT_FOLDER,
        "steering_response.png",
    ),
    dpi=300,
)


# --------------------------------------------------
# FIGURE 3
# VEHICLE TRAJECTORY
# --------------------------------------------------

plt.figure(
    figsize=(9, 7)
)

plt.plot(
    x_data,
    y_data,
    label="Vehicle Trajectory",
)

racing_x = [
    point[0]
    for point in racing_line
]

racing_y = [
    point[1]
    for point in racing_line
]

plt.plot(
    racing_x,
    racing_y,
    linestyle="--",
    label="Reference Racing Line",
)

plt.xlabel(
    "X Position"
)

plt.ylabel(
    "Y Position"
)

plt.title(
    "Formula Student Vehicle Trajectory"
)

plt.axis(
    "equal"
)

plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig(
    os.path.join(
        OUTPUT_FOLDER,
        "vehicle_trajectory.png",
    ),
    dpi=300,
)


# --------------------------------------------------
# FIGURE 4
# CROSS-TRACK ERROR
# --------------------------------------------------

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    time_data,
    cross_track_error_data,
    label="Cross-Track Error",
)

plt.xlabel(
    "Time (s)"
)

plt.ylabel(
    "Tracking Error (simulation units)"
)

plt.title(
    "Formula Student Cross-Track Error"
)

plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig(
    os.path.join(
        OUTPUT_FOLDER,
        "cross_track_error.png",
    ),
    dpi=300,
)


# --------------------------------------------------
# STATISTICS
# --------------------------------------------------

if (
    len(
        cross_track_error_data
    )
    > 0
):
    mean_cross_track_error = (
        sum(
            cross_track_error_data
        )
        / len(
            cross_track_error_data
        )
    )

    maximum_cross_track_error = max(
        cross_track_error_data
    )

    rmse_cross_track_error = math.sqrt(
        sum(
            error ** 2
            for error in cross_track_error_data
        )
        / len(
            cross_track_error_data
        )
    )

    print()
    print(
        "Telemetry Analysis"
    )

    print(
        "------------------"
    )

    print(
        f"Samples: "
        f"{len(time_data)}"
    )

    print(
        f"Mean Cross-Track Error: "
        f"{mean_cross_track_error:.2f}"
    )

    print(
        f"Maximum Cross-Track Error: "
        f"{maximum_cross_track_error:.2f}"
    )

    print(
        f"Cross-Track Error RMSE: "
        f"{rmse_cross_track_error:.2f}"
    )

    print()
    print(
        "Plots saved to:"
    )

    print(
        OUTPUT_FOLDER
    )


# --------------------------------------------------
# SHOW ALL FIGURES
# --------------------------------------------------

plt.show()