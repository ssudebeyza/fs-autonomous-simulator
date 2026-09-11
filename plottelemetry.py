import csv
import math
import os

import matplotlib.pyplot as plt

from track import (
    create_centerline,
    create_oval_track,
    create_racing_line,
)


CSV_FILE = "telemetry.csv"

OUTPUT_DIRECTORY = (
    "telemetry_plots"
)

SCREEN_WIDTH = 1400
SCREEN_HEIGHT = 900


# ============================================================
# LOAD
# ============================================================

def load_telemetry(
    filename,
):
    data = {
        "time": [],
        "speed": [],
        "target_speed": [],
        "steering_angle": [],
        "x": [],
        "y": [],
        "yaw": [],
        "nearest_index": [],
        "reference_cte": [],
        "local_path_error": [],
        "path_source": [],
    }

    with open(
        filename,
        "r",
        newline="",
    ) as file:

        reader = csv.DictReader(
            file
        )

        for row in reader:

            data["time"].append(
                float(row["time"])
            )

            data["speed"].append(
                float(row["speed"])
            )

            data["target_speed"].append(
                float(row["target_speed"])
            )

            data["steering_angle"].append(
                float(
                    row["steering_angle"]
                )
            )

            data["x"].append(
                float(row["x"])
            )

            data["y"].append(
                float(row["y"])
            )

            data["yaw"].append(
                float(row["yaw"])
            )

            data["nearest_index"].append(
                int(
                    float(
                        row["nearest_index"]
                    )
                )
            )

            data["reference_cte"].append(
                float(
                    row["reference_cte"]
                )
            )

            data["local_path_error"].append(
                float(
                    row["local_path_error"]
                )
            )

            data["path_source"].append(
                row["path_source"]
            )

    return data


# ============================================================
# STATISTICS
# ============================================================

def calculate_statistics(
    values,
):
    valid_values = [
        value
        for value in values
        if not math.isnan(value)
    ]

    if not valid_values:
        return (
            0.0,
            0.0,
            0.0,
        )

    mean_value = (
        sum(valid_values)
        / len(valid_values)
    )

    maximum_value = max(
        valid_values
    )

    rmse = math.sqrt(
        sum(
            value * value
            for value in valid_values
        )
        / len(valid_values)
    )

    return (
        mean_value,
        maximum_value,
        rmse,
    )


# ============================================================
# MAIN
# ============================================================

def main():

    if not os.path.exists(
        CSV_FILE
    ):

        print(
            f"{CSV_FILE} not found."
        )

        return


    os.makedirs(
        OUTPUT_DIRECTORY,
        exist_ok=True,
    )


    data = load_telemetry(
        CSV_FILE
    )


    if not data["time"]:

        print(
            "No telemetry samples."
        )

        return


    # ========================================================
    # STATISTICS
    # ========================================================

    (
        mean_reference_cte,
        maximum_reference_cte,
        reference_rmse,
    ) = calculate_statistics(
        data["reference_cte"]
    )


    (
        mean_local_error,
        maximum_local_error,
        local_rmse,
    ) = calculate_statistics(
        data["local_path_error"]
    )


    valid_local_samples = sum(
        1
        for value in data[
            "local_path_error"
        ]
        if not math.isnan(value)
    )


    print()

    print(
        "Telemetry Analysis"
    )

    print(
        "----------------------------"
    )

    print(
        f"Samples: "
        f"{len(data['time'])}"
    )

    print()


    print(
        "Reference Racing Line"
    )

    print(
        f"Mean CTE: "
        f"{mean_reference_cte:.3f} m"
    )

    print(
        f"Maximum CTE: "
        f"{maximum_reference_cte:.3f} m"
    )

    print(
        f"CTE RMSE: "
        f"{reference_rmse:.3f} m"
    )

    print()


    print(
        "Cone Local Path Tracking"
    )

    print(
        f"Valid Samples: "
        f"{valid_local_samples}"
    )

    print(
        f"Mean Error: "
        f"{mean_local_error:.3f} m"
    )

    print(
        f"Maximum Error: "
        f"{maximum_local_error:.3f} m"
    )

    print(
        f"Error RMSE: "
        f"{local_rmse:.3f} m"
    )

    print()


    # ========================================================
    # SPEED
    # ========================================================

    plt.figure(
        figsize=(12, 6)
    )

    plt.plot(
        data["time"],
        data["speed"],
        label="Actual Speed",
    )

    plt.plot(
        data["time"],
        data["target_speed"],
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
            OUTPUT_DIRECTORY,
            "speed_tracking.png",
        )
    )


    # ========================================================
    # STEERING
    # ========================================================

    plt.figure(
        figsize=(12, 6)
    )

    plt.plot(
        data["time"],
        data["steering_angle"],
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
            OUTPUT_DIRECTORY,
            "steering_response.png",
        )
    )


    # ========================================================
    # ERRORS
    # ========================================================

    plt.figure(
        figsize=(12, 6)
    )


    plt.plot(
        data["time"],
        data["reference_cte"],
        label="Reference Racing Line CTE",
    )


    valid_local_time = []

    valid_local_error = []


    for time_value, error_value in zip(
        data["time"],
        data["local_path_error"],
    ):

        if not math.isnan(
            error_value
        ):

            valid_local_time.append(
                time_value
            )

            valid_local_error.append(
                error_value
            )


    plt.plot(
        valid_local_time,
        valid_local_error,
        label="Cone Local Path Tracking Error",
    )


    plt.xlabel(
        "Time (s)"
    )

    plt.ylabel(
        "Error (m)"
    )

    plt.title(
        "Reference CTE vs Cone Local Path Tracking Error"
    )

    plt.legend()

    plt.grid(True)

    plt.tight_layout()

    plt.savefig(
        os.path.join(
            OUTPUT_DIRECTORY,
            "path_errors.png",
        )
    )


    # ========================================================
    # TRAJECTORY
    # ========================================================

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


    racing_line = (
        create_racing_line(
            centerline,
            offset_strength=18.0,
            smoothing_passes=4,
        )
    )


    reference_x = [
        point[0]
        for point in racing_line
    ]

    reference_y = [
        point[1]
        for point in racing_line
    ]


    plt.figure(
        figsize=(12, 8)
    )


    plt.plot(
        data["x"],
        data["y"],
        label="Vehicle Trajectory",
    )


    plt.plot(
        reference_x,
        reference_y,
        linestyle="--",
        label="Reference Racing Line",
    )


    plt.xlabel(
        "Position X (px)"
    )

    plt.ylabel(
        "Position Y (px)"
    )

    plt.title(
        "Formula Student Vehicle Trajectory"
    )

    plt.legend()

    plt.grid(True)

    plt.axis(
        "equal"
    )

    plt.gca().invert_yaxis()

    plt.tight_layout()

    plt.savefig(
        os.path.join(
            OUTPUT_DIRECTORY,
            "trajectory.png",
        )
    )


    print(
        "Plots saved to:"
    )

    print(
        OUTPUT_DIRECTORY
    )


    plt.show()


if __name__ == "__main__":
    main()