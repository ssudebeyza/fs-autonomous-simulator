import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize

from qss.vehiclemodel import QSSVehicleModel
from qss.trackmodel import QSSTrackModel
from qss.solver import QSSSolver


def prepare_qss():
    vehicle = QSSVehicleModel()

    track = QSSTrackModel()
    track.load_simulator_track()

    solver = QSSSolver(
        vehicle=vehicle,
        track=track,
    )

    solver.solve()

    return vehicle, track, solver


def plot_speed_profile(track, solver):
    distance = np.array(
        track.cumulative_distance
    )

    speed = np.array(
        solver.speed_profile
    )

    corner_limit = np.array(
        solver.corner_speed_limit
    )

    plt.figure(figsize=(11, 6))

    plt.plot(
        distance,
        corner_limit,
        linestyle="--",
        label="Curvature Speed Limit",
    )

    plt.plot(
        distance,
        speed,
        linewidth=2,
        label="QSS Feasible Speed",
    )

    minimum_index = int(
        np.argmin(speed)
    )

    maximum_index = int(
        np.argmax(speed)
    )

    plt.scatter(
        distance[minimum_index],
        speed[minimum_index],
        s=70,
        label="Minimum Speed",
    )

    plt.scatter(
        distance[maximum_index],
        speed[maximum_index],
        s=70,
        label="Maximum Speed",
    )

    plt.annotate(
        f"{speed[minimum_index]:.2f} m/s",
        (
            distance[minimum_index],
            speed[minimum_index],
        ),
        xytext=(10, 15),
        textcoords="offset points",
    )

    plt.annotate(
        f"{speed[maximum_index]:.2f} m/s",
        (
            distance[maximum_index],
            speed[maximum_index],
        ),
        xytext=(10, -25),
        textcoords="offset points",
    )

    plt.xlabel("Distance Around Track [m]")
    plt.ylabel("Speed [m/s]")

    plt.title(
        "Formula Student QSS Speed Profile"
    )

    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        "qss_speed_profile.png",
        dpi=200,
    )

    plt.show()


def plot_curvature(track):
    distance = np.array(
        track.cumulative_distance
    )

    curvature = np.array(
        track.curvature
    )

    plt.figure(figsize=(11, 6))

    plt.plot(
        distance,
        curvature,
        linewidth=1.8,
    )

    plt.axhline(
        0.0,
        linewidth=1,
    )

    max_index = int(
        np.argmax(
            np.abs(curvature)
        )
    )

    plt.scatter(
        distance[max_index],
        curvature[max_index],
        s=70,
        label="Maximum |Curvature|",
    )

    plt.annotate(
        f"{curvature[max_index]:.3f} 1/m",
        (
            distance[max_index],
            curvature[max_index],
        ),
        xytext=(10, 15),
        textcoords="offset points",
    )

    plt.xlabel("Distance Around Track [m]")
    plt.ylabel("Curvature [1/m]")

    plt.title(
        "Formula Student Track Curvature"
    )

    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        "qss_curvature.png",
        dpi=200,
    )

    plt.show()


def plot_speed_map(track, solver):
    points = np.array(
        track.points
    )

    speed = np.array(
        solver.speed_profile
    )

    # Add first point again so the closed
    # circuit is displayed completely.
    closed_points = np.vstack(
        [
            points,
            points[0],
        ]
    )

    closed_speed = np.append(
        speed,
        speed[0],
    )

    segments = np.stack(
        [
            closed_points[:-1],
            closed_points[1:],
        ],
        axis=1,
    )

    segment_speed = (
        closed_speed[:-1]
        + closed_speed[1:]
    ) / 2.0

    normalization = Normalize(
        vmin=float(np.min(speed)),
        vmax=float(np.max(speed)),
    )

    line_collection = LineCollection(
        segments,
        cmap="viridis",
        norm=normalization,
    )

    line_collection.set_array(
        segment_speed
    )

    line_collection.set_linewidth(
        5
    )

    figure, axis = plt.subplots(
        figsize=(10, 8)
    )

    axis.add_collection(
        line_collection
    )

    axis.autoscale()
    axis.set_aspect(
        "equal",
        adjustable="box",
    )

    minimum_index = int(
        np.argmin(speed)
    )

    maximum_index = int(
        np.argmax(speed)
    )

    axis.scatter(
        points[minimum_index, 0],
        points[minimum_index, 1],
        s=80,
        label=(
            f"Slowest: "
            f"{speed[minimum_index]:.2f} m/s"
        ),
    )

    axis.scatter(
        points[maximum_index, 0],
        points[maximum_index, 1],
        s=80,
        label=(
            f"Fastest: "
            f"{speed[maximum_index]:.2f} m/s"
        ),
    )

    colorbar = figure.colorbar(
        line_collection,
        ax=axis,
    )

    colorbar.set_label(
        "QSS Speed [m/s]"
    )

    axis.set_xlabel("X Position [m]")
    axis.set_ylabel("Y Position [m]")

    axis.set_title(
        "Formula Student QSS Track Speed Map"
    )

    axis.legend()
    axis.grid(True)

    figure.tight_layout()

    figure.savefig(
        "qss_speed_map.png",
        dpi=200,
    )

    plt.show()


def print_analysis(track, solver):
    speed = np.array(
        solver.speed_profile
    )

    curvature = np.array(
        track.curvature
    )

    distance = np.array(
        track.cumulative_distance
    )

    minimum_index = int(
        np.argmin(speed)
    )

    maximum_index = int(
        np.argmax(speed)
    )

    maximum_curvature_index = int(
        np.argmax(
            np.abs(curvature)
        )
    )

    print()
    print("QSS PERFORMANCE ANALYSIS")
    print("========================")

    print(
        f"Theoretical lap time: "
        f"{solver.lap_time:.3f} s"
    )

    print(
        f"Average speed: "
        f"{solver.average_speed:.2f} m/s"
    )

    print(
        f"Minimum speed: "
        f"{speed[minimum_index]:.2f} m/s "
        f"at {distance[minimum_index]:.1f} m"
    )

    print(
        f"Maximum speed: "
        f"{speed[maximum_index]:.2f} m/s "
        f"at {distance[maximum_index]:.1f} m"
    )

    print(
        f"Maximum |curvature|: "
        f"{abs(curvature[maximum_curvature_index]):.4f} 1/m "
        f"at {distance[maximum_curvature_index]:.1f} m"
    )


def main():
    print()
    print("FORMULA STUDENT QSS ANALYSIS")
    print("============================")

    _, track, solver = prepare_qss()

    solver.print_summary()

    print_analysis(
        track,
        solver,
    )

    plot_speed_profile(
        track,
        solver,
    )

    plot_curvature(
        track,
    )

    plot_speed_map(
        track,
        solver,
    )


if __name__ == "__main__":
    main()