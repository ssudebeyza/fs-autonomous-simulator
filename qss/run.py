from qss.vehiclemodel import QSSVehicleModel
from qss.trackmodel import QSSTrackModel
from qss.solver import QSSSolver


def main():

    print()
    print("FORMULA STUDENT QSS")
    print("===================")

    # Vehicle
    vehicle = QSSVehicleModel()

    # Track
    track = QSSTrackModel()
    track.load_simulator_track()

    # Solver
    solver = QSSSolver(
        vehicle=vehicle,
        track=track,
    )

    solver.solve()

    vehicle.print_summary()
    track.print_summary()
    solver.print_summary()


if __name__ == "__main__":
    main()