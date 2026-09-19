import math


class QSSVehicleModel:
    """
    Simplified Formula Student vehicle model for QSS lap simulation.

    SI units are used throughout:
        mass            kg
        speed           m/s
        acceleration    m/s^2
        force           N
        power           W
        distance        m
    """

    def __init__(
        self,
        mass=250.0,
        max_power=80000.0,
        max_speed=30.0,
        max_acceleration=6.0,
        max_braking=8.0,
        max_lateral_acceleration=10.0,
        drag_coefficient=0.9,
        frontal_area=1.0,
        air_density=1.225,
        rolling_resistance_coefficient=0.015,
    ):
        self.mass = mass

        self.max_power = max_power
        self.max_speed = max_speed

        self.max_acceleration = max_acceleration
        self.max_braking = max_braking
        self.max_lateral_acceleration = max_lateral_acceleration

        self.drag_coefficient = drag_coefficient
        self.frontal_area = frontal_area
        self.air_density = air_density

        self.rolling_resistance_coefficient = (
            rolling_resistance_coefficient
        )

        self.gravity = 9.81

    # ---------------------------------------------------------
    # AERODYNAMIC DRAG
    # ---------------------------------------------------------

    def aerodynamic_drag(self, speed):
        """
        F_drag = 0.5 * rho * Cd * A * v^2
        """

        return (
            0.5
            * self.air_density
            * self.drag_coefficient
            * self.frontal_area
            * speed ** 2
        )

    # ---------------------------------------------------------
    # ROLLING RESISTANCE
    # ---------------------------------------------------------

    def rolling_resistance(self):
        """
        F_rr = Crr * m * g
        """

        return (
            self.rolling_resistance_coefficient
            * self.mass
            * self.gravity
        )

    # ---------------------------------------------------------
    # POWER-LIMITED DRIVE FORCE
    # ---------------------------------------------------------

    def power_limited_drive_force(self, speed):
        """
        P = F * v
        therefore F = P / v.

        At very low speed we avoid division by zero.
        """

        minimum_speed = 1.0

        effective_speed = max(
            speed,
            minimum_speed,
        )

        return (
            self.max_power
            / effective_speed
        )

    # ---------------------------------------------------------
    # LONGITUDINAL ACCELERATION
    # ---------------------------------------------------------

    def available_acceleration(self, speed):
        """
        Calculates maximum forward acceleration at the
        current vehicle speed.

        Acceleration is limited by:
        1. configured traction/acceleration limit
        2. available power
        3. aerodynamic drag
        4. rolling resistance
        """

        if speed >= self.max_speed:
            return 0.0

        drive_force = (
            self.power_limited_drive_force(speed)
        )

        resistance_force = (
            self.aerodynamic_drag(speed)
            + self.rolling_resistance()
        )

        net_force = (
            drive_force
            - resistance_force
        )

        acceleration = (
            net_force
            / self.mass
        )

        acceleration = max(
            0.0,
            acceleration,
        )

        return min(
            acceleration,
            self.max_acceleration,
        )

    # ---------------------------------------------------------
    # BRAKING
    # ---------------------------------------------------------

    def available_braking(self, speed):
        """
        Positive value representing maximum deceleration.

        The first QSS version uses a constant braking limit.
        """

        return self.max_braking

    # ---------------------------------------------------------
    # CORNERING SPEED
    # ---------------------------------------------------------

    def maximum_cornering_speed(self, curvature):
        """
        ay = v^2 * kappa

        Therefore:

        v_max = sqrt(ay_max / |kappa|)
        """

        curvature = abs(curvature)

        if curvature < 1e-6:
            return self.max_speed

        corner_speed = math.sqrt(
            self.max_lateral_acceleration
            / curvature
        )

        return min(
            corner_speed,
            self.max_speed,
        )

    # ---------------------------------------------------------
    # SUMMARY
    # ---------------------------------------------------------

    def print_summary(self):
        print("QSS VEHICLE MODEL")
        print("-----------------")
        print(f"Mass: {self.mass:.1f} kg")
        print(
            f"Maximum power: "
            f"{self.max_power / 1000:.1f} kW"
        )
        print(
            f"Maximum speed: "
            f"{self.max_speed:.1f} m/s"
        )
        print(
            f"Maximum acceleration: "
            f"{self.max_acceleration:.1f} m/s^2"
        )
        print(
            f"Maximum braking: "
            f"{self.max_braking:.1f} m/s^2"
        )
        print(
            f"Maximum lateral acceleration: "
            f"{self.max_lateral_acceleration:.1f} m/s^2"
        )


if __name__ == "__main__":
    vehicle = QSSVehicleModel()

    vehicle.print_summary()

    print("\nTEST")
    print("----")

    test_speeds = [
        5.0,
        10.0,
        15.0,
        20.0,
        25.0,
    ]

    for speed in test_speeds:
        acceleration = (
            vehicle.available_acceleration(speed)
        )

        print(
            f"{speed:5.1f} m/s -> "
            f"{acceleration:.2f} m/s^2"
        )

    test_curvature = 0.10

    corner_speed = (
        vehicle.maximum_cornering_speed(
            test_curvature
        )
    )

    print(
        f"\nCorner speed at "
        f"kappa={test_curvature:.3f} 1/m: "
        f"{corner_speed:.2f} m/s"
    )