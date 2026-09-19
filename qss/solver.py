import math


class QSSSolver:
    """
    Quasi-Steady-State lap time solver.

    Uses:
        - track curvature
        - lateral acceleration limit
        - longitudinal acceleration limit
        - braking limit
        - vehicle maximum speed

    to calculate the maximum feasible speed profile.
    """

    def __init__(self, vehicle, track):
        self.vehicle = vehicle
        self.track = track

        self.corner_speed_limit = []
        self.speed_profile = []

        self.lap_time = 0.0
        self.average_speed = 0.0
        self.maximum_speed = 0.0
        self.minimum_speed = 0.0

    # ---------------------------------------------------------
    # CORNERING SPEED LIMIT
    # ---------------------------------------------------------

    def calculate_corner_speed_limits(self):
        self.corner_speed_limit = []

        for curvature in self.track.curvature:

            speed_limit = (
                self.vehicle.maximum_cornering_speed(
                    curvature
                )
            )

            self.corner_speed_limit.append(
                speed_limit
            )

        self.speed_profile = (
            self.corner_speed_limit.copy()
        )

    # ---------------------------------------------------------
    # BACKWARD BRAKING PASS
    # ---------------------------------------------------------

    def backward_braking_pass(self):
        """
        Work backwards around the track.

        Determines how fast the vehicle may be travelling
        before each corner while still being able to brake
        to the required corner speed.

        Equation:

            v1^2 = v2^2 + 2*a*ds
        """

        number_of_points = len(
            self.speed_profile
        )

        if number_of_points < 2:
            return

        # Several iterations are used because the track
        # is closed.
        for _ in range(3):

            for i in range(
                number_of_points - 1,
                -1,
                -1,
            ):
                next_index = (
                    i + 1
                ) % number_of_points

                ds = (
                    self.track.segment_lengths[i]
                )

                next_speed = (
                    self.speed_profile[next_index]
                )

                braking = (
                    self.vehicle.available_braking(
                        next_speed
                    )
                )

                allowed_speed = math.sqrt(
                    max(
                        0.0,
                        next_speed ** 2
                        + 2.0
                        * braking
                        * ds,
                    )
                )

                self.speed_profile[i] = min(
                    self.speed_profile[i],
                    allowed_speed,
                    self.vehicle.max_speed,
                )

    # ---------------------------------------------------------
    # FORWARD ACCELERATION PASS
    # ---------------------------------------------------------

    def forward_acceleration_pass(self):
        """
        Work forwards around the track.

        Limits the speed according to available vehicle
        acceleration.

        Equation:

            v2^2 = v1^2 + 2*a*ds
        """

        number_of_points = len(
            self.speed_profile
        )

        if number_of_points < 2:
            return

        # Closed track, therefore repeat to converge around
        # the start/finish boundary.
        for _ in range(3):

            for i in range(
                number_of_points
            ):
                next_index = (
                    i + 1
                ) % number_of_points

                ds = (
                    self.track.segment_lengths[i]
                )

                current_speed = (
                    self.speed_profile[i]
                )

                acceleration = (
                    self.vehicle.available_acceleration(
                        current_speed
                    )
                )

                reachable_speed = math.sqrt(
                    max(
                        0.0,
                        current_speed ** 2
                        + 2.0
                        * acceleration
                        * ds,
                    )
                )

                reachable_speed = min(
                    reachable_speed,
                    self.vehicle.max_speed,
                )

                self.speed_profile[next_index] = min(
                    self.speed_profile[next_index],
                    reachable_speed,
                    self.corner_speed_limit[
                        next_index
                    ],
                )

    # ---------------------------------------------------------
    # LAP TIME
    # ---------------------------------------------------------

    def calculate_lap_time(self):
        """
        Integrates time around the complete track.

        For each segment:

            dt = 2*ds / (v1 + v2)

        This is better than simply ds/v because speed may
        change considerably across a segment.
        """

        total_time = 0.0

        number_of_points = len(
            self.speed_profile
        )

        for i in range(
            number_of_points
        ):
            next_index = (
                i + 1
            ) % number_of_points

            v1 = self.speed_profile[i]
            v2 = self.speed_profile[next_index]

            ds = (
                self.track.segment_lengths[i]
            )

            average_segment_speed = (
                0.5 * (v1 + v2)
            )

            if average_segment_speed < 0.01:
                continue

            dt = (
                ds
                / average_segment_speed
            )

            total_time += dt

        self.lap_time = total_time

        if total_time > 0.0:
            self.average_speed = (
                self.track.track_length
                / total_time
            )
        else:
            self.average_speed = 0.0

        if self.speed_profile:
            self.maximum_speed = max(
                self.speed_profile
            )

            self.minimum_speed = min(
                self.speed_profile
            )

    # ---------------------------------------------------------
    # SOLVE
    # ---------------------------------------------------------

    def solve(self):
        print(
            "Calculating cornering limits..."
        )

        self.calculate_corner_speed_limits()

        print(
            "Applying braking constraints..."
        )

        self.backward_braking_pass()

        print(
            "Applying acceleration constraints..."
        )

        self.forward_acceleration_pass()

        # One more braking pass ensures that acceleration
        # changes have not created an impossible approach
        # speed before a corner.
        self.backward_braking_pass()

        self.calculate_lap_time()

        return self.speed_profile

    # ---------------------------------------------------------
    # SUMMARY
    # ---------------------------------------------------------

    def print_summary(self):
        print()
        print("QSS LAP SIMULATION")
        print("------------------")

        print(
            f"Track length: "
            f"{self.track.track_length:.2f} m"
        )

        print(
            f"Theoretical lap time: "
            f"{self.lap_time:.3f} s"
        )

        print(
            f"Average speed: "
            f"{self.average_speed:.2f} m/s"
        )

        print(
            f"Maximum speed: "
            f"{self.maximum_speed:.2f} m/s"
        )

        print(
            f"Minimum speed: "
            f"{self.minimum_speed:.2f} m/s"
        )