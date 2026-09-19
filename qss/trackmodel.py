import math

from track import create_oval_track, create_centerline


PIXELS_PER_METER = 8.0


class QSSTrackModel:
    """
    Converts the simulator track into a metric representation
    suitable for Quasi-Steady-State lap simulation.

    Calculates:
        - x/y position [m]
        - segment distance [m]
        - cumulative distance [m]
        - curvature [1/m]
    """

    def __init__(
        self,
        screen_width=1400,
        screen_height=900,
        pixels_per_meter=PIXELS_PER_METER,
    ):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.pixels_per_meter = pixels_per_meter

        self.points = []
        self.segment_lengths = []
        self.cumulative_distance = []
        self.curvature = []

    # ---------------------------------------------------------
    # LOAD SIMULATOR TRACK
    # ---------------------------------------------------------

    def load_simulator_track(self):
        blue_cones, yellow_cones = create_oval_track(
            self.screen_width,
            self.screen_height,
        )

        centerline_pixels = create_centerline(
            blue_cones,
            yellow_cones,
        )

        self.points = []

        for x_pixel, y_pixel in centerline_pixels:
            x_m = (
                x_pixel
                / self.pixels_per_meter
            )

            # Convert screen coordinates to normal
            # Cartesian coordinates.
            y_m = (
                -y_pixel
                / self.pixels_per_meter
            )

            self.points.append(
                (x_m, y_m)
            )

        self.calculate_geometry()

        return self.points

    # ---------------------------------------------------------
    # DISTANCE
    # ---------------------------------------------------------

    def calculate_segment_lengths(self):
        self.segment_lengths = []

        number_of_points = len(self.points)

        if number_of_points < 2:
            return

        for i in range(number_of_points):
            j = (i + 1) % number_of_points

            x1, y1 = self.points[i]
            x2, y2 = self.points[j]

            ds = math.hypot(
                x2 - x1,
                y2 - y1,
            )

            self.segment_lengths.append(ds)

    # ---------------------------------------------------------
    # CUMULATIVE DISTANCE
    # ---------------------------------------------------------

    def calculate_cumulative_distance(self):
        self.cumulative_distance = []

        if not self.points:
            return

        distance = 0.0

        for i in range(len(self.points)):
            self.cumulative_distance.append(
                distance
            )

            if i < len(self.segment_lengths):
                distance += (
                    self.segment_lengths[i]
                )

    # ---------------------------------------------------------
    # CURVATURE
    # ---------------------------------------------------------

    def calculate_curvature(self):
        """
        Curvature from three consecutive points.

        For triangle ABC:

            kappa = 4A / (abc)

        where:
            A = triangle area
            a,b,c = side lengths

        Sign is determined by turn direction.
        """

        self.curvature = []

        number_of_points = len(self.points)

        if number_of_points < 3:
            self.curvature = [
                0.0
                for _ in self.points
            ]
            return

        for i in range(number_of_points):
            previous_index = (
                i - 1
            ) % number_of_points

            next_index = (
                i + 1
            ) % number_of_points

            x1, y1 = self.points[
                previous_index
            ]

            x2, y2 = self.points[i]

            x3, y3 = self.points[
                next_index
            ]

            a = math.hypot(
                x2 - x1,
                y2 - y1,
            )

            b = math.hypot(
                x3 - x2,
                y3 - y2,
            )

            c = math.hypot(
                x3 - x1,
                y3 - y1,
            )

            denominator = (
                a * b * c
            )

            if denominator < 1e-9:
                self.curvature.append(
                    0.0
                )
                continue

            cross_product = (
                (x2 - x1)
                * (y3 - y1)
                - (y2 - y1)
                * (x3 - x1)
            )

            triangle_area = (
                abs(cross_product)
                / 2.0
            )

            curvature_magnitude = (
                4.0
                * triangle_area
                / denominator
            )

            if cross_product > 0.0:
                sign = 1.0
            elif cross_product < 0.0:
                sign = -1.0
            else:
                sign = 0.0

            self.curvature.append(
                sign
                * curvature_magnitude
            )

    # ---------------------------------------------------------
    # COMPLETE GEOMETRY
    # ---------------------------------------------------------

    def calculate_geometry(self):
        self.calculate_segment_lengths()
        self.calculate_cumulative_distance()
        self.calculate_curvature()

    # ---------------------------------------------------------
    # TRACK LENGTH
    # ---------------------------------------------------------

    @property
    def track_length(self):
        return sum(
            self.segment_lengths
        )

    # ---------------------------------------------------------
    # MAXIMUM CURVATURE
    # ---------------------------------------------------------

    @property
    def maximum_curvature(self):
        if not self.curvature:
            return 0.0

        return max(
            abs(value)
            for value in self.curvature
        )

    # ---------------------------------------------------------
    # MINIMUM RADIUS
    # ---------------------------------------------------------

    @property
    def minimum_radius(self):
        maximum_curvature = (
            self.maximum_curvature
        )

        if maximum_curvature < 1e-9:
            return float("inf")

        return (
            1.0
            / maximum_curvature
        )

    # ---------------------------------------------------------
    # SUMMARY
    # ---------------------------------------------------------

    def print_summary(self):
        print()
        print("QSS TRACK MODEL")
        print("----------------")

        print(
            f"Track points: "
            f"{len(self.points)}"
        )

        print(
            f"Track length: "
            f"{self.track_length:.2f} m"
        )

        print(
            f"Maximum curvature: "
            f"{self.maximum_curvature:.4f} 1/m"
        )

        print(
            f"Minimum radius: "
            f"{self.minimum_radius:.2f} m"
        )


if __name__ == "__main__":
    track = QSSTrackModel()

    track.load_simulator_track()

    track.print_summary()

    print()
    print("FIRST 10 TRACK POINTS")
    print("---------------------")

    for i in range(
        min(10, len(track.points))
    ):
        x, y = track.points[i]

        print(
            f"{i:3d} | "
            f"x={x:7.2f} m | "
            f"y={y:7.2f} m | "
            f"kappa={track.curvature[i]:8.4f} 1/m"
        )