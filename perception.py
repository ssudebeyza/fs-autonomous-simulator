import math

from vehicle import PIXELS_PER_METER


class ConePerception:
    def __init__(
        self,
        detection_range=30.0,
        field_of_view=120.0,
    ):
        self.detection_range = detection_range
        self.field_of_view = math.radians(field_of_view)

    def _normalize_angle(self, angle):
        return (
            angle + math.pi
        ) % (
            2.0 * math.pi
        ) - math.pi

    def detect_cones(
        self,
        car,
        cones,
    ):
        detected = []

        max_distance_pixels = (
            self.detection_range
            * PIXELS_PER_METER
        )

        for cone in cones:

            cone_x = cone[0]
            cone_y = cone[1]

            dx = cone_x - car.x
            dy = car.y - cone_y

            distance = math.hypot(
                dx,
                dy,
            )

            if distance > max_distance_pixels:
                continue

            angle_to_cone = math.atan2(
                dy,
                dx,
            )

            relative_angle = (
                self._normalize_angle(
                    angle_to_cone
                    - car.yaw
                )
            )

            if (
                abs(relative_angle)
                <= self.field_of_view / 2.0
            ):
                detected.append(cone)

        return detected

    def detect_track_cones(
        self,
        car,
        blue_cones,
        yellow_cones,
    ):
        detected_blue = (
            self.detect_cones(
                car,
                blue_cones,
            )
        )

        detected_yellow = (
            self.detect_cones(
                car,
                yellow_cones,
            )
        )

        return (
            detected_blue,
            detected_yellow,
        )