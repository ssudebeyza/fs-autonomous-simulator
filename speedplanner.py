import math


class CurvatureSpeedPlanner:

    def __init__(
        self,
        max_speed=140.0,
        min_speed=65.0,
    ):
        self.max_speed = max_speed
        self.min_speed = min_speed

    def calculate_target_speed(
        self,
        racing_line,
        nearest_index,
    ):
        point_count = len(racing_line)

        previous_point = racing_line[
            (nearest_index - 3) % point_count
        ]

        current_point = racing_line[
            nearest_index
        ]

        next_point = racing_line[
            (nearest_index + 3) % point_count
        ]

        angle1 = math.atan2(
            current_point[1] - previous_point[1],
            current_point[0] - previous_point[0],
        )

        angle2 = math.atan2(
            next_point[1] - current_point[1],
            next_point[0] - current_point[0],
        )

        curvature = abs(angle2 - angle1)

        while curvature > math.pi:
            curvature -= 2 * math.pi

        curvature = abs(curvature)

        curvature = min(
            curvature,
            1.2,
        )

        ratio = curvature / 1.2

        target_speed = (
            self.max_speed
            - ratio
            * (self.max_speed - self.min_speed)
        )

        return target_speed