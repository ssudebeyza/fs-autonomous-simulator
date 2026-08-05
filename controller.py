import math

from utils import clamp, distance_between_points, normalize_angle


class PurePursuitController:
    """
    Pure Pursuit steering controller.
    """

    def __init__(
        self,
        lookahead_distance: float = 70.0,
        wheelbase: float = 35.0,
        max_steering: float = 0.60,
    ) -> None:

        self.lookahead_distance = lookahead_distance
        self.wheelbase = wheelbase
        self.max_steering = max_steering

    def find_nearest_index(
        self,
        car,
        path,
    ) -> int:

        nearest_index = 0
        nearest_distance = float("inf")

        for index, point in enumerate(path):

            distance = distance_between_points(
                car.x,
                car.y,
                point[0],
                point[1],
            )

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = index

        return nearest_index

    def find_target_point(
        self,
        car,
        path,
    ):

        nearest_index = self.find_nearest_index(
            car,
            path,
        )

        travelled_distance = 0.0

        previous_point = path[nearest_index]

        for offset in range(
            1,
            len(path) + 1,
        ):

            index = (
                nearest_index + offset
            ) % len(path)

            current_point = path[index]

            travelled_distance += distance_between_points(
                previous_point[0],
                previous_point[1],
                current_point[0],
                current_point[1],
            )

            if (
                travelled_distance
                >= self.lookahead_distance
            ):
                return current_point

            previous_point = current_point

        return path[nearest_index]

    def calculate_steering(
        self,
        car,
        path,
    ):

        target_point = self.find_target_point(
            car,
            path,
        )

        dx = target_point[0] - car.x
        dy = -(target_point[1] - car.y)

        target_heading = math.atan2(
            dy,
            dx,
        )

        alpha = normalize_angle(
            target_heading - car.yaw
        )
        lookahead = max(
            self.lookahead_distance,
            1.0,
        )

        steering_angle = math.atan2(
            2.0
            * self.wheelbase
            * math.sin(alpha),
            lookahead,
        )

        steering_angle = clamp(
            steering_angle,
            -self.max_steering,
            self.max_steering,
        )

        return (
            steering_angle,
            target_point,
        )


class PIDSpeedController:
    """
    PID controller for longitudinal speed.
    """

    def __init__(
        self,
        kp: float = 3.0,
        ki: float = 0.0,
        kd: float = 0.15,
        target_speed: float = 120.0,
    ) -> None:

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.target_speed = target_speed

        self.integral = 0.0
        self.previous_error = 0.0

    def reset(self):

        self.integral = 0.0
        self.previous_error = 0.0

    def calculate_acceleration(
        self,
        current_speed: float,
        delta_time: float,
    ) -> float:

        if delta_time <= 0.0:
            return 0.0

        error = (
            self.target_speed
            - current_speed
        )

        self.integral += (
            error * delta_time
        )

        derivative = (
            error
            - self.previous_error
        ) / delta_time

        self.previous_error = error

        acceleration = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        acceleration = clamp(
            acceleration,
            -8.0,
            6.0,
        )

        return acceleration