import math


# Screen scale
# Vehicle dynamics use SI units:
# speed       -> m/s
# acceleration -> m/s^2
# wheelbase   -> m
# position on screen -> pixels

PIXELS_PER_METER = 8.0


def clamp(
    value: float,
    minimum: float,
    maximum: float,
) -> float:
    return max(
        minimum,
        min(value, maximum),
    )


class FormulaStudentCar:
    """
    Simplified Formula Student vehicle using
    a kinematic bicycle model.
    """

    def __init__(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
    ) -> None:

        # Screen position in pixels
        self.x = x
        self.y = y

        # Vehicle heading in radians
        self.yaw = yaw

        # Longitudinal speed in m/s
        self.speed = 0.0

        # Steering angle in radians
        self.steering_angle = 0.0

        # Physical vehicle parameters
        self.wheelbase = 1.6

        # Steering limits
        self.max_steering_angle = 0.60

        # Longitudinal limits
        self.max_acceleration = 6.0
        self.max_braking = 8.0

        # Speed limits
        self.max_speed = 30.0
        self.max_reverse_speed = 5.0


    def set_steering_angle(
        self,
        desired_steering_angle: float,
    ) -> None:

        self.steering_angle = clamp(
            desired_steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle,
        )


    def update_speed_automatic(
        self,
        longitudinal_acceleration: float,
        delta_time: float,
    ) -> None:

        if delta_time <= 0.0:
            return

        acceleration = clamp(
            longitudinal_acceleration,
            -self.max_braking,
            self.max_acceleration,
        )

        self.speed += (
            acceleration
            * delta_time
        )

        self.speed = clamp(
            self.speed,
            0.0,
            self.max_speed,
        )


    def update_speed_manual(
        self,
        throttle: float,
        brake: float,
        delta_time: float,
    ) -> None:

        if delta_time <= 0.0:
            return

        throttle = clamp(
            throttle,
            0.0,
            1.0,
        )

        brake = clamp(
            brake,
            0.0,
            1.0,
        )

        acceleration = (
            throttle
            * self.max_acceleration
        )

        braking = (
            brake
            * self.max_braking
        )

        self.speed += (
            acceleration
            * delta_time
        )

        self.speed -= (
            braking
            * delta_time
        )

        self.speed = clamp(
            self.speed,
            0.0,
            self.max_speed,
        )


    def update_position(
        self,
        delta_time: float,
    ) -> None:

        if delta_time <= 0.0:
            return

        # Convert physical speed (m/s)
        # to screen movement (pixels/s)
        screen_speed = (
            self.speed
            * PIXELS_PER_METER
        )

        self.x += (
            screen_speed
            * math.cos(self.yaw)
            * delta_time
        )

        self.y -= (
            screen_speed
            * math.sin(self.yaw)
            * delta_time
        )

        # Kinematic bicycle model.
        # speed and wheelbase are both SI units,
        # therefore yaw_rate is rad/s.
        yaw_rate = (
            self.speed
            / self.wheelbase
            * math.tan(
                self.steering_angle
            )
        )

        self.yaw += (
            yaw_rate
            * delta_time
        )

        # Keep yaw bounded
        self.yaw = (
            self.yaw
            + math.pi
        ) % (
            2.0
            * math.pi
        ) - math.pi


    def reset(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
    ) -> None:

        self.x = x
        self.y = y
        self.yaw = yaw

        self.speed = 0.0
        self.steering_angle = 0.0