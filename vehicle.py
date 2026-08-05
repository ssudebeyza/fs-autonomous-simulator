import math

from utils import clamp


class FormulaStudentCar:
    """
    A simple Formula Student vehicle model based on
    the kinematic bicycle model.
    """

    def __init__(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
    ) -> None:
        # Vehicle state
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = 0.0
        self.steering_angle = 0.0

        # Initial state, used by reset()
        self.initial_x = x
        self.initial_y = y
        self.initial_yaw = yaw

        # Vehicle parameters
        self.wheelbase = 80.0

        self.maximum_steering_angle = math.radians(28)
        self.steering_rate = math.radians(75)

        self.acceleration = 220.0
        self.brake_strength = 260.0
        self.rolling_resistance = 45.0

        self.maximum_speed = 360.0
        self.maximum_reverse_speed = -100.0

    def update_speed_manual(
        self,
        throttle: bool,
        brake: bool,
        delta_time: float,
    ) -> None:
        """
        Updates speed using keyboard throttle and brake inputs.
        """

        longitudinal_acceleration = 0.0

        if throttle:
            longitudinal_acceleration += self.acceleration

        if brake:
            if self.speed > 0.0:
                longitudinal_acceleration -= self.brake_strength
            else:
                longitudinal_acceleration -= self.acceleration * 0.6

        if not throttle and not brake:
            if self.speed > 0.0:
                longitudinal_acceleration -= self.rolling_resistance

            elif self.speed < 0.0:
                longitudinal_acceleration += self.rolling_resistance

        self.speed += longitudinal_acceleration * delta_time

        self.speed = clamp(
            self.speed,
            self.maximum_reverse_speed,
            self.maximum_speed,
        )

        if (
            not throttle
            and not brake
            and abs(self.speed) < 3.0
        ):
            self.speed = 0.0

    def update_speed_automatic(
        self,
        longitudinal_acceleration: float,
        delta_time: float,
    ) -> None:
        """
        Updates vehicle speed using an acceleration command
        supplied by an automatic controller.
        """

        longitudinal_acceleration = clamp(
            longitudinal_acceleration,
            -self.brake_strength,
            self.acceleration,
        )

        self.speed += longitudinal_acceleration * delta_time

        self.speed = clamp(
            self.speed,
            self.maximum_reverse_speed,
            self.maximum_speed,
        )

    def update_steering_manual(
        self,
        steer_left: bool,
        steer_right: bool,
        delta_time: float,
    ) -> None:
        """
        Updates steering angle using keyboard inputs.
        """

        if steer_left:
            self.steering_angle += self.steering_rate * delta_time

        elif steer_right:
            self.steering_angle -= self.steering_rate * delta_time

        else:
            self.return_steering_to_centre(delta_time)

        self.steering_angle = clamp(
            self.steering_angle,
            -self.maximum_steering_angle,
            self.maximum_steering_angle,
        )

    def set_steering_angle(
        self,
        desired_steering_angle: float,
    ) -> None:
        """
        Sets the steering angle using an automatic controller.
        """

        self.steering_angle = clamp(
            desired_steering_angle,
            -self.maximum_steering_angle,
            self.maximum_steering_angle,
        )

    def return_steering_to_centre(
        self,
        delta_time: float,
    ) -> None:
        """
        Gradually returns steering to zero.
        """

        steering_change = self.steering_rate * delta_time

        if self.steering_angle > 0.0:
            self.steering_angle -= steering_change

            if self.steering_angle < 0.0:
                self.steering_angle = 0.0

        elif self.steering_angle < 0.0:
            self.steering_angle += steering_change

            if self.steering_angle > 0.0:
                self.steering_angle = 0.0

    def update_position(
        self,
        delta_time: float,
    ) -> None:
        """
        Updates position and heading with a kinematic
        bicycle model.
        """

        yaw_rate = (
            self.speed
            / self.wheelbase
            * math.tan(self.steering_angle)
        )

        self.yaw += yaw_rate * delta_time

        self.x += (
            self.speed
            * math.cos(self.yaw)
            * delta_time
        )

        self.y -= (
            self.speed
            * math.sin(self.yaw)
            * delta_time
        )

    def reset(self) -> None:
        """
        Restores the vehicle to its initial state.
        """

        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw = self.initial_yaw

        self.speed = 0.0
        self.steering_angle = 0.0