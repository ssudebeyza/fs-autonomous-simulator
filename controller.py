import math


def clamp(
    value,
    minimum,
    maximum,
):
    return max(
        minimum,
        min(
            value,
            maximum,
        ),
    )


def normalize_angle(
    angle,
):
    return (
        angle + math.pi
    ) % (
        2.0 * math.pi
    ) - math.pi


# ============================================================
# PURE PURSUIT CONTROLLER
# ============================================================

class PurePursuitController:
    def __init__(
        self,
        lookahead_distance=16.0,
        wheelbase=12.8,
        max_steering=0.60,
        dynamic_lookahead=True,
        lookahead_gain=0.35,
        min_lookahead=8.0,
        max_lookahead=24.0,
        search_backward=4,
        search_forward=18,
    ):
        self.lookahead_distance = (
            lookahead_distance
        )

        self.wheelbase = (
            wheelbase
        )

        self.max_steering = (
            max_steering
        )

        self.dynamic_lookahead = (
            dynamic_lookahead
        )

        self.lookahead_gain = (
            lookahead_gain
        )

        self.min_lookahead = (
            min_lookahead
        )

        self.max_lookahead = (
            max_lookahead
        )

        self.search_backward = (
            search_backward
        )

        self.search_forward = (
            search_forward
        )

        self.previous_nearest_index = None


    # ========================================================
    # RESET
    # ========================================================

    def reset(
        self,
    ):
        self.previous_nearest_index = None


    # ========================================================
    # DYNAMIC LOOKAHEAD
    # ========================================================

    def _calculate_lookahead(
        self,
        speed,
    ):
        if not self.dynamic_lookahead:
            return (
                self.lookahead_distance
            )

        lookahead = (
            self.min_lookahead
            + speed
            * self.lookahead_gain
        )

        return clamp(
            lookahead,
            self.min_lookahead,
            self.max_lookahead,
        )


    # ========================================================
    # GLOBAL / CLOSED PATH
    # FIND NEAREST INDEX
    # ========================================================

    def find_nearest_index(
        self,
        car,
        path,
    ):
        if not path:
            return 0

        path_length = len(path)


        # First search:
        # search all points.
        if self.previous_nearest_index is None:

            nearest_index = 0
            nearest_distance = float("inf")

            for index, point in enumerate(path):

                distance = math.hypot(
                    point[0] - car.x,
                    point[1] - car.y,
                )

                if (
                    distance
                    < nearest_distance
                ):
                    nearest_distance = distance
                    nearest_index = index


            self.previous_nearest_index = (
                nearest_index
            )

            return nearest_index


        # Later searches:
        # only search around previous progress.
        best_index = (
            self.previous_nearest_index
        )

        best_distance = float("inf")


        for offset in range(
            -self.search_backward,
            self.search_forward + 1,
        ):

            index = (
                self.previous_nearest_index
                + offset
            ) % path_length


            point = path[index]


            distance = math.hypot(
                point[0] - car.x,
                point[1] - car.y,
            )


            if (
                distance
                < best_distance
            ):
                best_distance = distance
                best_index = index


        self.previous_nearest_index = (
            best_index
        )

        return best_index


    # ========================================================
    # GLOBAL / CLOSED PATH
    # FIND TARGET
    # ========================================================

    def find_target_point(
        self,
        car,
        path,
    ):
        if not path:
            return None


        if len(path) == 1:
            return path[0]


        nearest_index = (
            self.find_nearest_index(
                car,
                path,
            )
        )


        lookahead = (
            self._calculate_lookahead(
                car.speed
            )
        )


        accumulated_distance = 0.0

        current_index = (
            nearest_index
        )

        path_length = len(path)


        for _ in range(
            path_length
        ):

            next_index = (
                current_index + 1
            ) % path_length


            point_1 = (
                path[current_index]
            )

            point_2 = (
                path[next_index]
            )


            segment_distance = math.hypot(
                point_2[0] - point_1[0],
                point_2[1] - point_1[1],
            )


            accumulated_distance += (
                segment_distance
            )


            if (
                accumulated_distance
                >= lookahead
            ):
                return point_2


            current_index = (
                next_index
            )


        return path[
            nearest_index
        ]


    # ========================================================
    # GLOBAL / CLOSED PATH
    # STEERING
    # ========================================================

    def calculate_steering(
        self,
        car,
        path,
    ):
        if len(path) < 2:
            return 0.0, None


        target_point = (
            self.find_target_point(
                car,
                path,
            )
        )


        if target_point is None:
            return 0.0, None


        dx = (
            target_point[0]
            - car.x
        )

        dy = (
            car.y
            - target_point[1]
        )


        target_heading = math.atan2(
            dy,
            dx,
        )


        alpha = normalize_angle(
            target_heading
            - car.yaw
        )


        distance = max(
            math.hypot(
                dx,
                dy,
            ),
            1.0,
        )


        steering_angle = math.atan2(
            2.0
            * self.wheelbase
            * math.sin(alpha),
            distance,
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


    # ========================================================
    # LOCAL / OPEN PATH
    #
    # Important:
    # - no previous index memory
    # - no modulo
    # - no wrap-around
    # - searches from scratch each frame
    # ========================================================

    def calculate_steering_local(
        self,
        car,
        path,
    ):
        if len(path) < 2:
            return 0.0, None


        # ----------------------------------------------------
        # 1. FIND NEAREST POINT FROM SCRATCH
        # ----------------------------------------------------

        nearest_index = 0
        nearest_distance = float("inf")


        for index, point in enumerate(path):

            distance = math.hypot(
                point[0] - car.x,
                point[1] - car.y,
            )


            if (
                distance
                < nearest_distance
            ):

                nearest_distance = (
                    distance
                )

                nearest_index = (
                    index
                )


        # ----------------------------------------------------
        # 2. LOCAL LOOKAHEAD
        # ----------------------------------------------------

        lookahead = (
            self._calculate_lookahead(
                car.speed
            )
        )


        # ----------------------------------------------------
        # 3. WALK FORWARD ALONG OPEN PATH
        # ----------------------------------------------------

        target_index = (
            nearest_index
        )

        accumulated_distance = 0.0


        for index in range(
            nearest_index,
            len(path) - 1,
        ):

            point_1 = (
                path[index]
            )

            point_2 = (
                path[index + 1]
            )


            segment_distance = math.hypot(
                point_2[0] - point_1[0],
                point_2[1] - point_1[1],
            )


            accumulated_distance += (
                segment_distance
            )


            target_index = (
                index + 1
            )


            if (
                accumulated_distance
                >= lookahead
            ):
                break


        target_point = (
            path[target_index]
        )


        # ----------------------------------------------------
        # 4. TARGET RELATIVE TO VEHICLE
        # ----------------------------------------------------

        dx = (
            target_point[0]
            - car.x
        )

        dy = (
            car.y
            - target_point[1]
        )


        target_heading = math.atan2(
            dy,
            dx,
        )


        alpha = normalize_angle(
            target_heading
            - car.yaw
        )


        actual_distance = max(
            math.hypot(
                dx,
                dy,
            ),
            1.0,
        )


        # ----------------------------------------------------
        # 5. PURE PURSUIT
        # ----------------------------------------------------

        steering_angle = math.atan2(
            2.0
            * self.wheelbase
            * math.sin(alpha),
            actual_distance,
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


# ============================================================
# PID SPEED CONTROLLER
# ============================================================

class PIDSpeedController:
    def __init__(
        self,
        kp=3.0,
        ki=0.0,
        kd=0.15,
        target_speed=20.0,
        integral_limit=10.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.target_speed = (
            target_speed
        )

        self.integral_limit = (
            integral_limit
        )

        self.integral = 0.0

        self.previous_error = None


    # ========================================================
    # RESET
    # ========================================================

    def reset(
        self,
    ):
        self.integral = 0.0

        self.previous_error = None


    # ========================================================
    # PID
    # ========================================================

    def calculate_acceleration(
        self,
        current_speed,
        delta_time,
    ):
        if delta_time <= 0.0:
            return 0.0


        error = (
            self.target_speed
            - current_speed
        )


        # ----------------------------------------------------
        # INTEGRAL
        # ----------------------------------------------------

        self.integral += (
            error
            * delta_time
        )


        self.integral = clamp(
            self.integral,
            -self.integral_limit,
            self.integral_limit,
        )


        # ----------------------------------------------------
        # DERIVATIVE
        # ----------------------------------------------------

        if self.previous_error is None:

            derivative = 0.0

        else:

            derivative = (
                error
                - self.previous_error
            ) / delta_time


        self.previous_error = (
            error
        )


        # ----------------------------------------------------
        # OUTPUT
        # ----------------------------------------------------

        acceleration = (
            self.kp
            * error

            + self.ki
            * self.integral

            + self.kd
            * derivative
        )


        return acceleration