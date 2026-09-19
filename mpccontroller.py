import math
import numpy as np


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class MPCController:

    def __init__(
        self,
        wheelbase=12.8,
        max_steering=0.60,
        prediction_horizon=18,
        prediction_dt=0.08,
        steering_samples=13,
        pixels_per_meter=8.0,

        path_weight=10.0,
        heading_weight=8.0,
        steering_weight=0.03,
        steering_change_weight=0.10,
        progress_weight=0.08,

        terminal_path_weight=18.0,
        terminal_heading_weight=20.0,

        soft_cone_clearance_m=1.5,
        hard_cone_clearance_m=0.70,
        cone_weight=25.0,
    ):
        self.wheelbase = wheelbase
        self.max_steering = max_steering

        self.prediction_horizon = prediction_horizon
        self.prediction_dt = prediction_dt

        self.steering_samples = steering_samples

        self.pixels_per_meter = pixels_per_meter

        self.path_weight = path_weight
        self.heading_weight = heading_weight

        self.steering_weight = steering_weight
        self.steering_change_weight = steering_change_weight

        self.progress_weight = progress_weight

        self.terminal_path_weight = terminal_path_weight
        self.terminal_heading_weight = terminal_heading_weight

        self.soft_cone_clearance_m = soft_cone_clearance_m
        self.hard_cone_clearance_m = hard_cone_clearance_m
        self.cone_weight = cone_weight

        self.previous_steering = 0.0

    # ======================================================
    # PATH HELPERS
    # ======================================================

    def find_nearest_index(
        self,
        x,
        y,
        path,
    ):
        if not path:
            return 0

        best_index = 0
        best_distance = float("inf")

        for index, point in enumerate(path):

            dx = point[0] - x
            dy = point[1] - y

            distance_squared = (
                dx * dx
                + dy * dy
            )

            if distance_squared < best_distance:

                best_distance = distance_squared
                best_index = index

        return best_index

    # ======================================================

    def point_to_segment_distance(
        self,
        x,
        y,
        x1,
        y1,
        x2,
        y2,
    ):
        dx = x2 - x1
        dy = y2 - y1

        length_squared = (
            dx * dx
            + dy * dy
        )

        if length_squared < 1e-9:

            return math.hypot(
                x - x1,
                y - y1,
            )

        t = (
            (x - x1) * dx
            + (y - y1) * dy
        ) / length_squared

        t = clamp(
            t,
            0.0,
            1.0,
        )

        closest_x = (
            x1 + t * dx
        )

        closest_y = (
            y1 + t * dy
        )

        return math.hypot(
            x - closest_x,
            y - closest_y,
        )

    # ======================================================

    def distance_to_path(
        self,
        x,
        y,
        path,
    ):
        if path is None or len(path) < 2:
            return 0.0

        minimum_distance = float("inf")

        for index in range(
            len(path) - 1
        ):

            x1, y1 = path[index]
            x2, y2 = path[index + 1]

            distance = (
                self.point_to_segment_distance(
                    x,
                    y,
                    x1,
                    y1,
                    x2,
                    y2,
                )
            )

            minimum_distance = min(
                minimum_distance,
                distance,
            )

        return (
            minimum_distance
            / self.pixels_per_meter
        )

    # ======================================================

    def path_heading(
        self,
        path,
        index,
    ):
        if len(path) < 2:
            return 0.0

        index = max(
            0,
            min(
                index,
                len(path) - 2,
            ),
        )

        x1, y1 = path[index]
        x2, y2 = path[index + 1]

        dx = x2 - x1

        # pygame Y -> Cartesian Y
        dy = y1 - y2

        return math.atan2(
            dy,
            dx,
        )

    # ======================================================
    # LOCAL PATH CURVATURE
    # ======================================================

    def local_turn_amount(
        self,
        path,
        start_index,
        lookahead_points=5,
    ):
        """
        Estimate how much the path turns ahead.

        Used to detect:
        - hairpin entry
        - apex
        - hairpin exit
        """

        if len(path) < 3:
            return 0.0

        end_index = min(
            start_index + lookahead_points,
            len(path) - 2,
        )

        maximum_turn = 0.0

        for index in range(
            start_index,
            end_index + 1,
        ):

            heading_1 = self.path_heading(
                path,
                index,
            )

            next_index = min(
                index + 1,
                len(path) - 2,
            )

            heading_2 = self.path_heading(
                path,
                next_index,
            )

            turn = abs(
                normalize_angle(
                    heading_2
                    - heading_1
                )
            )

            maximum_turn = max(
                maximum_turn,
                turn,
            )

        return maximum_turn

    # ======================================================
    # CONE CLEARANCE
    # ======================================================

    def distance_to_nearest_cone(
        self,
        x,
        y,
        blue_cones,
        yellow_cones,
    ):
        nearest_distance = float("inf")

        all_cones = []

        if blue_cones:
            all_cones.extend(
                blue_cones
            )

        if yellow_cones:
            all_cones.extend(
                yellow_cones
            )

        for cone_x, cone_y in all_cones:

            distance_pixels = math.hypot(
                cone_x - x,
                cone_y - y,
            )

            distance_metres = (
                distance_pixels
                / self.pixels_per_meter
            )

            nearest_distance = min(
                nearest_distance,
                distance_metres,
            )

        return nearest_distance

    # ======================================================

    def calculate_cone_cost(
        self,
        x,
        y,
        blue_cones,
        yellow_cones,
    ):
        distance = (
            self.distance_to_nearest_cone(
                x,
                y,
                blue_cones,
                yellow_cones,
            )
        )

        if math.isinf(distance):
            return 0.0

        if (
            distance
            < self.hard_cone_clearance_m
        ):
            return 1_000_000.0

        if (
            distance
            >= self.soft_cone_clearance_m
        ):
            return 0.0

        clearance_error = (
            self.soft_cone_clearance_m
            - distance
        )

        return (
            self.cone_weight
            * clearance_error ** 2
        )

    # ======================================================
    # VEHICLE MODEL
    # ======================================================

    def predict_state(
        self,
        x,
        y,
        yaw,
        speed,
        steering,
    ):
        dt = self.prediction_dt

        screen_speed = (
            speed
            * self.pixels_per_meter
        )

        predicted_x = (
            x
            + screen_speed
            * math.cos(yaw)
            * dt
        )

        predicted_y = (
            y
            - screen_speed
            * math.sin(yaw)
            * dt
        )

        wheelbase_metres = (
            self.wheelbase
            / self.pixels_per_meter
        )

        yaw_rate = (
            speed
            / wheelbase_metres
            * math.tan(steering)
        )

        predicted_yaw = (
            yaw
            + yaw_rate * dt
        )

        predicted_yaw = normalize_angle(
            predicted_yaw
        )

        return (
            predicted_x,
            predicted_y,
            predicted_yaw,
        )

    # ======================================================
    # STATE COST
    # ======================================================

    def calculate_state_cost(
        self,
        x,
        y,
        yaw,
        steering,
        previous_steering,
        path,
    ):
        if path is None or len(path) < 2:
            return 0.0, 0

        nearest_index = (
            self.find_nearest_index(
                x,
                y,
                path,
            )
        )

        path_error = (
            self.distance_to_path(
                x,
                y,
                path,
            )
        )

        desired_heading = (
            self.path_heading(
                path,
                nearest_index,
            )
        )

        heading_error = (
            normalize_angle(
                desired_heading
                - yaw
            )
        )

        # ----------------------------------------------
        # WRONG BRANCH PROTECTION
        # ----------------------------------------------

        if (
            abs(heading_error)
            > math.radians(105.0)
        ):
            return (
                1_000_000.0,
                nearest_index,
            )

        steering_change = (
            steering
            - previous_steering
        )

        cost = 0.0

        cost += (
            self.path_weight
            * path_error ** 2
        )

        cost += (
            self.heading_weight
            * heading_error ** 2
        )

        cost += (
            self.steering_weight
            * steering ** 2
        )

        cost += (
            self.steering_change_weight
            * steering_change ** 2
        )

        cost -= (
            self.progress_weight
            * nearest_index
        )

        return (
            cost,
            nearest_index,
        )

    # ======================================================
    # TERMINAL COST
    # ======================================================

    def calculate_terminal_cost(
        self,
        x,
        y,
        yaw,
        path,
    ):
        """
        The final predicted state matters strongly.

        This is especially important at a hairpin exit:
        a candidate that reaches the apex but points back
        into the corner should lose against a candidate
        that is aligned with the exit.
        """

        if path is None or len(path) < 2:
            return 0.0

        nearest_index = (
            self.find_nearest_index(
                x,
                y,
                path,
            )
        )

        path_error = (
            self.distance_to_path(
                x,
                y,
                path,
            )
        )

        # Look slightly further ahead than the
        # nearest segment.
        heading_index = min(
            nearest_index + 2,
            len(path) - 2,
        )

        desired_heading = (
            self.path_heading(
                path,
                heading_index,
            )
        )

        heading_error = (
            normalize_angle(
                desired_heading
                - yaw
            )
        )

        cost = 0.0

        cost += (
            self.terminal_path_weight
            * path_error ** 2
        )

        cost += (
            self.terminal_heading_weight
            * heading_error ** 2
        )

        return cost

    # ======================================================
    # SIMULATE TWO-STAGE STEERING SEQUENCE
    # ======================================================

    def simulate_candidate(
        self,
        car,
        path,
        steering_first,
        steering_second,
        blue_cones=None,
        yellow_cones=None,
    ):
        x = car.x
        y = car.y
        yaw = car.yaw
        speed = car.speed

        total_cost = 0.0

        previous_steering = (
            self.previous_steering
        )

        # First 45% = corner entry/apex.
        # Remaining 55% = apex/exit.
        switch_step = max(
            2,
            int(
                self.prediction_horizon
                * 0.45
            ),
        )

        for step in range(
            self.prediction_horizon
        ):

            if step < switch_step:
                target = steering_first
            else:
                target = steering_second

            # Fast actuator response.
            steering_response = 0.75

            steering = (
                previous_steering
                + steering_response
                * (
                    target
                    - previous_steering
                )
            )

            steering = clamp(
                steering,
                -self.max_steering,
                self.max_steering,
            )

            (
                x,
                y,
                yaw,
            ) = self.predict_state(
                x=x,
                y=y,
                yaw=yaw,
                speed=speed,
                steering=steering,
            )

            (
                state_cost,
                _,
            ) = self.calculate_state_cost(
                x=x,
                y=y,
                yaw=yaw,
                steering=steering,
                previous_steering=previous_steering,
                path=path,
            )

            if (
                state_cost
                >= 1_000_000.0
            ):
                return state_cost

            cone_cost = (
                self.calculate_cone_cost(
                    x=x,
                    y=y,
                    blue_cones=blue_cones,
                    yellow_cones=yellow_cones,
                )
            )

            if (
                cone_cost
                >= 1_000_000.0
            ):
                return cone_cost

            state_cost += cone_cost

            # Later predictions matter more.
            horizon_weight = (
                1.0
                + 0.10 * step
            )

            total_cost += (
                state_cost
                * horizon_weight
            )

            previous_steering = (
                steering
            )

        # ----------------------------------------------
        # VERY IMPORTANT:
        # reward correct hairpin EXIT orientation.
        # ----------------------------------------------

        total_cost += (
            self.calculate_terminal_cost(
                x=x,
                y=y,
                yaw=yaw,
                path=path,
            )
        )

        return total_cost

    # ======================================================
    # STEERING CANDIDATES
    # ======================================================

    def build_steering_candidates(
        self,
    ):
        """
        Include zero explicitly.

        Zero steering is important at hairpin exit.
        """

        candidates = list(
            np.linspace(
                -self.max_steering,
                self.max_steering,
                self.steering_samples,
            )
        )

        candidates.extend(
            [
                -0.15,
                -0.08,
                0.0,
                0.08,
                0.15,
            ]
        )

        candidates = [
            clamp(
                float(value),
                -self.max_steering,
                self.max_steering,
            )
            for value in candidates
        ]

        # Remove duplicates.
        candidates = sorted(
            set(
                round(
                    value,
                    5,
                )
                for value in candidates
            )
        )

        return candidates

    # ======================================================
    # MPC CONTROL
    # ======================================================

    def calculate_steering(
        self,
        car,
        path,
        blue_cones=None,
        yellow_cones=None,
    ):
        if path is None or len(path) < 2:

            # Don't freeze a large steering angle
            # when perception briefly disappears.
            steering_command = (
                self.previous_steering
                * 0.80
            )

            self.previous_steering = (
                steering_command
            )

            return (
                steering_command,
                None,
            )

        nearest_index = (
            self.find_nearest_index(
                car.x,
                car.y,
                path,
            )
        )

        local_turn = (
            self.local_turn_amount(
                path,
                nearest_index,
                lookahead_points=5,
            )
        )

        candidates = (
            self.build_steering_candidates()
        )

        best_first = (
            self.previous_steering
        )

        best_second = (
            self.previous_steering
        )

        best_cost = float("inf")

        # ==================================================
        # TWO-STAGE SEARCH
        # ==================================================

        for steering_first in candidates:

            for steering_second in candidates:

                # ------------------------------------------
                # HAIRPIN EXIT BIAS
                #
                # If the upcoming path is becoming gentle,
                # strongly discourage MORE steering in the
                # second half of the horizon.
                # ------------------------------------------

                sequence_penalty = 0.0

                if (
                    local_turn
                    < math.radians(18.0)
                ):

                    sequence_penalty += (
                        12.0
                        * steering_second ** 2
                    )

                # Generally prefer opening steering
                # toward the exit rather than increasing
                # lock indefinitely.
                if (
                    abs(steering_second)
                    > abs(steering_first)
                    + 0.10
                ):
                    sequence_penalty += (
                        8.0
                        * (
                            abs(steering_second)
                            - abs(steering_first)
                        ) ** 2
                    )

                cost = (
                    self.simulate_candidate(
                        car=car,
                        path=path,
                        steering_first=steering_first,
                        steering_second=steering_second,
                        blue_cones=blue_cones,
                        yellow_cones=yellow_cones,
                    )
                )

                cost += (
                    sequence_penalty
                )

                if cost < best_cost:

                    best_cost = cost

                    best_first = (
                        steering_first
                    )

                    best_second = (
                        steering_second
                    )

        # ==================================================
        # APPLY FIRST MPC ACTION ONLY
        # ==================================================

        # Receding horizon:
        # we only execute the first action and
        # solve again next frame.
        command_response = 0.78

        steering_command = (
            self.previous_steering
            + command_response
            * (
                best_first
                - self.previous_steering
            )
        )

        # --------------------------------------------------
        # EXIT UNWIND ASSIST
        #
        # MPC's planned second steering is already telling
        # us whether the corner is opening.
        # --------------------------------------------------

        if (
            abs(best_second)
            < abs(best_first)
        ):

            unwind_strength = 0.18

            steering_command = (
                steering_command
                + unwind_strength
                * (
                    best_second
                    - steering_command
                )
            )

        steering_command = clamp(
            steering_command,
            -self.max_steering,
            self.max_steering,
        )

        self.previous_steering = (
            steering_command
        )

        # ==================================================
        # VISUAL TARGET
        # ==================================================

        target_index = min(
            nearest_index + 3,
            len(path) - 1,
        )

        target_point = (
            path[target_index]
        )

        return (
            steering_command,
            target_point,
        )

    # ======================================================
    # RESET
    # ======================================================

    def reset(self):
        self.previous_steering = 0.0