import math


class CurvatureSpeedPlanner:
    def __init__(
        self,
        max_speed=15.0,
        min_speed=5.0,
        lateral_acceleration_limit=6.0,
        braking_acceleration=7.0,
        preview_points=12,
        acceleration_rate=3.0,
        deceleration_rate=8.0,
    ):
        self.max_speed = max_speed
        self.min_speed = min_speed

        self.lateral_acceleration_limit = (
            lateral_acceleration_limit
        )

        self.braking_acceleration = (
            braking_acceleration
        )

        self.preview_points = (
            preview_points
        )

        self.acceleration_rate = (
            acceleration_rate
        )

        self.deceleration_rate = (
            deceleration_rate
        )

        self.previous_target_speed = (
            min_speed
        )


    # ========================================================
    # RESET
    # ========================================================

    def reset(self):
        self.previous_target_speed = (
            self.min_speed
        )


    # ========================================================
    # DISTANCE
    # ========================================================

    def _distance(
        self,
        point_1,
        point_2,
    ):
        return math.hypot(
            point_2[0] - point_1[0],
            point_2[1] - point_1[1],
        )


    # ========================================================
    # CURVATURE FROM THREE POINTS
    # ========================================================

    def _calculate_curvature(
        self,
        point_1,
        point_2,
        point_3,
    ):
        a = self._distance(
            point_1,
            point_2,
        )

        b = self._distance(
            point_2,
            point_3,
        )

        c = self._distance(
            point_1,
            point_3,
        )

        denominator = (
            a * b * c
        )

        if denominator < 1e-6:
            return 0.0

        area_twice = abs(
            (
                point_2[0]
                - point_1[0]
            )
            * (
                point_3[1]
                - point_1[1]
            )
            -
            (
                point_2[1]
                - point_1[1]
            )
            * (
                point_3[0]
                - point_1[0]
            )
        )

        curvature_pixels = (
            2.0
            * area_twice
            / denominator
        )

        return curvature_pixels


    # ========================================================
    # SPEED FROM CURVATURE
    # ========================================================

    def _speed_from_curvature(
        self,
        curvature,
        pixels_per_meter,
    ):
        if curvature < 1e-6:
            return self.max_speed

        curvature_per_meter = (
            curvature
            * pixels_per_meter
        )

        speed = math.sqrt(
            self.lateral_acceleration_limit
            / curvature_per_meter
        )

        return max(
            self.min_speed,
            min(
                self.max_speed,
                speed,
            ),
        )


    # ========================================================
    # LOCAL PATH SPEED
    # ========================================================

    def calculate_local_target_speed(
        self,
        local_path,
        current_speed,
        delta_time,
        pixels_per_meter,
    ):
        if delta_time <= 0.0:
            return self.previous_target_speed

        if len(local_path) < 3:

            raw_target_speed = (
                self.min_speed
            )

        else:

            preview_count = min(
                len(local_path),
                self.preview_points,
            )

            preview_path = (
                local_path[
                    :preview_count
                ]
            )

            speed_limits = []

            distances = []

            cumulative_distance_pixels = (
                0.0
            )

            for index in range(
                1,
                len(preview_path) - 1,
            ):
                previous_point = (
                    preview_path[
                        index - 1
                    ]
                )

                current_point = (
                    preview_path[
                        index
                    ]
                )

                next_point = (
                    preview_path[
                        index + 1
                    ]
                )

                cumulative_distance_pixels += (
                    self._distance(
                        previous_point,
                        current_point,
                    )
                )

                curvature = (
                    self._calculate_curvature(
                        previous_point,
                        current_point,
                        next_point,
                    )
                )

                corner_speed = (
                    self._speed_from_curvature(
                        curvature,
                        pixels_per_meter,
                    )
                )

                distance_metres = (
                    cumulative_distance_pixels
                    / pixels_per_meter
                )

                # --------------------------------------------
                # BRAKING PREVIEW
                #
                # Maximum speed we may have NOW if we need
                # to reach corner_speed after distance d.
                #
                # v_now² = v_corner² + 2*a*d
                # --------------------------------------------

                allowed_speed_now = math.sqrt(
                    max(
                        0.0,
                        corner_speed
                        * corner_speed
                        +
                        2.0
                        * self.braking_acceleration
                        * distance_metres,
                    )
                )

                allowed_speed_now = min(
                    allowed_speed_now,
                    self.max_speed,
                )

                speed_limits.append(
                    allowed_speed_now
                )

                distances.append(
                    distance_metres
                )


            if speed_limits:

                raw_target_speed = min(
                    speed_limits
                )

            else:

                raw_target_speed = (
                    self.min_speed
                )


        # ====================================================
        # TARGET SPEED RATE LIMITING
        # ====================================================

        if (
            raw_target_speed
            > self.previous_target_speed
        ):

            maximum_change = (
                self.acceleration_rate
                * delta_time
            )

            target_speed = min(
                raw_target_speed,
                self.previous_target_speed
                + maximum_change,
            )

        else:

            maximum_change = (
                self.deceleration_rate
                * delta_time
            )

            target_speed = max(
                raw_target_speed,
                self.previous_target_speed
                - maximum_change,
            )


        target_speed = max(
            self.min_speed,
            min(
                self.max_speed,
                target_speed,
            ),
        )


        self.previous_target_speed = (
            target_speed
        )

        return target_speed