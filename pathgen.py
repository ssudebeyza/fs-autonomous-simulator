import math

from vehicle import PIXELS_PER_METER


class LocalPathGenerator:
    def __init__(
        self,
        min_track_width_m=3.0,
        max_track_width_m=14.0,
        max_forward_distance_m=50.0,
        min_forward_distance_m=-6.0,
        max_lateral_distance_m=24.0,
        max_forward_pair_difference_m=10.0,
        max_midpoint_gap_m=18.0,
    ):
        self.min_track_width_px = (
            min_track_width_m
            * PIXELS_PER_METER
        )

        self.max_track_width_px = (
            max_track_width_m
            * PIXELS_PER_METER
        )

        self.max_forward_distance_px = (
            max_forward_distance_m
            * PIXELS_PER_METER
        )

        self.min_forward_distance_px = (
            min_forward_distance_m
            * PIXELS_PER_METER
        )

        self.max_lateral_distance_px = (
            max_lateral_distance_m
            * PIXELS_PER_METER
        )

        self.max_forward_pair_difference_px = (
            max_forward_pair_difference_m
            * PIXELS_PER_METER
        )

        self.max_midpoint_gap_px = (
            max_midpoint_gap_m
            * PIXELS_PER_METER
        )


    # ==================================================
    # ANGLE
    # ==================================================

    def _normalize_angle(
        self,
        angle,
    ):
        return (
            angle + math.pi
        ) % (
            2.0 * math.pi
        ) - math.pi


    # ==================================================
    # WORLD -> CAR COORDINATES
    # ==================================================

    def _to_car_coordinates(
        self,
        car,
        point,
    ):
        dx = (
            point[0]
            - car.x
        )

        dy = (
            car.y
            - point[1]
        )

        cos_yaw = math.cos(
            car.yaw
        )

        sin_yaw = math.sin(
            car.yaw
        )

        forward = (
            dx * cos_yaw
            + dy * sin_yaw
        )

        lateral = (
            -dx * sin_yaw
            + dy * cos_yaw
        )

        return (
            forward,
            lateral,
        )


    # ==================================================
    # FILTER CONES
    # ==================================================

    def _filter_cones(
        self,
        car,
        cones,
    ):
        valid = []

        for cone in cones:

            forward, lateral = (
                self._to_car_coordinates(
                    car,
                    cone,
                )
            )

            if (
                forward
                < self.min_forward_distance_px
            ):
                continue

            if (
                forward
                > self.max_forward_distance_px
            ):
                continue

            if (
                abs(lateral)
                > self.max_lateral_distance_px
            ):
                continue

            valid.append(
                {
                    "point": cone,
                    "forward": forward,
                    "lateral": lateral,
                }
            )

        valid.sort(
            key=lambda item: item["forward"]
        )

        return valid


    # ==================================================
    # CREATE BLUE/YELLOW PAIRS
    # ==================================================

    def _create_pairs(
        self,
        blue,
        yellow,
    ):
        candidates = []


        for (
            blue_index,
            blue_cone,
        ) in enumerate(blue):

            for (
                yellow_index,
                yellow_cone,
            ) in enumerate(yellow):


                # --------------------------------------
                # FORWARD DIFFERENCE
                # --------------------------------------

                forward_difference = abs(
                    blue_cone["forward"]
                    - yellow_cone["forward"]
                )

                if (
                    forward_difference
                    > self.max_forward_pair_difference_px
                ):
                    continue


                # --------------------------------------
                # TRACK WIDTH
                # --------------------------------------

                dx = (
                    yellow_cone["point"][0]
                    - blue_cone["point"][0]
                )

                dy = (
                    yellow_cone["point"][1]
                    - blue_cone["point"][1]
                )

                track_width = math.hypot(
                    dx,
                    dy,
                )

                if (
                    track_width
                    < self.min_track_width_px
                ):
                    continue

                if (
                    track_width
                    > self.max_track_width_px
                ):
                    continue


                # --------------------------------------
                # LATERAL BALANCE
                # --------------------------------------

                lateral_balance = abs(
                    abs(
                        blue_cone["lateral"]
                    )
                    -
                    abs(
                        yellow_cone["lateral"]
                    )
                )


                # --------------------------------------
                # SAME SIDE PENALTY
                # --------------------------------------

                same_side_penalty = 0.0

                if (
                    blue_cone["lateral"]
                    * yellow_cone["lateral"]
                    > 0.0
                ):
                    same_side_penalty = (
                        4.0
                        * PIXELS_PER_METER
                    )


                # --------------------------------------
                # PAIR SCORE
                # --------------------------------------

                score = (
                    forward_difference
                    * 3.0

                    + lateral_balance
                    * 1.5

                    + track_width
                    * 0.2

                    + same_side_penalty
                )


                candidates.append(
                    (
                        score,
                        blue_index,
                        yellow_index,
                    )
                )


        # Best candidate first
        candidates.sort(
            key=lambda item: item[0]
        )


        # ==================================================
        # ONE TO ONE PAIRING
        # ==================================================

        used_blue = set()
        used_yellow = set()

        pairs = []


        for (
            score,
            blue_index,
            yellow_index,
        ) in candidates:

            if (
                blue_index
                in used_blue
            ):
                continue

            if (
                yellow_index
                in used_yellow
            ):
                continue


            blue_cone = (
                blue[blue_index]
            )

            yellow_cone = (
                yellow[yellow_index]
            )


            midpoint = (
                (
                    blue_cone["point"][0]
                    + yellow_cone["point"][0]
                )
                / 2.0,

                (
                    blue_cone["point"][1]
                    + yellow_cone["point"][1]
                )
                / 2.0,
            )


            midpoint_forward = (
                blue_cone["forward"]
                + yellow_cone["forward"]
            ) / 2.0


            pairs.append(
                (
                    midpoint_forward,
                    midpoint,
                )
            )


            used_blue.add(
                blue_index
            )

            used_yellow.add(
                yellow_index
            )


        return pairs


    # ==================================================
    # BUILD CONTINUOUS PATH
    # ==================================================

    def _build_continuous_path(
        self,
        car,
        pairs,
    ):
        if not pairs:
            return []


        if len(pairs) == 1:
            return [
                pairs[0][1]
            ]


        # ==================================================
        # START POINT
        #
        # Do not blindly use the most rearward point.
        # Prefer a midpoint close to the vehicle.
        # ==================================================

        usable_pairs = [
            pair
            for pair in pairs
            if (
                pair[0]
                >= -1.5
                * PIXELS_PER_METER
            )
        ]


        if usable_pairs:

            start_pair = min(
                usable_pairs,
                key=lambda pair: abs(
                    pair[0]
                ),
            )

        else:

            start_pair = min(
                pairs,
                key=lambda pair: abs(
                    pair[0]
                ),
            )


        start_point = (
            start_pair[1]
        )


        points = [
            pair[1]
            for pair in pairs
        ]


        local_path = [
            start_point
        ]


        remaining = [
            point
            for point in points
            if point != start_point
        ]


        # First direction comes from vehicle heading
        previous_heading = (
            car.yaw
        )


        # ==================================================
        # BUILD CHAIN
        # ==================================================

        while remaining:

            current_point = (
                local_path[-1]
            )

            best_point = None
            best_score = float("inf")
            best_heading = None


            for point in remaining:

                dx = (
                    point[0]
                    - current_point[0]
                )

                # Screen Y increases downward.
                dy_world = (
                    current_point[1]
                    - point[1]
                )


                distance = math.hypot(
                    dx,
                    dy_world,
                )


                # --------------------------------------
                # TOO CLOSE
                # --------------------------------------

                if distance < 3.0:
                    continue


                # --------------------------------------
                # TOO FAR
                # --------------------------------------

                if (
                    distance
                    > self.max_midpoint_gap_px
                ):
                    continue


                # --------------------------------------
                # CANDIDATE HEADING
                # --------------------------------------

                candidate_heading = (
                    math.atan2(
                        dy_world,
                        dx,
                    )
                )


                angle_difference = (
                    self._normalize_angle(
                        candidate_heading
                        - previous_heading
                    )
                )


                # --------------------------------------
                # REJECT SUDDEN REVERSAL
                #
                # Hairpin can turn strongly overall,
                # but consecutive path segments should
                # still rotate gradually.
                # --------------------------------------

                if (
                    abs(angle_difference)
                    > math.radians(95.0)
                ):
                    continue


                # --------------------------------------
                # FORWARD PROJECTION
                #
                # Prevent jumping across to the nearby
                # opposite branch of the hairpin.
                # --------------------------------------

                projection = (
                    dx
                    * math.cos(
                        previous_heading
                    )

                    + dy_world
                    * math.sin(
                        previous_heading
                    )
                )


                # Small backward tolerance is allowed.
                if (
                    projection
                    < -1.5
                    * PIXELS_PER_METER
                ):
                    continue


                # --------------------------------------
                # HEADING PENALTY
                # --------------------------------------

                heading_penalty = (
                    abs(
                        angle_difference
                    )
                    * 80.0
                )


                # --------------------------------------
                # BACKWARD PENALTY
                # --------------------------------------

                backward_penalty = 0.0


                if projection < 0.0:

                    backward_penalty = (
                        abs(
                            projection
                        )
                        * 2.5
                    )


                # --------------------------------------
                # TOTAL SCORE
                # --------------------------------------

                score = (
                    distance
                    + heading_penalty
                    + backward_penalty
                )


                if score < best_score:

                    best_score = (
                        score
                    )

                    best_point = (
                        point
                    )

                    best_heading = (
                        candidate_heading
                    )


            # ------------------------------------------
            # NO VALID CONTINUATION
            # ------------------------------------------

            if best_point is None:
                break


            # ------------------------------------------
            # ADD TO PATH
            # ------------------------------------------

            local_path.append(
                best_point
            )


            previous_heading = (
                best_heading
            )


            remaining.remove(
                best_point
            )


        return local_path


    # ==================================================
    # SMOOTH PATH
    # ==================================================

    def _smooth_path(
        self,
        path,
    ):
        if len(path) < 3:
            return path


        smoothed = [
            path[0]
        ]


        for index in range(
            1,
            len(path) - 1,
        ):

            previous_point = (
                path[index - 1]
            )

            current_point = (
                path[index]
            )

            next_point = (
                path[index + 1]
            )


            x = (
                previous_point[0]
                + current_point[0] * 2.0
                + next_point[0]
            ) / 4.0


            y = (
                previous_point[1]
                + current_point[1] * 2.0
                + next_point[1]
            ) / 4.0


            smoothed.append(
                (
                    x,
                    y,
                )
            )


        smoothed.append(
            path[-1]
        )


        return smoothed


    # ==================================================
    # GENERATE LOCAL PATH
    # ==================================================

    def generate_local_path(
        self,
        car,
        detected_blue,
        detected_yellow,
    ):

        blue = (
            self._filter_cones(
                car,
                detected_blue,
            )
        )


        yellow = (
            self._filter_cones(
                car,
                detected_yellow,
            )
        )


        if (
            len(blue) == 0
            or len(yellow) == 0
        ):
            return []


        pairs = (
            self._create_pairs(
                blue,
                yellow,
            )
        )


        if not pairs:
            return []


        local_path = (
            self._build_continuous_path(
                car,
                pairs,
            )
        )


        local_path = (
            self._smooth_path(
                local_path
            )
        )


        return local_path