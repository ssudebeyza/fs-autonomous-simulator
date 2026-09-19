import math

from vehicle import PIXELS_PER_METER


class LocalPathGenerator:
    """
    Camera-based local path generator.

    Main idea:
    1. Filter camera-visible cones.
    2. Order blue and yellow boundaries independently.
    3. Build centre path from valid blue-yellow pairs.
    4. If pairing becomes weak in a hairpin, estimate the
       centre from ONE visible boundary.
    5. Use previous path for short-term temporal continuity.

    This prevents the control path from disappearing in
    tight hairpins or partial camera visibility.
    """

    def __init__(
        self,
        min_track_width_m=3.0,
        max_track_width_m=11.0,
        expected_track_width_m=6.0,
        max_forward_distance_m=35.0,
        min_forward_distance_m=-4.0,
        max_lateral_distance_m=20.0,
        max_forward_pair_difference_m=10.0,
        max_midpoint_gap_m=14.0,
    ):
        self.min_track_width_px = (
            min_track_width_m * PIXELS_PER_METER
        )

        self.max_track_width_px = (
            max_track_width_m * PIXELS_PER_METER
        )

        self.expected_track_width_px = (
            expected_track_width_m * PIXELS_PER_METER
        )

        self.max_forward_distance_px = (
            max_forward_distance_m * PIXELS_PER_METER
        )

        self.min_forward_distance_px = (
            min_forward_distance_m * PIXELS_PER_METER
        )

        self.max_lateral_distance_px = (
            max_lateral_distance_m * PIXELS_PER_METER
        )

        self.max_forward_pair_difference_px = (
            max_forward_pair_difference_m
            * PIXELS_PER_METER
        )

        self.max_midpoint_gap_px = (
            max_midpoint_gap_m
            * PIXELS_PER_METER
        )

        # Boundary tracing
        self.max_boundary_step_px = (
            11.0 * PIXELS_PER_METER
        )

        self.max_boundary_turn = math.radians(125.0)

        # Final centre-path validation
        self.max_path_turn = math.radians(125.0)

        # Previous valid camera path
        self.previous_path = []

        # Remember recently measured track width.
        self.track_width_estimate_px = (
            self.expected_track_width_px
        )

    # ======================================================
    # RESET
    # ======================================================

    def reset(self):
        self.previous_path = []

        self.track_width_estimate_px = (
            self.expected_track_width_px
        )

    # ======================================================
    # HELPERS
    # ======================================================

    @staticmethod
    def _normalize_angle(angle):
        return (
            angle + math.pi
        ) % (
            2.0 * math.pi
        ) - math.pi

    @staticmethod
    def _distance(a, b):
        return math.hypot(
            b[0] - a[0],
            b[1] - a[1],
        )

    # ======================================================
    # WORLD -> VEHICLE COORDINATES
    # ======================================================

    def _to_car_coordinates(
        self,
        car,
        point,
    ):
        dx = point[0] - car.x

        # pygame Y axis is downward.
        dy = car.y - point[1]

        cos_yaw = math.cos(car.yaw)
        sin_yaw = math.sin(car.yaw)

        forward = (
            dx * cos_yaw
            + dy * sin_yaw
        )

        lateral = (
            -dx * sin_yaw
            + dy * cos_yaw
        )

        return forward, lateral

    # ======================================================
    # CAMERA CONE FILTER
    # ======================================================

    def _filter_cones(
        self,
        car,
        cones,
    ):
        filtered = []

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

            filtered.append(
                {
                    "point": (
                        float(cone[0]),
                        float(cone[1]),
                    ),
                    "forward": forward,
                    "lateral": lateral,
                }
            )

        return filtered

    # ======================================================
    # BOUNDARY START
    # ======================================================

    def _choose_boundary_start(
        self,
        cones,
    ):
        if not cones:
            return None

        candidates = [
            cone
            for cone in cones
            if (
                cone["forward"]
                >= -1.0 * PIXELS_PER_METER
            )
        ]

        if not candidates:
            candidates = cones

        return min(
            candidates,
            key=lambda cone: (
                math.hypot(
                    cone["forward"],
                    cone["lateral"],
                )
            ),
        )

    # ======================================================
    # ORDER ONE COLOUR BOUNDARY
    # ======================================================

    def _order_boundary(
        self,
        car,
        cones,
    ):
        if not cones:
            return []

        start = self._choose_boundary_start(
            cones
        )

        if start is None:
            return []

        ordered = [start]

        remaining = [
            cone
            for cone in cones
            if cone is not start
        ]

        # First segment roughly follows vehicle heading.
        previous_heading = car.yaw

        while remaining:

            current = ordered[-1]["point"]

            best = None
            best_heading = None
            best_score = float("inf")

            for candidate in remaining:

                point = candidate["point"]

                dx = (
                    point[0]
                    - current[0]
                )

                dy = (
                    current[1]
                    - point[1]
                )

                distance = math.hypot(
                    dx,
                    dy,
                )

                if distance < 1.0:
                    continue

                if (
                    distance
                    > self.max_boundary_step_px
                ):
                    continue

                heading = math.atan2(
                    dy,
                    dx,
                )

                heading_change = abs(
                    self._normalize_angle(
                        heading
                        - previous_heading
                    )
                )

                if (
                    heading_change
                    > self.max_boundary_turn
                ):
                    continue

                projection = (
                    dx
                    * math.cos(previous_heading)
                    + dy
                    * math.sin(previous_heading)
                )

                # Hairpins need some backwards
                # projection to be allowed.
                if (
                    projection
                    < -3.0 * PIXELS_PER_METER
                ):
                    continue

                score = distance

                # Prefer direction continuity.
                score += (
                    heading_change * 70.0
                )

                if projection < 0.0:
                    score += (
                        abs(projection) * 1.5
                    )

                # Avoid giant jumps in camera-forward
                # coordinate when two hairpin branches
                # are physically close.
                forward_jump = abs(
                    candidate["forward"]
                    - ordered[-1]["forward"]
                )

                score += (
                    0.15 * forward_jump
                )

                if score < best_score:

                    best = candidate
                    best_heading = heading
                    best_score = score

            if best is None:
                break

            ordered.append(best)

            remaining.remove(best)

            previous_heading = (
                best_heading
            )

        return ordered

    # ======================================================
    # BLUE/YELLOW PAIR VALIDITY
    # ======================================================

    def _pair_valid(
        self,
        blue,
        yellow,
    ):
        width = self._distance(
            blue["point"],
            yellow["point"],
        )

        if (
            width
            < self.min_track_width_px
        ):
            return False

        if (
            width
            > self.max_track_width_px
        ):
            return False

        forward_difference = abs(
            blue["forward"]
            - yellow["forward"]
        )

        if (
            forward_difference
            > self.max_forward_pair_difference_px
        ):
            return False

        # IMPORTANT:
        #
        # We deliberately DO NOT require:
        #
        # blue lateral * yellow lateral < 0
        #
        # because during a hairpin both boundaries
        # may temporarily appear on the same side
        # of the vehicle/camera coordinate frame.

        return True

    # ======================================================
    # PAIR ORDERED BOUNDARIES
    # ======================================================

    def _pair_boundaries(
        self,
        blue,
        yellow,
    ):
        if not blue or not yellow:
            return [], []

        midpoints = []
        widths = []

        blue_index = 0
        yellow_index = 0

        while (
            blue_index < len(blue)
            and
            yellow_index < len(yellow)
        ):

            b = blue[blue_index]
            y = yellow[yellow_index]

            if self._pair_valid(
                b,
                y,
            ):
                midpoint = (
                    (
                        b["point"][0]
                        + y["point"][0]
                    ) / 2.0,

                    (
                        b["point"][1]
                        + y["point"][1]
                    ) / 2.0,
                )

                width = self._distance(
                    b["point"],
                    y["point"],
                )

                midpoints.append(midpoint)
                widths.append(width)

                blue_index += 1
                yellow_index += 1

                continue

            # Try skipping one cone instead of
            # immediately abandoning the path.

            next_blue_valid = False
            next_yellow_valid = False

            if (
                blue_index + 1
                < len(blue)
            ):
                next_blue_valid = (
                    self._pair_valid(
                        blue[
                            blue_index + 1
                        ],
                        y,
                    )
                )

            if (
                yellow_index + 1
                < len(yellow)
            ):
                next_yellow_valid = (
                    self._pair_valid(
                        b,
                        yellow[
                            yellow_index + 1
                        ],
                    )
                )

            if (
                next_blue_valid
                and not next_yellow_valid
            ):
                blue_index += 1

            elif (
                next_yellow_valid
                and not next_blue_valid
            ):
                yellow_index += 1

            else:
                # No obvious pair.
                # Advance whichever is currently
                # closer to the vehicle.
                if (
                    b["forward"]
                    < y["forward"]
                ):
                    blue_index += 1
                else:
                    yellow_index += 1

        return midpoints, widths

    # ======================================================
    # TRACK WIDTH MEMORY
    # ======================================================

    def _update_track_width(
        self,
        widths,
    ):
        if not widths:
            return

        valid_widths = [
            width
            for width in widths
            if (
                self.min_track_width_px
                <= width
                <= self.max_track_width_px
            )
        ]

        if not valid_widths:
            return

        measured = (
            sum(valid_widths)
            / len(valid_widths)
        )

        # Low-pass update so one bad pair cannot
        # suddenly change our width estimate.
        self.track_width_estimate_px = (
            0.85
            * self.track_width_estimate_px
            + 0.15
            * measured
        )

        self.track_width_estimate_px = max(
            self.min_track_width_px,
            min(
                self.track_width_estimate_px,
                self.max_track_width_px,
            ),
        )

    # ======================================================
    # SINGLE BOUNDARY -> ESTIMATED CENTRE
    # ======================================================

    def _centre_from_boundary(
        self,
        boundary,
        colour,
    ):
        """
        Estimate track centre from one ordered boundary.

        For every boundary point we estimate its tangent and
        shift it approximately half a track width toward the
        inside of the track.

        Blue and yellow use opposite normal directions.
        """

        if len(boundary) < 2:
            return []

        centre_path = []

        half_width = (
            0.5
            * self.track_width_estimate_px
        )

        for index in range(
            len(boundary)
        ):

            point = (
                boundary[index]["point"]
            )

            if index == 0:

                next_point = (
                    boundary[index + 1][
                        "point"
                    ]
                )

                dx = (
                    next_point[0]
                    - point[0]
                )

                dy_cart = (
                    point[1]
                    - next_point[1]
                )

            elif (
                index
                == len(boundary) - 1
            ):

                previous_point = (
                    boundary[index - 1][
                        "point"
                    ]
                )

                dx = (
                    point[0]
                    - previous_point[0]
                )

                dy_cart = (
                    previous_point[1]
                    - point[1]
                )

            else:

                previous_point = (
                    boundary[index - 1][
                        "point"
                    ]
                )

                next_point = (
                    boundary[index + 1][
                        "point"
                    ]
                )

                dx = (
                    next_point[0]
                    - previous_point[0]
                )

                dy_cart = (
                    previous_point[1]
                    - next_point[1]
                )

            length = math.hypot(
                dx,
                dy_cart,
            )

            if length < 1e-6:
                continue

            tangent_x = (
                dx / length
            )

            tangent_y = (
                dy_cart / length
            )

            # Cartesian left normal.
            normal_x = (
                -tangent_y
            )

            normal_y = (
                tangent_x
            )

            # Convention:
            # blue boundary -> centre to its right
            # yellow boundary -> centre to its left
            #
            # If your track.py uses the opposite colour
            # convention, swap these two signs.
            if colour == "blue":

                shift_x = (
                    normal_x
                    * half_width
                )

                shift_y_cart = (
                    normal_y
                    * half_width
                )

            else:

                shift_x = (
                    -normal_x
                    * half_width
                )

                shift_y_cart = (
                    -normal_y
                    * half_width
                )

            centre_x = (
                point[0]
                + shift_x
            )

            # Cartesian Y -> pygame Y
            centre_y = (
                point[1]
                - shift_y_cart
            )

            centre_path.append(
                (
                    centre_x,
                    centre_y,
                )
            )

        return centre_path

    # ======================================================
    # SCORE A PATH
    # ======================================================

    def _path_score(
        self,
        car,
        path,
    ):
        if len(path) < 2:
            return float("inf")

        score = 0.0

        previous_heading = car.yaw

        for index in range(
            1,
            len(path)
        ):

            p0 = path[index - 1]
            p1 = path[index]

            dx = p1[0] - p0[0]
            dy = p0[1] - p1[1]

            distance = math.hypot(
                dx,
                dy,
            )

            if distance < 1.0:
                continue

            heading = math.atan2(
                dy,
                dx,
            )

            turn = abs(
                self._normalize_angle(
                    heading
                    - previous_heading
                )
            )

            score += (
                turn * 30.0
            )

            if (
                distance
                > self.max_midpoint_gap_px
            ):
                score += 1000.0

            previous_heading = heading

        # Prefer longer usable paths.
        score -= (
            len(path) * 5.0
        )

        return score

    # ======================================================
    # CHOOSE FALLBACK BOUNDARY
    # ======================================================

    def _single_boundary_fallback(
        self,
        car,
        blue_boundary,
        yellow_boundary,
    ):
        candidates = []

        if len(blue_boundary) >= 2:

            blue_path = (
                self._centre_from_boundary(
                    blue_boundary,
                    "blue",
                )
            )

            if len(blue_path) >= 2:

                candidates.append(
                    (
                        self._path_score(
                            car,
                            blue_path,
                        ),
                        blue_path,
                    )
                )

        if len(yellow_boundary) >= 2:

            yellow_path = (
                self._centre_from_boundary(
                    yellow_boundary,
                    "yellow",
                )
            )

            if len(yellow_path) >= 2:

                candidates.append(
                    (
                        self._path_score(
                            car,
                            yellow_path,
                        ),
                        yellow_path,
                    )
                )

        if not candidates:
            return []

        candidates.sort(
            key=lambda item: item[0]
        )

        return candidates[0][1]

    # ======================================================
    # CLEAN CENTRE PATH
    # ======================================================

    def _clean_path(
        self,
        car,
        path,
    ):
        if not path:
            return []

        clean = []

        previous_heading = car.yaw

        for point in path:

            if not clean:

                forward, _ = (
                    self._to_car_coordinates(
                        car,
                        point,
                    )
                )

                if (
                    forward
                    < -2.0
                    * PIXELS_PER_METER
                ):
                    continue

                clean.append(point)

                continue

            previous = clean[-1]

            dx = (
                point[0]
                - previous[0]
            )

            dy = (
                previous[1]
                - point[1]
            )

            distance = math.hypot(
                dx,
                dy,
            )

            if distance < 1.0:
                continue

            if (
                distance
                > self.max_midpoint_gap_px
            ):
                break

            heading = math.atan2(
                dy,
                dx,
            )

            turn = abs(
                self._normalize_angle(
                    heading
                    - previous_heading
                )
            )

            if (
                turn
                > self.max_path_turn
            ):
                break

            clean.append(point)

            previous_heading = heading

        return clean

    # ======================================================
    # REMOVE OLD PATH BEHIND VEHICLE
    # ======================================================

    def _prune_previous_path(
        self,
        car,
    ):
        useful = []

        for point in self.previous_path:

            forward, lateral = (
                self._to_car_coordinates(
                    car,
                    point,
                )
            )

            if (
                forward
                < -1.5
                * PIXELS_PER_METER
            ):
                continue

            if (
                forward
                > 25.0
                * PIXELS_PER_METER
            ):
                continue

            if (
                abs(lateral)
                > 18.0
                * PIXELS_PER_METER
            ):
                continue

            useful.append(point)

        self.previous_path = useful

    # ======================================================
    # TEMPORAL CONTINUITY
    # ======================================================

    def _apply_temporal_continuity(
        self,
        car,
        path,
    ):
        if not path:
            return []

        self._prune_previous_path(
            car
        )

        if (
            len(self.previous_path)
            < 2
        ):
            return path

        accepted = []

        for index, point in enumerate(
            path
        ):

            nearest_old = min(
                self._distance(
                    point,
                    old_point,
                )
                for old_point
                in self.previous_path
            )

            tolerance = (
                5.0
                * PIXELS_PER_METER
                + index
                * 2.0
                * PIXELS_PER_METER
            )

            if (
                index < 3
                and
                nearest_old
                > tolerance
            ):
                continue

            accepted.append(point)

        # New lap / new camera geometry:
        # do NOT force stale lap-1 path.
        if len(accepted) < 2:

            self.previous_path = []

            return path

        return accepted

    # ======================================================
    # HAIRPIN-SAFE SMOOTHING
    # ======================================================

    def _smooth_path(
        self,
        path,
    ):
        if len(path) < 3:
            return path

        result = [path[0]]

        for index in range(
            1,
            len(path) - 1,
        ):

            p0 = path[index - 1]
            p1 = path[index]
            p2 = path[index + 1]

            heading_1 = math.atan2(
                p0[1] - p1[1],
                p1[0] - p0[0],
            )

            heading_2 = math.atan2(
                p1[1] - p2[1],
                p2[0] - p1[0],
            )

            turn = abs(
                self._normalize_angle(
                    heading_2
                    - heading_1
                )
            )

            # Preserve hairpin geometry.
            if (
                turn
                > math.radians(22.0)
            ):
                result.append(p1)
                continue

            # Very mild smoothing elsewhere.
            result.append(
                (
                    0.10 * p0[0]
                    + 0.80 * p1[0]
                    + 0.10 * p2[0],

                    0.10 * p0[1]
                    + 0.80 * p1[1]
                    + 0.10 * p2[1],
                )
            )

        result.append(
            path[-1]
        )

        return result

    # ======================================================
    # VEHICLE ANCHOR
    # ======================================================

    def _add_vehicle_anchor(
        self,
        car,
        path,
    ):
        if not path:
            return []

        # Short anchor so PP does not artificially
        # straighten the entrance of the hairpin.
        distance = (
            0.50
            * PIXELS_PER_METER
        )

        anchor = (
            car.x
            + math.cos(car.yaw)
            * distance,

            car.y
            - math.sin(car.yaw)
            * distance,
        )

        if (
            self._distance(
                anchor,
                path[0],
            )
            < 0.25
            * PIXELS_PER_METER
        ):
            return path

        return [
            anchor,
            *path,
        ]

    # ======================================================
    # GENERATE LOCAL PATH
    # ======================================================

    def generate_local_path(
        self,
        car,
        detected_blue,
        detected_yellow,
    ):
        # --------------------------------------------------
        # 1. Camera detections
        # --------------------------------------------------

        blue = self._filter_cones(
            car,
            detected_blue,
        )

        yellow = self._filter_cones(
            car,
            detected_yellow,
        )

        # --------------------------------------------------
        # 2. Order each colour independently
        # --------------------------------------------------

        blue_boundary = (
            self._order_boundary(
                car,
                blue,
            )
            if len(blue) >= 2
            else []
        )

        yellow_boundary = (
            self._order_boundary(
                car,
                yellow,
            )
            if len(yellow) >= 2
            else []
        )

        # --------------------------------------------------
        # 3. Try normal two-boundary midpoint path
        # --------------------------------------------------

        paired_path = []
        measured_widths = []

        if (
            len(blue_boundary) >= 2
            and
            len(yellow_boundary) >= 2
        ):

            (
                paired_path,
                measured_widths,
            ) = self._pair_boundaries(
                blue_boundary,
                yellow_boundary,
            )

        self._update_track_width(
            measured_widths
        )

        # --------------------------------------------------
        # 4. Decide whether normal pairing is strong enough
        # --------------------------------------------------

        if len(paired_path) >= 3:

            path = paired_path

        else:

            # Hairpin / partial visibility fallback.
            #
            # Use ONE continuous boundary and estimate
            # the centre instead of returning [].
            path = (
                self._single_boundary_fallback(
                    car,
                    blue_boundary,
                    yellow_boundary,
                )
            )

            # If single-boundary reconstruction also
            # failed but we still have some real pairs,
            # keep those pairs.
            if (
                len(path) < 2
                and
                len(paired_path) >= 2
            ):
                path = paired_path

        # --------------------------------------------------
        # 5. Last perception fallback:
        # keep remaining useful previous path
        # --------------------------------------------------

        if len(path) < 2:

            self._prune_previous_path(
                car
            )

            if (
                len(self.previous_path)
                >= 2
            ):
                return (
                    self.previous_path.copy()
                )

            return []

        # --------------------------------------------------
        # 6. Geometry validation
        # --------------------------------------------------

        path = self._clean_path(
            car,
            path,
        )

        if len(path) < 2:

            self._prune_previous_path(
                car
            )

            if (
                len(self.previous_path)
                >= 2
            ):
                return (
                    self.previous_path.copy()
                )

            return []

        # --------------------------------------------------
        # 7. Frame-to-frame continuity
        # --------------------------------------------------

        path = (
            self._apply_temporal_continuity(
                car,
                path,
            )
        )

        if len(path) < 2:
            return []

        # --------------------------------------------------
        # 8. Gentle smoothing
        # --------------------------------------------------

        path = self._smooth_path(
            path
        )

        # --------------------------------------------------
        # 9. Short vehicle anchor
        # --------------------------------------------------

        path = self._add_vehicle_anchor(
            car,
            path,
        )

        # --------------------------------------------------
        # 10. Save for next camera frame
        # --------------------------------------------------

        self.previous_path = (
            path.copy()
        )

        return path