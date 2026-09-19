import math
import pygame

from vehicle import PIXELS_PER_METER


class SimulatedCamera:
    """
    Synthetic front-facing camera for the Formula Student simulator.

    Important:
    This is not real computer vision yet.

    The simulator knows the true world position of each cone.
    The camera:
        1. transforms world cone coordinates into vehicle coordinates,
        2. checks camera FOV/range,
        3. projects visible cones into an image,
        4. returns camera-style detections.

    Controllers should eventually use the returned detections instead
    of accessing the global cone map directly.
    """

    def __init__(
        self,
        image_width=640,
        image_height=360,
        horizontal_fov_deg=120.0,
        max_range_m=35.0,
        camera_height_m=0.75,
    ):
        self.image_width = image_width
        self.image_height = image_height

        self.horizontal_fov = math.radians(
            horizontal_fov_deg
        )

        self.max_range_m = max_range_m
        self.camera_height_m = camera_height_m

        self.center_x = image_width / 2.0

        # Horizon position.
        self.horizon_y = image_height * 0.36

        # Pinhole-camera focal length in pixels.
        self.focal_length = (
            image_width
            / (
                2.0
                * math.tan(
                    self.horizontal_fov / 2.0
                )
            )
        )

    # ========================================================
    # ANGLE
    # ========================================================

    @staticmethod
    def normalize_angle(angle):
        return (
            angle + math.pi
        ) % (
            2.0 * math.pi
        ) - math.pi

    # ========================================================
    # WORLD -> VEHICLE COORDINATES
    # ========================================================

    def world_to_vehicle(
        self,
        car,
        world_x,
        world_y,
    ):
        """
        Returns cone position relative to the vehicle.

        forward_m:
            positive = in front of vehicle

        lateral_m:
            positive = vehicle left
            negative = vehicle right
        """

        dx = world_x - car.x

        # Convert pygame downward-Y to Cartesian upward-Y.
        dy = car.y - world_y

        cos_yaw = math.cos(car.yaw)
        sin_yaw = math.sin(car.yaw)

        forward_pixels = (
            dx * cos_yaw
            + dy * sin_yaw
        )

        lateral_pixels = (
            -dx * sin_yaw
            + dy * cos_yaw
        )

        forward_m = (
            forward_pixels
            / PIXELS_PER_METER
        )

        lateral_m = (
            lateral_pixels
            / PIXELS_PER_METER
        )

        return (
            forward_m,
            lateral_m,
        )

    # ========================================================
    # VEHICLE -> WORLD
    # ========================================================

    def vehicle_to_world(
        self,
        car,
        forward_m,
        lateral_m,
    ):
        """
        Convert a camera-relative position back into simulator
        world coordinates.

        This will be useful for generating the camera local path.
        """

        forward_pixels = (
            forward_m
            * PIXELS_PER_METER
        )

        lateral_pixels = (
            lateral_m
            * PIXELS_PER_METER
        )

        cos_yaw = math.cos(car.yaw)
        sin_yaw = math.sin(car.yaw)

        dx = (
            forward_pixels * cos_yaw
            - lateral_pixels * sin_yaw
        )

        dy_cartesian = (
            forward_pixels * sin_yaw
            + lateral_pixels * cos_yaw
        )

        world_x = (
            car.x + dx
        )

        # Convert Cartesian Y back to pygame Y.
        world_y = (
            car.y - dy_cartesian
        )

        return (
            world_x,
            world_y,
        )

    # ========================================================
    # CAMERA PROJECTION
    # ========================================================

    def project_cone(
        self,
        car,
        cone,
        colour_name,
    ):
        cone_x = cone[0]
        cone_y = cone[1]

        (
            forward_m,
            lateral_m,
        ) = self.world_to_vehicle(
            car,
            cone_x,
            cone_y,
        )

        # Cone is behind the camera.
        if forward_m <= 0.25:
            return None

        distance_m = math.hypot(
            forward_m,
            lateral_m,
        )

        # Outside camera range.
        if distance_m > self.max_range_m:
            return None

        bearing = math.atan2(
            lateral_m,
            forward_m,
        )

        # Outside horizontal camera FOV.
        if (
            abs(bearing)
            > self.horizontal_fov / 2.0
        ):
            return None

        # ----------------------------------------------------
        # PINHOLE PROJECTION
        # ----------------------------------------------------

        image_x = (
            self.center_x
            - self.focal_length
            * (
                lateral_m
                / forward_m
            )
        )

        vertical_offset = (
            self.focal_length
            * self.camera_height_m
            / max(
                forward_m,
                0.5,
            )
        )

        base_y = (
            self.horizon_y
            + vertical_offset
        )

        # Approximate Formula Student cone height.
        cone_height_m = 0.50

        cone_height = (
            self.focal_length
            * cone_height_m
            / max(
                forward_m,
                0.5,
            )
        )

        cone_height = max(
            4.0,
            min(
                cone_height,
                self.image_height * 0.65,
            ),
        )

        cone_width = max(
            3.0,
            cone_height * 0.45,
        )

        # Image clipping.
        if (
            image_x < -cone_width
            or image_x
            > self.image_width + cone_width
        ):
            return None

        if (
            base_y < 0
            or base_y > self.image_height + cone_height
        ):
            return None

        return {
            "color": colour_name,

            # Kept only so we can validate the simulated sensor.
            # Controllers should not need this later.
            "world_point": (
                cone_x,
                cone_y,
            ),

            # These are the important perception outputs.
            "forward_m": forward_m,
            "lateral_m": lateral_m,
            "distance_m": distance_m,
            "bearing_rad": bearing,

            # Camera image coordinates.
            "image_x": image_x,
            "base_y": base_y,

            "cone_height": cone_height,
            "cone_width": cone_width,
        }

    # ========================================================
    # DETECT ALL VISIBLE CONES
    # ========================================================

    def detect_cones(
        self,
        car,
        blue_cones,
        yellow_cones,
    ):
        """
        Returns all cones visible to the camera.

        Detection output is sorted by forward distance.
        """

        detections = []

        for cone in blue_cones:
            detection = self.project_cone(
                car,
                cone,
                "blue",
            )

            if detection is not None:
                detections.append(
                    detection
                )

        for cone in yellow_cones:
            detection = self.project_cone(
                car,
                cone,
                "yellow",
            )

            if detection is not None:
                detections.append(
                    detection
                )

        detections.sort(
            key=lambda detection:
            detection["forward_m"]
        )

        return detections

    # ========================================================
    # SPLIT DETECTIONS BY COLOUR
    # ========================================================

    @staticmethod
    def split_detections(
        detections,
    ):
        blue = []
        yellow = []

        for detection in detections:

            if (
                detection["color"]
                == "blue"
            ):
                blue.append(
                    detection
                )

            elif (
                detection["color"]
                == "yellow"
            ):
                yellow.append(
                    detection
                )

        return blue, yellow

    # ========================================================
    # CAMERA DETECTION -> RELATIVE CONES
    # ========================================================

    @staticmethod
    def relative_cones(
        detections,
    ):
        """
        Returns controller-friendly relative cone positions:

            (forward_m, lateral_m)

        separated into blue and yellow lists.
        """

        blue_relative = []
        yellow_relative = []

        for detection in detections:

            point = (
                detection["forward_m"],
                detection["lateral_m"],
            )

            if (
                detection["color"]
                == "blue"
            ):
                blue_relative.append(
                    point
                )

            elif (
                detection["color"]
                == "yellow"
            ):
                yellow_relative.append(
                    point
                )

        blue_relative.sort(
            key=lambda point: point[0]
        )

        yellow_relative.sort(
            key=lambda point: point[0]
        )

        return (
            blue_relative,
            yellow_relative,
        )

    # ========================================================
    # CAMERA DETECTION -> WORLD CONES
    # ========================================================

    def detections_to_world(
        self,
        car,
        detections,
    ):
        """
        Reconstruct world positions only from the camera-relative
        forward/lateral measurements.

        We deliberately do NOT use detection["world_point"] here.

        This gives us data in the same coordinate system expected by
        the existing LocalPathGenerator.
        """

        blue_world = []
        yellow_world = []

        for detection in detections:

            world_point = (
                self.vehicle_to_world(
                    car,
                    detection["forward_m"],
                    detection["lateral_m"],
                )
            )

            if (
                detection["color"]
                == "blue"
            ):
                blue_world.append(
                    world_point
                )

            elif (
                detection["color"]
                == "yellow"
            ):
                yellow_world.append(
                    world_point
                )

        return (
            blue_world,
            yellow_world,
        )

    # ========================================================
    # DRAW CAMERA CONE
    # ========================================================

    def draw_cone(
        self,
        surface,
        detection,
    ):
        image_x = int(
            detection["image_x"]
        )

        base_y = int(
            detection["base_y"]
        )

        cone_height = int(
            detection["cone_height"]
        )

        cone_width = int(
            detection["cone_width"]
        )

        if (
            detection["color"]
            == "blue"
        ):
            colour = (
                40,
                110,
                255,
            )

        else:
            colour = (
                255,
                220,
                40,
            )

        top = (
            image_x,
            base_y - cone_height,
        )

        bottom_left = (
            image_x
            - cone_width // 2,
            base_y,
        )

        bottom_right = (
            image_x
            + cone_width // 2,
            base_y,
        )

        pygame.draw.polygon(
            surface,
            colour,
            [
                top,
                bottom_left,
                bottom_right,
            ],
        )

        pygame.draw.polygon(
            surface,
            (
                20,
                20,
                20,
            ),
            [
                top,
                bottom_left,
                bottom_right,
            ],
            1,
        )

    # ========================================================
    # DRAW CAMERA IMAGE
    # ========================================================

    def draw_background(
        self,
        surface,
    ):
        # Sky
        surface.fill(
            (
                150,
                195,
                225,
            )
        )

        # Ground
        ground_rect = pygame.Rect(
            0,
            int(self.horizon_y),
            self.image_width,
            self.image_height
            - int(self.horizon_y),
        )

        pygame.draw.rect(
            surface,
            (
                70,
                75,
                70,
            ),
            ground_rect,
        )

        # Horizon
        pygame.draw.line(
            surface,
            (
                230,
                230,
                230,
            ),
            (
                0,
                int(self.horizon_y),
            ),
            (
                self.image_width,
                int(self.horizon_y),
            ),
            2,
        )

    # ========================================================
    # CAMERA HUD
    # ========================================================

    def draw_hud(
        self,
        surface,
        detections,
    ):
        font = pygame.font.Font(
            None,
            24,
        )

        blue_count = sum(
            1
            for detection in detections
            if detection["color"] == "blue"
        )

        yellow_count = sum(
            1
            for detection in detections
            if detection["color"] == "yellow"
        )

        text = (
            f"CAMERA  "
            f"B:{blue_count}  "
            f"Y:{yellow_count}"
        )

        rendered = font.render(
            text,
            True,
            (
                255,
                255,
                255,
            ),
        )

        surface.blit(
            rendered,
            (
                10,
                10,
            ),
        )

    # ========================================================
    # RENDER
    # ========================================================

    def render(
        self,
        car,
        blue_cones,
        yellow_cones,
    ):
        """
        Returns:

            camera_surface,
            detections
        """

        surface = pygame.Surface(
            (
                self.image_width,
                self.image_height,
            )
        )

        self.draw_background(
            surface
        )

        detections = (
            self.detect_cones(
                car,
                blue_cones,
                yellow_cones,
            )
        )

        # Draw far cones first and near cones last.
        draw_order = sorted(
            detections,
            key=lambda detection:
            detection["distance_m"],
            reverse=True,
        )

        for detection in draw_order:
            self.draw_cone(
                surface,
                detection,
            )

        self.draw_hud(
            surface,
            detections,
        )

        return (
            surface,
            detections,
        )