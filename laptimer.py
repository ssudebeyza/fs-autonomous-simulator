import math
import time


class LapTimer:
    def __init__(
        self,
        start_x,
        start_y,
        trigger_radius=25.0,
        minimum_lap_time=5.0,
    ):
        self.start_x = start_x
        self.start_y = start_y

        self.trigger_radius = trigger_radius
        self.minimum_lap_time = minimum_lap_time

        self.lap_start_time = time.time()
        self.last_trigger_time = 0.0

        self.was_inside_trigger = False

        self.lap_times = []
        self.current_lap = 1
        self.best_lap = None

    def distance_to_start(
        self,
        car,
    ):
        return math.hypot(
            car.x - self.start_x,
            car.y - self.start_y,
        )

    def update(
        self,
        car,
    ):
        current_time = time.time()

        distance = self.distance_to_start(
            car
        )

        inside_trigger = (
            distance
            <= self.trigger_radius
        )

        lap_completed = False
        completed_lap_time = None

        if (
            inside_trigger
            and not self.was_inside_trigger
        ):
            lap_time = (
                current_time
                - self.lap_start_time
            )

            if (
                lap_time
                >= self.minimum_lap_time
            ):
                self.lap_times.append(
                    lap_time
                )

                completed_lap_time = lap_time
                lap_completed = True

                if (
                    self.best_lap is None
                    or lap_time < self.best_lap
                ):
                    self.best_lap = lap_time

                self.current_lap += 1

                self.lap_start_time = (
                    current_time
                )

                self.last_trigger_time = (
                    current_time
                )

        self.was_inside_trigger = (
            inside_trigger
        )

        return (
            lap_completed,
            completed_lap_time,
        )

    def get_current_lap_time(
        self,
    ):
        return (
            time.time()
            - self.lap_start_time
        )

    def get_average_lap_time(
        self,
    ):
        if not self.lap_times:
            return None

        return (
            sum(self.lap_times)
            / len(self.lap_times)
        )