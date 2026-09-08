import csv
import os
import time


class TelemetryLogger:
    def __init__(
        self,
        filename="telemetry.csv",
    ):
        self.filename = filename
        self.start_time = time.time()
        self.file = None
        self.writer = None

        # 10 Hz telemetry logging
        self.log_interval = 0.1
        self.last_log_time = 0.0

    def start(self):
        file_exists = os.path.exists(
            self.filename
        )

        self.file = open(
            self.filename,
            "a",
            newline="",
        )

        self.writer = csv.writer(
            self.file
        )

        if not file_exists:
            self.writer.writerow(
                [
                    "time",
                    "speed",
                    "target_speed",
                    "steering_angle",
                    "x",
                    "y",
                    "yaw",
                    "nearest_index",
                ]
            )

    def log(
        self,
        car,
        target_speed,
        nearest_index,
    ):
        if self.writer is None:
            return

        current_time = (
            time.time()
            - self.start_time
        )

        # Only record one sample every 0.1 seconds = 10 Hz
        if (
            current_time - self.last_log_time
            < self.log_interval
        ):
            return

        self.last_log_time = current_time

        self.writer.writerow(
            [
                current_time,
                car.speed,
                target_speed,
                car.steering_angle,
                car.x,
                car.y,
                car.yaw,
                nearest_index,
            ]
        )

    def close(self):
        if self.file is not None:
            self.file.close()