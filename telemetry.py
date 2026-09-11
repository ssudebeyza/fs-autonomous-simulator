import csv
import time


class TelemetryLogger:
    def __init__(
        self,
        filename="telemetry.csv",
        logging_frequency=10.0,
    ):
        self.filename = filename
        self.logging_frequency = logging_frequency

        self.logging_interval = (
            1.0 / logging_frequency
        )

        self.file = None
        self.writer = None

        self.start_time = None
        self.last_log_time = None


    # ========================================================
    # START
    # ========================================================

    def start(self):

        self.file = open(
            self.filename,
            "w",
            newline="",
        )

        self.writer = csv.writer(
            self.file
        )

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
                "reference_cte",
                "local_path_error",
                "path_source",
            ]
        )

        self.start_time = (
            time.perf_counter()
        )

        self.last_log_time = (
            self.start_time
        )


    # ========================================================
    # LOG
    # ========================================================

    def log(
        self,
        car,
        target_speed,
        nearest_index,
        reference_cte=0.0,
        local_path_error=0.0,
        path_source="NONE",
    ):
        if self.writer is None:
            return


        current_time = (
            time.perf_counter()
        )


        if (
            current_time
            - self.last_log_time
            < self.logging_interval
        ):
            return


        elapsed_time = (
            current_time
            - self.start_time
        )


        self.writer.writerow(
            [
                elapsed_time,
                car.speed,
                target_speed,
                car.steering_angle,
                car.x,
                car.y,
                car.yaw,
                nearest_index,
                reference_cte,
                local_path_error,
                path_source,
            ]
        )


        self.last_log_time = (
            current_time
        )


    # ========================================================
    # CLOSE
    # ========================================================

    def close(self):

        if self.file is not None:

            self.file.flush()
            self.file.close()


        self.file = None
        self.writer = None