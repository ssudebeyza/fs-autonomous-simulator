import math


class LapPerformanceTracker:
    def __init__(self):
        self.current_speeds = []
        self.current_cte_values = []

        self.completed_laps = []

    def reset_current_lap(self):
        self.current_speeds = []
        self.current_cte_values = []

    def add_sample(
        self,
        speed,
        cross_track_error,
    ):
        self.current_speeds.append(
            speed
        )

        self.current_cte_values.append(
            cross_track_error
        )

    def complete_lap(
        self,
        lap_number,
        lap_time,
    ):
        if not self.current_speeds:
            average_speed = 0.0
            maximum_speed = 0.0
        else:
            average_speed = (
                sum(self.current_speeds)
                / len(self.current_speeds)
            )

            maximum_speed = max(
                self.current_speeds
            )

        if not self.current_cte_values:
            mean_cte = 0.0
            cte_rmse = 0.0
        else:
            mean_cte = (
                sum(self.current_cte_values)
                / len(self.current_cte_values)
            )

            cte_rmse = math.sqrt(
                sum(
                    error ** 2
                    for error
                    in self.current_cte_values
                )
                / len(
                    self.current_cte_values
                )
            )

        result = {
            "lap": lap_number,
            "lap_time": lap_time,
            "average_speed": average_speed,
            "maximum_speed": maximum_speed,
            "mean_cte": mean_cte,
            "cte_rmse": cte_rmse,
        }

        self.completed_laps.append(
            result
        )

        self.reset_current_lap()

        return result