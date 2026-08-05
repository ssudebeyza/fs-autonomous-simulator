import math


def clamp(
    value: float,
    minimum: float,
    maximum: float,
) -> float:
    """
    Restricts a value to a specified range.
    """

    return max(
        minimum,
        min(value, maximum),
    )


def distance_between_points(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> float:
    """
    Returns the Euclidean distance between two points.
    """

    return math.hypot(
        x2 - x1,
        y2 - y1,
    )


def normalize_angle(angle: float) -> float:
    """
    Normalises an angle to the range -pi to pi.
    """

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )