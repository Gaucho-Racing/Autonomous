import math
from typing import Iterable, List, Optional, Sequence, Tuple


Point2 = Tuple[float, float]


def grid_index(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> Optional[int]:
    if resolution <= 0.0:
        return None
    gx = math.floor((x - origin_x) / resolution)
    gy = math.floor((y - origin_y) / resolution)
    if gx < 0 or gy < 0 or gx >= width or gy >= height:
        return None
    return int(gy * width + gx)


def shifted_path(points: Sequence[Point2], lateral_offset: float, ramp_distance: float) -> List[Point2]:
    if not points:
        return []
    shifted: List[Point2] = []
    ramp = max(ramp_distance, 1e-3)
    for index, (x, y) in enumerate(points):
        if index + 1 < len(points):
            dx = points[index + 1][0] - x
            dy = points[index + 1][1] - y
        elif index > 0:
            dx = x - points[index - 1][0]
            dy = y - points[index - 1][1]
        else:
            dx, dy = 1.0, 0.0
        length = math.hypot(dx, dy)
        nx, ny = (-dy / length, dx / length) if length > 1e-6 else (0.0, 1.0)
        forward_distance = max(0.0, math.hypot(x, y))
        blend = min(1.0, forward_distance / ramp)
        shifted.append((x + nx * lateral_offset * blend, y + ny * lateral_offset * blend))
    return shifted


def path_collides(
    points: Sequence[Point2],
    data: Sequence[int],
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    occupied_threshold: int,
) -> bool:
    for x, y in points:
        index = grid_index(x, y, origin_x, origin_y, resolution, width, height)
        if index is None or data[index] < 0 or data[index] >= occupied_threshold:
            return True
    return False


def path_clearance(
    points: Sequence[Point2],
    occupied_points: Iterable[Point2],
    maximum: float = 5.0,
) -> float:
    occupied = list(occupied_points)
    if not points or not occupied:
        return maximum
    return min(
        math.hypot(path_x - obstacle_x, path_y - obstacle_y)
        for path_x, path_y in points
        for obstacle_x, obstacle_y in occupied
    )


def inside_cone_corridor(
    points: Sequence[Point2],
    left: Sequence[Point2],
    right: Sequence[Point2],
    margin: float,
    match_distance: float,
) -> bool:
    for x, y in points:
        left_matches = [cone_y for cone_x, cone_y in left if abs(cone_x - x) <= match_distance]
        right_matches = [cone_y for cone_x, cone_y in right if abs(cone_x - x) <= match_distance]
        if left_matches and y > min(left_matches) - margin:
            return False
        if right_matches and y < max(right_matches) + margin:
            return False
    return True

