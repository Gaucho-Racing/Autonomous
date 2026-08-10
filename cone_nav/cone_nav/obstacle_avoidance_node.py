#!/usr/bin/env python3

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.time import Time

from cone_nav.avoidance_algorithms import (
    inside_cone_corridor,
    grid_index,
    path_clearance,
    path_collides,
    shifted_path,
)


Point2 = Tuple[float, float]


class ObstacleAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_avoidance_node")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("avoidance_enabled", True)
        self.declare_parameter("avoidance_shadow_mode", False)
        self.declare_parameter("avoidance_publish_rate_hz", 20.0)
        self.declare_parameter("avoidance_grid_timeout_sec", 0.20)
        self.declare_parameter("avoidance_path_timeout_sec", 0.30)
        self.declare_parameter("avoidance_max_lateral_offset", 0.55)
        self.declare_parameter("avoidance_offset_step", 0.10)
        self.declare_parameter("avoidance_ramp_distance", 1.0)
        self.declare_parameter("avoidance_boundary_margin", 0.22)
        self.declare_parameter("avoidance_cone_match_distance", 0.75)
        self.declare_parameter("avoidance_occupied_threshold", 50)
        self.declare_parameter("avoidance_deviation_weight", 1.0)
        self.declare_parameter("avoidance_clearance_weight", 0.35)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.enabled = bool(self.get_parameter("avoidance_enabled").value)
        self.shadow_mode = bool(self.get_parameter("avoidance_shadow_mode").value)
        self.grid_timeout = float(self.get_parameter("avoidance_grid_timeout_sec").value)
        self.path_timeout = float(self.get_parameter("avoidance_path_timeout_sec").value)
        self.max_offset = float(self.get_parameter("avoidance_max_lateral_offset").value)
        self.offset_step = max(0.01, float(self.get_parameter("avoidance_offset_step").value))
        self.ramp_distance = float(self.get_parameter("avoidance_ramp_distance").value)
        self.boundary_margin = float(self.get_parameter("avoidance_boundary_margin").value)
        self.cone_match_distance = float(
            self.get_parameter("avoidance_cone_match_distance").value
        )
        self.occupied_threshold = int(self.get_parameter("avoidance_occupied_threshold").value)
        self.deviation_weight = float(self.get_parameter("avoidance_deviation_weight").value)
        self.clearance_weight = float(self.get_parameter("avoidance_clearance_weight").value)

        self.nominal_path: Optional[Path] = None
        self.grid: Optional[OccupancyGrid] = None
        self.left_cones: List[Point2] = []
        self.right_cones: List[Point2] = []

        self.create_subscription(Path, "/path/nominal", self._path_callback, 10)
        self.create_subscription(OccupancyGrid, "/obstacles/local_grid", self._grid_callback, 1)
        self.create_subscription(PoseArray, "/cones/left", self._left_callback, 10)
        self.create_subscription(PoseArray, "/cones/right", self._right_callback, 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)

        rate = max(1.0, float(self.get_parameter("avoidance_publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self._publish)
        state = "enabled" if self.enabled else "bypassed"
        suffix = " (shadow mode)" if self.shadow_mode else ""
        self.get_logger().info(f"obstacle avoidance {state}{suffix}")

    def _path_callback(self, msg: Path) -> None:
        self.nominal_path = msg

    def _grid_callback(self, msg: OccupancyGrid) -> None:
        self.grid = msg

    def _left_callback(self, msg: PoseArray) -> None:
        self.left_cones = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def _right_callback(self, msg: PoseArray) -> None:
        self.right_cones = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def _fresh(self, stamp, timeout: float) -> bool:
        stamp_time = Time.from_msg(stamp)
        return (self.get_clock().now() - stamp_time).nanoseconds * 1e-9 <= timeout

    def _publish(self) -> None:
        if self.nominal_path is None or not self._fresh(
            self.nominal_path.header.stamp, self.path_timeout
        ):
            self.path_pub.publish(self._empty_path())
            return
        if self.nominal_path.header.frame_id != self.base_frame:
            self.path_pub.publish(self._empty_path())
            return

        if not self.enabled:
            self.path_pub.publish(self._copy_path(self.nominal_path))
            return

        if self.grid is None or not self._fresh(self.grid.header.stamp, self.grid_timeout):
            self.path_pub.publish(self._empty_path())
            return
        if self.grid.header.frame_id != self.base_frame:
            self.path_pub.publish(self._empty_path())
            return

        nominal = [
            (pose.pose.position.x, pose.pose.position.y) for pose in self.nominal_path.poses
        ]
        nominal = [
            point
            for point in nominal
            if grid_index(
                point[0],
                point[1],
                self.grid.info.origin.position.x,
                self.grid.info.origin.position.y,
                self.grid.info.resolution,
                self.grid.info.width,
                self.grid.info.height,
            )
            is not None
        ]
        if len(nominal) < 2:
            self.path_pub.publish(self._empty_path())
            return

        occupied = self._occupied_points(self.grid)
        candidates = []
        for offset in self._offsets():
            points = shifted_path(nominal, offset, self.ramp_distance)
            if path_collides(
                points,
                self.grid.data,
                self.grid.info.origin.position.x,
                self.grid.info.origin.position.y,
                self.grid.info.resolution,
                self.grid.info.width,
                self.grid.info.height,
                self.occupied_threshold,
            ):
                continue
            if not inside_cone_corridor(
                points,
                self.left_cones,
                self.right_cones,
                self.boundary_margin,
                self.cone_match_distance,
            ):
                continue
            clearance = path_clearance(points, occupied)
            score = self.deviation_weight * abs(offset) - self.clearance_weight * clearance
            candidates.append((score, points))

        if not candidates:
            self.path_pub.publish(self._copy_path(self.nominal_path) if self.shadow_mode else self._empty_path())
            return

        candidates.sort(key=lambda candidate: candidate[0])
        selected = nominal if self.shadow_mode else candidates[0][1]
        self.path_pub.publish(self._build_path(selected))

    def _offsets(self) -> Sequence[float]:
        count = int(math.floor(self.max_offset / self.offset_step))
        offsets = [0.0]
        for index in range(1, count + 1):
            offsets.extend((index * self.offset_step, -index * self.offset_step))
        return offsets

    def _occupied_points(self, grid: OccupancyGrid) -> List[Point2]:
        points: List[Point2] = []
        width = int(grid.info.width)
        for index, value in enumerate(grid.data):
            if value < self.occupied_threshold:
                continue
            gx = index % width
            gy = index // width
            points.append(
                (
                    grid.info.origin.position.x + (gx + 0.5) * grid.info.resolution,
                    grid.info.origin.position.y + (gy + 0.5) * grid.info.resolution,
                )
            )
        return points

    def _build_path(self, points: Sequence[Point2]) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.base_frame
        for index, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if index + 1 < len(points):
                yaw = math.atan2(points[index + 1][1] - y, points[index + 1][0] - x)
            elif index > 0:
                yaw = math.atan2(y - points[index - 1][1], x - points[index - 1][0])
            else:
                yaw = 0.0
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            path.poses.append(pose)
        return path

    def _copy_path(self, source: Path) -> Path:
        return self._build_path(
            [(pose.pose.position.x, pose.pose.position.y) for pose in source.poses]
        )

    def _empty_path(self) -> Path:
        return self._build_path([])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
