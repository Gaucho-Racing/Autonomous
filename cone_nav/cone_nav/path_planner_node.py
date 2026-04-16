#!/usr/bin/env python3

import math
from typing import List, Optional, Sequence, Tuple

import message_filters
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


Point2 = Tuple[float, float]


def _pose_to_xy(pose: Pose) -> Point2:
    return pose.position.x, pose.position.y


def _distance(point: Point2) -> float:
    return math.hypot(point[0], point[1])


def _rolling_average(points: Sequence[Point2], window: int) -> List[Point2]:
    if window <= 1 or len(points) <= 1:
        return list(points)

    half = window // 2
    smoothed: List[Point2] = []
    for idx in range(len(points)):
        start = max(0, idx - half)
        end = min(len(points), idx + half + 1)
        span = points[start:end]
        smoothed.append(
            (
                sum(point[0] for point in span) / len(span),
                sum(point[1] for point in span) / len(span),
            )
        )
    return smoothed


class PathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_planner_node")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("path_smoothing_window", 5)
        self.declare_parameter("min_cones_to_plan", 2)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("track_half_width", 0.75)

        self.base_frame = self.get_parameter("base_frame").value
        self.path_smoothing_window = int(self.get_parameter("path_smoothing_window").value)
        self.min_cones_to_plan = int(self.get_parameter("min_cones_to_plan").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.track_half_width = float(self.get_parameter("track_half_width").value)

        self.left_sub = message_filters.Subscriber(self, PoseArray, "/cones/left")
        self.right_sub = message_filters.Subscriber(self, PoseArray, "/cones/right")
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=10,
            slop=0.05,
            allow_headerless=False,
        )
        self.sync.registerCallback(self._cones_callback)

        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/path/markers", 10)

        self.latest_left: List[Point2] = []
        self.latest_right: List[Point2] = []
        self.latest_stamp = None
        self.last_path: Optional[Path] = None
        self.last_markers: Optional[MarkerArray] = None

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._publish_timer)

        self.get_logger().info(
            f"path_planner_node publishing /path at {self.publish_rate_hz:.1f} Hz"
        )

    def _cones_callback(self, left_msg: PoseArray, right_msg: PoseArray) -> None:
        self.latest_left = sorted((_pose_to_xy(pose) for pose in left_msg.poses), key=_distance)
        self.latest_right = sorted((_pose_to_xy(pose) for pose in right_msg.poses), key=_distance)
        self.latest_stamp = self.get_clock().now().to_msg()

    def _publish_timer(self) -> None:
        waypoints = self._compute_waypoints(self.latest_left, self.latest_right)
        if not waypoints:
            if self.last_path is not None:
                self.last_path.header.stamp = self.get_clock().now().to_msg()
                for pose in self.last_path.poses:
                    pose.header.stamp = self.last_path.header.stamp
                self.path_pub.publish(self.last_path)
                if self.last_markers is not None:
                    self._refresh_marker_stamps(self.last_markers)
                    self.marker_pub.publish(self.last_markers)
            return

        path = self._build_path(waypoints)
        markers = self._build_markers(waypoints)
        self.last_path = path
        self.last_markers = markers
        self.path_pub.publish(path)
        self.marker_pub.publish(markers)

    def _compute_waypoints(self, left: Sequence[Point2], right: Sequence[Point2]) -> List[Point2]:
        total_cones = len(left) + len(right)
        if total_cones < self.min_cones_to_plan:
            return []

        midpoints: List[Point2] = []
        if left and right:
            unused_right = list(right)
            for left_point in left:
                nearest_idx = min(
                    range(len(unused_right)),
                    key=lambda idx: math.hypot(
                        left_point[0] - unused_right[idx][0],
                        left_point[1] - unused_right[idx][1],
                    ),
                )
                right_point = unused_right.pop(nearest_idx)
                midpoints.append(
                    (
                        0.5 * (left_point[0] + right_point[0]),
                        0.5 * (left_point[1] + right_point[1]),
                    )
                )
                if not unused_right:
                    break
        elif left:
            midpoints = [(point[0], point[1] - self.track_half_width) for point in left]
        elif right:
            midpoints = [(point[0], point[1] + self.track_half_width) for point in right]

        midpoints = sorted(midpoints, key=_distance)
        smoothed = _rolling_average(midpoints, self.path_smoothing_window)
        return [(0.0, 0.0)] + smoothed

    def _build_path(self, waypoints: Sequence[Point2]) -> Path:
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.base_frame

        for idx, point in enumerate(waypoints):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            yaw = self._heading_at(waypoints, idx)
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            path.poses.append(pose)

        return path

    def _build_markers(self, waypoints: Sequence[Point2]) -> MarkerArray:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = self.base_frame
        line.ns = "centerline"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.06
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0
        line.pose.orientation.w = 1.0

        for x, y in waypoints:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.03
            line.points.append(point)

        markers.markers.append(line)
        return markers

    def _refresh_marker_stamps(self, markers: MarkerArray) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in markers.markers:
            if marker.action != Marker.DELETEALL:
                marker.header.stamp = stamp

    @staticmethod
    def _heading_at(waypoints: Sequence[Point2], idx: int) -> float:
        if len(waypoints) < 2:
            return 0.0
        if idx < len(waypoints) - 1:
            dx = waypoints[idx + 1][0] - waypoints[idx][0]
            dy = waypoints[idx + 1][1] - waypoints[idx][1]
        else:
            dx = waypoints[idx][0] - waypoints[idx - 1][0]
            dy = waypoints[idx][1] - waypoints[idx - 1][1]
        return math.atan2(dy, dx)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
