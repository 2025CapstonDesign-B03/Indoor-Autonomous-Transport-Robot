#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
import math
from copy import deepcopy


class ScanMerger(Node):
    def __init__(self):
        super().__init__('merge_lidar_ultrasonic_node')

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
        self.ultra_front_sub = self.create_subscription(
            Range, '/es0', self.front_callback, qos_profile_sensor_data)
        self.ultra_right_sub = self.create_subscription(
            Range, '/es2', self.right_callback, qos_profile_sensor_data)
        self.ultra_back_sub = self.create_subscription(
            Range, '/es3', self.back_callback, qos_profile_sensor_data)
        self.ultra_left_sub = self.create_subscription(
            Range, '/es5', self.left_callback, qos_profile_sensor_data)

        # Publisher
        self.pub = self.create_publisher(LaserScan, '/scan_merged', qos_profile_sensor_data)

        # 최근 데이터 저장
        self.latest_lidar = None
        self.front_msg = None
        self.back_msg = None
        self.left_msg = None
        self.right_msg = None

        # 초음파 오프셋 (로봇 외곽 ~ 센서 거리)
        self.offset_front = 0.325
        self.offset_back = 0.325
        self.offset_left = 0.325
        self.offset_right = 0.325

        # 초음파 확산 각도(도 단위)
        self.spread_deg = 3  # 필요시 5~6도까지 조정

        # 동적 파라미터 (초음파 병합 On/Off)
        self.declare_parameter('use_ultrasonic', True)
        self.use_ultrasonic = self.get_parameter('use_ultrasonic').value
        self.add_on_set_parameters_callback(self._on_param_change)

        # Timer
        self.timer = self.create_timer(0.1, self.merge_and_publish)

        self.get_logger().info(
            f"[Init] lidar_ultrasonic node started. spread_deg={self.spread_deg}, use_ultrasonic={self.use_ultrasonic}"
        )

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'use_ultrasonic':
                try:
                    self.use_ultrasonic = bool(p.value)
                    self.get_logger().info(f'use_ultrasonic set via param -> {self.use_ultrasonic}')
                    return SetParametersResult(successful=True)
                except Exception:
                    self.get_logger().warn('use_ultrasonic must be boolean')
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    # ---- Callbacks ----
    def lidar_callback(self, msg):
        self.latest_lidar = msg
        self.merge_and_publish()

    def front_callback(self, msg): self.front_msg = msg
    def back_callback(self, msg): self.back_msg = msg
    def left_callback(self, msg): self.left_msg = msg
    def right_callback(self, msg): self.right_msg = msg

    # ---- Helper ----
    def is_valid_range(self, msg):
        return (
            msg is not None and
            not math.isnan(msg.range) and
            0.05 < msg.range < msg.max_range
        )

    # ---- Main merge ----
    def merge_and_publish(self):
        if self.latest_lidar is None:
            return

        merged_scan = deepcopy(self.latest_lidar)
        MIN, MAX = merged_scan.range_min, merged_scan.range_max
        ranges = list(merged_scan.ranges)

        # 초음파 병합 함수
        def override_range_spread(center_deg, msg, offset):
            if not self.is_valid_range(msg):
                return

            # 실제 거리 보정
            corrected = msg.range + offset
            corrected = max(MIN, min(corrected, MAX - 1e-3))

            # 라디안 각도 → index 변환
            rad = math.radians(center_deg)
            idx = int(round((rad - merged_scan.angle_min) / merged_scan.angle_increment))
            idx = max(0, min(idx, len(ranges) - 1))

            # spread_deg 만큼 부채꼴 덮기
            half = int(round(math.radians(self.spread_deg) / merged_scan.angle_increment))
            for k in range(-half, half + 1):
                j = idx + k
                if 0 <= j < len(ranges):
                    # LiDAR보다 가까울 때만 덮기 (보수적 병합)
                    if math.isinf(ranges[j]) or corrected < ranges[j]:
                        ranges[j] = corrected

        # 초음파 병합
        if self.use_ultrasonic:
            override_range_spread(0, self.front_msg, self.offset_front)
            override_range_spread(180, self.back_msg, self.offset_back)
            override_range_spread(90, self.left_msg, self.offset_left)
            override_range_spread(-90, self.right_msg, self.offset_right)

        # LiDAR의 range_min/max 유지 (절대 변경하지 않음)
        merged_scan.ranges = ranges
        merged_scan.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(merged_scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

