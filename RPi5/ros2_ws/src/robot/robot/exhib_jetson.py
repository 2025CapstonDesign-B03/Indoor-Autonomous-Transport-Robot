#!/usr/bin/env python3
import socket
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class PersonReceiver(Node):
    def __init__(self):
        super().__init__('person_receiver_node')

        # UDP receiver
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5000))
        self.sock.settimeout(0.01)

        # Publishers
        self.person_pub = self.create_publisher(PointStamped, 'person_detection', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.state_pub = self.create_publisher(String, 'tracking_state', 10)

        # Timers
        self.create_timer(0.1, self.poll_udp)
        self.create_timer(0.5, self.timer_marker_callback)
        self.create_timer(0.1, self.periodic_update)

        # State
        self.latest_distance = None
        self.latest_angle = None
        self.last_marker_position = None
        self.mode = "idle"
        self.last_seen_time = self.get_clock().now()
        self.last_raw_msg = None

        # Distance / angle interpretation
        self.declare_parameter('distance_mode', 'range')       # 'range' or 'depth'
        self.declare_parameter('angle_offset_deg', 0.0)
        self.distance_mode = self.get_parameter('distance_mode').get_parameter_value().string_value
        self.angle_offset_deg = self.get_parameter('angle_offset_deg').get_parameter_value().double_value

        # Lost-rotate parameters
        self.declare_parameter('lost_rotate_duration', 10.5)
        self.declare_parameter('lost_rotate_ang_vel', 0.6)
        self.lost_rotate_duration = self.get_parameter('lost_rotate_duration').get_parameter_value().double_value
        self.lost_rotate_ang_vel = self.get_parameter('lost_rotate_ang_vel').get_parameter_value().double_value
        self.lost_rotate_start = None

        # P control parameters
        self.declare_parameter('target_distance', 1.1)
        self.declare_parameter('kp_lin', 0.4)
        self.declare_parameter('kp_ang', 0.8)
        self.declare_parameter('max_lin_vel', 0.25)
        self.declare_parameter('max_ang_vel', 0.6)
        self.declare_parameter('dist_deadzone', 0.05)
        self.declare_parameter('ang_deadzone_deg', 3.0)

        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.kp_lin = self.get_parameter('kp_lin').get_parameter_value().double_value
        self.kp_ang = self.get_parameter('kp_ang').get_parameter_value().double_value
        self.max_lin_vel = self.get_parameter('max_lin_vel').get_parameter_value().double_value
        self.max_ang_vel = self.get_parameter('max_ang_vel').get_parameter_value().double_value
        self.dist_deadzone = self.get_parameter('dist_deadzone').get_parameter_value().double_value
        self.ang_deadzone_deg = self.get_parameter('ang_deadzone_deg').get_parameter_value().double_value

        # Mode control (from GUI or CLI)
        self.mode_sub = self.create_subscription(
            String,
            'mode_select',
            self.mode_callback,
            10
        )

        self.get_logger().info("PersonReceiver minimal P-control node started (UDP port 5000)")

    def mode_callback(self, msg: String):
        self.set_mode(msg.data)

    def set_mode(self, new_mode: str):
        self.mode = new_mode
        self.state_pub.publish(String(data=f"mode:{new_mode}"))

        if new_mode == "lost_rotate":
            self.lost_rotate_start = self.get_clock().now()

        if new_mode == "idle":
            twist = Twist()
            self.cmd_pub.publish(twist)

    def poll_udp(self):
        new_data = False
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().strip()
            if msg != self.last_raw_msg:
                distance, angle = map(float, msg.split(","))
                self.latest_distance = distance
                self.latest_angle = angle
                self.last_seen_time = self.get_clock().now()
                self.last_raw_msg = msg
                new_data = True
        except socket.timeout:
            pass

        if self.mode == "tracking" and new_data:
            if self.latest_distance is not None and self.latest_angle is not None:
                if -90.0 <= self.latest_angle <= 90.0:
                    self.handle_yolo()
                self.get_logger().info(
                    f"recv person: dist={self.latest_distance:.2f} m, angle={self.latest_angle:.2f} deg"
                )

        if self.mode == "lost_rotate" and new_data:
            self.set_mode("tracking")
            self.handle_yolo()

    def handle_yolo(self):
        try:
            follow_dis = self.target_distance

            theta_deg = -(self.latest_angle + self.angle_offset_deg)
            theta = math.radians(theta_deg)
            d = self.latest_distance

            if self.distance_mode == 'range':
                marker_x = d * math.cos(theta)
                marker_y = d * math.sin(theta)
            elif self.distance_mode == 'depth':
                marker_x = d
                marker_y = d * math.tan(theta)
            else:
                marker_x = d * math.cos(theta)
                marker_y = d * math.sin(theta)

            distance_error = d - self.target_distance
            angle_error = theta

            if abs(distance_error) < self.dist_deadzone:
                distance_error = 0.0
            if abs(math.degrees(angle_error)) < self.ang_deadzone_deg:
                angle_error = 0.0

            vx = self.kp_lin * distance_error
            wz = self.kp_ang * angle_error

            if vx < 0.0:
                vx = 0.0
            vx = max(-self.max_lin_vel, min(self.max_lin_vel, vx))
            wz = max(-self.max_ang_vel, min(self.max_ang_vel, wz))

            twist = Twist()
            twist.linear.x = vx
            twist.angular.z = wz
            self.cmd_pub.publish(twist)

            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = "base_footprint"
            pt.point.x = marker_x
            pt.point.y = marker_y
            pt.point.z = 0.0
            self.person_pub.publish(pt)

            self.publish_marker(marker_x, marker_y)
            self.last_marker_position = (marker_x, marker_y)
            self.state_pub.publish(String(data="tracking_p_control"))
        except Exception as e:
            self.get_logger().error(f"[YOLO PARSE ERROR] {e}")

    def periodic_update(self):
        if self.mode == "tracking":
            self.check_person_timeout()
        elif self.mode == "lost_rotate":
            self.update_lost_rotate()
        elif self.mode == "idle":
            twist = Twist()
            self.cmd_pub.publish(twist)

    def check_person_timeout(self):
        now = self.get_clock().now()
        time_diff_sec = (now - self.last_seen_time).nanoseconds * 1e-9

        if time_diff_sec > 1.0:
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.set_mode("lost_rotate")

    def update_lost_rotate(self):
        if self.lost_rotate_start is None:
            self.lost_rotate_start = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.lost_rotate_start).nanoseconds * 1e-9

        if elapsed < self.lost_rotate_duration:
            twist = Twist()
            twist.angular.z = self.lost_rotate_ang_vel
            self.cmd_pub.publish(twist)
            self.state_pub.publish(String(data="lost_rotate"))
        else:
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.set_mode("idle")
            self.get_logger().info("Lost rotate done. Switching to idle.")

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def timer_marker_callback(self):
        if self.last_marker_position:
            x, y = self.last_marker_position
            self.publish_marker(x, y)


def main(args=None):
    rclpy.init(args=args)
    node = PersonReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

