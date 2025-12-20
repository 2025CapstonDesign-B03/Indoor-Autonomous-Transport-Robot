#!/usr/bin/env python3
import socket
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose

from nav2_msgs.action import NavigateToPose


class PersonReceiver(Node):
    def __init__(self):
        super().__init__('person_receiver_node')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5000))
        self.sock.settimeout(0.01)

        self.pub = self.create_publisher(PointStamped, 'person_detection', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'tracking_state', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer = self.create_timer(0.1, self.poll_udp)
        self.create_timer(0.5, self.timer_marker_callback)
        self.create_timer(0.1, self.periodic_update)

        self.latest_distance = None
        self.latest_angle = None
        self.last_marker_position = None
        self.last_published_goal = None
        self.current_pose = None
        self.mode = "idle"
        self.last_seen_time = self.get_clock().now()
        self.last_raw_msg = None

        self.declare_parameter('goal_tolerance', 0.24)    # nav2
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        self.declare_parameter('distance_mode', 'range')
        self.declare_parameter('angle_offset_deg', 0.0)
        self.distance_mode = self.get_parameter('distance_mode').get_parameter_value().string_value
        self.angle_offset_deg = self.get_parameter('angle_offset_deg').get_parameter_value().double_value

        # rotate-after-lost parameters
        self.declare_parameter('lost_rotate_duration', 10.5)   # spin time
        self.declare_parameter('lost_rotate_ang_vel', -0.6)    # rad/s
        self.lost_rotate_duration = self.get_parameter('lost_rotate_duration').get_parameter_value().double_value
        self.lost_rotate_ang_vel = self.get_parameter('lost_rotate_ang_vel').get_parameter_value().double_value
        self.lost_rotate_start = None
        
        self.declare_parameter('lost_rotate_turns', 2.0)
        self.lost_rotate_turns = self.get_parameter('lost_rotate_turns').get_parameter_value().double_value
        self.rotate_target_angle = None
        self.rotate_accum = 0.0
        self.rotate_prev_yaw = None

        self.mode_sub = self.create_subscription(
            String,
            'mode_select',
            self.mode_callback,
            10
        )

        self.get_logger().info("Listening on UDP port 5000...")

    def mode_callback(self, msg: String):
        self.set_mode(msg.data)

    def set_mode(self, new_mode: str):
        self.mode = new_mode
        self.state_pub.publish(String(data=f"mode:{new_mode}"))
        if new_mode == "lost_rotate":
            self.lost_rotate_start = self.get_clock().now()
            self.rotate_target_angle = 2.0 * math.pi * max(0.0, self.lost_rotate_turns)
            self.rotate_accum = 0.0
            self.rotate_prev_yaw = self._get_yaw_from_tf()


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
                self.get_logger().info(f"recv person: dist={self.latest_distance:.2f} m, angle={self.latest_angle:.2f} deg")               
        if self.mode == "lost_rotate" and new_data:
            self.set_mode("tracking")
            self.handle_yolo()


    def handle_yolo(self):
        try:
            follow_dis = 1.1
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

            goal_x = marker_x - follow_dis * math.cos(theta)
            goal_y = marker_y - follow_dis * math.sin(theta)

            dx = marker_x - goal_x
            dy = marker_y - goal_y
            yaw = math.atan2(dy, dx)

            ps = PoseStamped()
            ps.header.stamp = rclpy.time.Time().to_msg()
            ps.header.frame_id = "base_footprint"
            ps.pose.position.x = goal_x
            ps.pose.position.y = goal_y
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw

            try:
                out = self.tf_buffer.transform(
                    ps,
                    "map",
                    timeout=Duration(seconds=0.05)
                )
            except TransformException as ex:
                self.get_logger().warn(f"[TF] map<-base_footprint not available: {ex}")
                return

            self.goal_pub.publish(out)
            self.send_nav_goal(out)
            self.last_published_goal = out

            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = "base_footprint"
            pt.point.x = marker_x
            pt.point.y = marker_y
            pt.point.z = 0.0
            self.pub.publish(pt)

            self.publish_marker(marker_x, marker_y)
            self.last_marker_position = (marker_x, marker_y)
            self.state_pub.publish(String(data="tracking"))
        except Exception as e:
            self.get_logger().error(f"[YOLO PARSE ERROR] {e}")

    def send_nav_goal(self, pose_stamped: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=0.01):
            self.get_logger().warn("Nav2 action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        self.nav_client.send_goal_async(goal_msg)

    def update_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time()
            )
            self.current_pose = PoseStamped()
            self.current_pose.header.frame_id = "map"
            self.current_pose.pose.position.x = transform.transform.translation.x
            self.current_pose.pose.position.y = transform.transform.translation.y
        except TransformException as e:
            self.get_logger().warn(f"[TF] lookup failed: {e}")

    def periodic_update(self):
        # update pose
        if self.mode != "idle":
            self.update_current_pose()

        # mode-specific
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
            if self.last_published_goal is not None and self.current_pose is not None:
                goal_x = self.last_published_goal.pose.position.x
                goal_y = self.last_published_goal.pose.position.y
                curr_x = self.current_pose.pose.position.x
                curr_y = self.current_pose.pose.position.y
                distance = math.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)

                if distance < self.goal_tolerance:
                    self.get_logger().info("Goal reached after target lost. Rotating.")
                    self.set_mode("lost_rotate")
                    return
                else:
                    goal = self.last_published_goal
                    goal.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pub.publish(goal)
                    self.send_nav_goal(goal)
                    self.state_pub.publish(String(data=f"lost_moving_to_goal:{distance:.2f}m"))
            else:
                self.set_mode("lost_rotate")
            

    def _get_yaw_from_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=0.1))
            q = tf.transform.rotation
            # yaw 추출 (roll/pitch 무시)
            siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
            return math.atan2(siny_cosp, cosy_cosp)
        except TransformException:
            return None

    
    def update_lost_rotate(self):
        # 각도 누적이 가능하면: 각도 기준 종료
        curr_yaw = self._get_yaw_from_tf()
        if curr_yaw is not None and self.rotate_target_angle is not None and self.rotate_prev_yaw is not None:
            # unwrap된 yaw 변화량 누적
            dyaw = curr_yaw - self.rotate_prev_yaw
            while dyaw > math.pi:  dyaw -= 2.0*math.pi
            while dyaw < -math.pi: dyaw += 2.0*math.pi
            self.rotate_accum += abs(dyaw)
            self.rotate_prev_yaw = curr_yaw

            if self.rotate_accum >= self.rotate_target_angle:
                twist = Twist()
                self.cmd_pub.publish(twist)
                self.set_mode("idle")
                self.last_published_goal = None
                self.get_logger().info("Lost rotate done by angle. Switching to idle.")
                return

        # 회전 명령 퍼블리시
        twist = Twist()
        twist.angular.z = self.lost_rotate_ang_vel
        self.cmd_pub.publish(twist)
        self.state_pub.publish(String(data="lost_rotate"))

        # TF가 계속 안 잡히는 환경을 대비한 시간 기반 백업 종료(여유 1.2배)
        if self.rotate_target_angle is None or self.rotate_prev_yaw is None:
            elapsed = (self.get_clock().now() - self.lost_rotate_start).nanoseconds * 1e-9
            backup_duration = (2.0*math.pi/abs(self.lost_rotate_ang_vel)) * self.lost_rotate_turns * 1.2
            if elapsed >= backup_duration:
                twist = Twist()
                self.cmd_pub.publish(twist)
                self.set_mode("idle")
                self.last_published_goal = None
                self.get_logger().info("Lost rotate done by time backup. Switching to idle.")


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
