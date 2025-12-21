# zynq_node.py

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from .packet_utils import parse_packet
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState, Range
import serial
import threading
import math
import time

class ZynqPacket(Node):
    def __init__(self):
        super().__init__('uart_zynq_node')
        self.serial_port = serial.Serial(port='/dev/ttyAMA4', baudrate=115200, timeout=0.1)
        self.uart_lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.pose_lock = threading.Lock()
        self.enc_lock  = threading.Lock()

        self.buffer = ""
        
        self.Vbx = 0.0
        self.Wbx = 0.0
        self.w = 0.316  # fixed(origin): 0.345
        
        # 변수 초기화: 오도메트리 계산용
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_enc_time = self.get_clock().now()
        self.last_vx = 0.0
        self.last_wz = 0.0

        
        # 엔코더 값 저장용 변수 추가
        self.current_enc_L = 0
        self.current_enc_R = 0
        self.prev_enc_L = 0
        self.prev_enc_R = 0

        self.joint_angle_L = 0.0
        self.joint_angle_R = 0.0

        self.handlers = {
            "ENC": self.handle_enc,
            "ESD": self.handle_esd,
        }
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.es_pubs = [self.create_publisher(Range, f'/es{i}', 10) for i in range(6)]
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.thread_read = threading.Thread(target=self.PacketParser, daemon=True)
        self.thread_read.start()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(1.0/15.0, self.send_packet)  # 15 Hz
        self.pub_timer = self.create_timer(1.0/30.0, self.publish_odom_tf)
        
        self.first_enc = True
        
    def cmd_vel_callback(self, msg):
        self.Vbx = msg.linear.x
        self.Wbx = msg.angular.z

    def compute_wheel_speeds(self):
        v_l = self.Vbx - self.Wbx * self.w / 2
        v_r = self.Vbx + self.Wbx * self.w / 2
        return v_l, v_r

    def to_7char_str(self, value: float) -> str:
        return f"{value:+07.3f}"[-7:]

    def generate_packet(self, left: float, right: float) -> str:
        left_str = self.to_7char_str(left)
        right_str = self.to_7char_str(right)
        return f"!{left_str}/{right_str}?"

    def send_packet(self):
        v_left, v_right = self.compute_wheel_speeds()
        packet = self.generate_packet(v_left, v_right)
        # self.get_logger().info(f"send: {packet}")
        
        with self.uart_lock:
            self.serial_port.write(packet.encode('ascii'))
        # if not hasattr(self, 'log_counter'):
            # self.log_counter = 0
        # self.log_counter += 1
        # if self.log_counter % 5 ==0:
            # self.get_logger().info(f"send: {packet}")
           
    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()

    def PacketParser(self):
        while rclpy.ok():
            try:
                bytes_to_read = self.serial_port.in_waiting
                if bytes_to_read > 0:
                    # 쌓인 데이터 전체를 한번에 읽음 (매우 효율적)
                    data = self.serial_port.read(bytes_to_read).decode('ascii', errors='ignore')
                    self.buffer += data

                # 버퍼에 완전한 패킷('!...?' 형식)이 있는지 확인하고 처리
                while '!' in self.buffer and '?' in self.buffer:
                    start = self.buffer.find('!')
                    end = self.buffer.find('?', start)
                    if end == -1:
                        # '!'는 있지만 '?'가 아직 도착하지 않음
                        break
                
                    packet = self.buffer[start:end+1]
                    self.buffer = self.buffer[end+1:]
                
                    # 패킷 처리 로직은 그대로 사용
                    self.handle_packet(packet)
            except Exception as e:
                self.get_logger().debug(f"[UART READ ERROR] {e}")

            time.sleep(0.001)

    def handle_packet(self, packet):
        try:
            result = parse_packet(packet)
            if not result:
                return
            cmd = result['command']
            if cmd in self.handlers:
                self.handlers[cmd](result['fields'])
        except Exception as e:
            self.get_logger().debug(f"[HANDLE PACKET ERROR] {e} | raw={packet}")
    
    # 오도메트리 계산을 ENC 데이터 수신 시점으로 옮김
    def handle_enc(self, fields):
        try:
            enc_L = int(fields[0], 10)
            enc_R = int(fields[1], 10)
            
            with self.enc_lock:
                self.current_enc_L = enc_L
                self.current_enc_R = enc_R
            # self.get_logger().info(f"Received ENC: enc_L={enc_L}, enc_R={enc_R}")
            
            # if not hasattr(self, 'log_counter'):
                # self.log_counter = 0
            # self.log_counter += 1
            # if self.log_counter % 5 ==0:
                # self.get_logger().info(f"Received ENC: enc_L={enc_L}, enc_R={enc_R}")
                
            self.get_logger().debug(f"Received ENC: enc_L={enc_L}, enc_R={enc_R}")
            
            # 패킷이 올 때마다 오도메트리 계산 및 발행
            self.process_enc()

        except ValueError:
            self.get_logger().warn("ENC 패킷 파싱 실패")
            return None
            
    def handle_esd(self, fields):
        try:
            es_vals = [float(f) for f in fields]
            self.process_esd(es_vals)
            # self.get_logger().info(f"ESD: {es_vals}")
        except ValueError:
            self.get_logger().warn("ESD 패킷 파싱 실패")
            return None

    def process_enc(self):
        now = self.get_clock().now()
        dt = (now - self.last_enc_time).nanoseconds / 1e9
        
        if self.first_enc:
            with self.enc_lock:
                self.prev_enc_L = self.current_enc_L
                self.prev_enc_R = self.current_enc_R
            self.last_enc_time = now
            self.first_enc = False
            return

        if dt <= 0:
            self.get_logger().warn(f"[RESET] Abnormal dt={dt:.3f}s → enc 기준 재설정")
            with self.enc_lock:
                self.prev_enc_L = self.current_enc_L
                self.prev_enc_R = self.current_enc_R
            self.last_enc_time = now
            return
            
        with self.enc_lock:
            curL = self.current_enc_L
            curR = self.current_enc_R
            prevL = self.prev_enc_L
            prevR = self.prev_enc_R

        def wrap_delta(now_cnt, prev_cnt, mod=2**32):
            d = (now_cnt - prev_cnt) % mod
            if d > mod//2:
                d -= mod
            return d
    
        delta_L = wrap_delta(curL, prevL)
        delta_R = wrap_delta(curR, prevR)
        
        TICKS_PER_REV_EQ = 9880.0
        WHEEL_CIRCUMFERENCE = 0.42
        
        tick_to_m = WHEEL_CIRCUMFERENCE / TICKS_PER_REV_EQ

        v_l = (delta_L * tick_to_m) / dt
        v_r = (delta_R * tick_to_m) / dt

        vx = (v_r + v_l) / 2.0
        wz = (v_r - v_l) / self.w

        with self.pose_lock:
            self.theta += wz * dt
            self.x += vx * math.cos(self.theta) * dt
            self.y += vx * math.sin(self.theta) * dt
            
            self.last_vx = vx
            self.last_wz = wz
            #self.get_logger().info(f"[distance] w: {self.theta:.3f}")
        
        
            # joint_state 메시지 발행
            TICK_TO_RAD = 2 * math.pi / TICKS_PER_REV_EQ
            self.joint_angle_L += delta_L * TICK_TO_RAD
            self.joint_angle_R += delta_R * TICK_TO_RAD
            
        with self.enc_lock:
            self.prev_enc_L = curL
            self.prev_enc_R = curR
        self.last_enc_time = now
            
    def publish_odom_tf(self):
        now = self.get_clock().now()
        with self.pose_lock:
            x = self.x; y = self.y; th = self.theta
            vx = self.last_vx; wz = self.last_wz
            jl = self.joint_angle_L; jr = self.joint_angle_R

        qx,qy,qz,qw = quaternion_from_euler(0,0,th)
        
        joint_msg = JointState()
        joint_msg.header.stamp = now.to_msg()
        joint_msg.name = ['drivewhl_l_joint', 'drivewhl_r_joint']
        joint_msg.position = [jl, jr]
        self.joint_pub.publish(joint_msg)

        # Odometry 및 TF 메시지 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation = Quaternion(x=qx,y=qy,z=qz,w=qw)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = wz
        self.odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_footprint"
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = Quaternion(x=qx,y=qy,z=qz,w=qw)
        self.tf_broadcaster.sendTransform(tf_msg)
        

    def process_esd(self, es_vals):
        FRAME_MAP = [
            "echo_front",        # es0
            "echo_right",        # es2
            "echo_back",         # es3
            "echo_left",         # es5
        ]
        PUB_INDEX = [
            0,  # front  -> /es0
            2,  # right  -> /es2
            3,  # back   -> /es3
            5,  # left   -> /es5
        ]

        MIN_R = 0.0
        MAX_R = 2.0
        FOV = math.radians(15.0)
        now = self.get_clock().now().to_msg()

        for i, v in enumerate(es_vals):
            frame_id = FRAME_MAP[i]
            pub_idx = PUB_INDEX[i]

            r = Range()
            r.header.stamp = now
            r.header.frame_id = frame_id
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = FOV
            r.min_range = MIN_R
            r.max_range = MAX_R
            if math.isfinite(v) and (MIN_R <= v <= MAX_R):
                r.range = float(v)
            else:
                r.range = float('nan')
            self.es_pubs[pub_idx].publish(r)

def main():
    rclpy.init()
    node = ZynqPacket()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
