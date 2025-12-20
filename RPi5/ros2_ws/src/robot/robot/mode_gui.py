import tkinter as tk
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading
from action_msgs.msg import GoalStatus
from std_msgs.msg import String


class ModeGUI(Node):
    def __init__(self):
        super().__init__('mode_gui_node')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.mode_pub = self.create_publisher(String, 'mode_select', 10)
        self.state_sub = self.create_subscription(
            String, 'tracking_state', self.state_callback, 10
        )

        self.room_coords = {
            419: {"pos": (-2.82569, -16.0437, 0.0), "ori": (0.0, 0.0, -0.997061, 0.0766178)},
            420: {"pos": (-5.05534, -16.1696, 0.0), "ori": (0.0, 0.0, -0.994816, 0.101687)},
            421: {"pos": (-10.3274, -17.5808, 0.0), "ori": (0.0, 0.0, -0.996778, 0.0802135)},
            422: {"pos": (-12.5756, -17.9976, 0.0), "ori": (0.0, 0.0, -0.998794, 0.0490926)},
            423: {"pos": (-18.08, -18.9307, 0.0), "ori": (0.0, 0.0, -0.99505, 0.0993754)},
            424: {"pos": (-20.0529, -19.1811, 0.0), "ori": (0.0, 0.0, -0.993656, 0.112462)},
            425: {"pos": (-25.173, -20.2847, 0.0), "ori": (0.0, 0.0, -0.996535, 0.0831774)},
            426: {"pos": (-27.5333, -20.6125, 0.0), "ori": (0.0, 0.0, -0.998136, 0.0610217)},
            427: {"pos": (-31.0154, -21.3361, 0.0), "ori": (0.0, 0.0, -0.997458, 0.0712628)},
            416: {"pos": (-34.2612, -11.4227, 0.0), "ori": (0.0, 0.0, 0.999885, 0.0151466)},
            415: {"pos": (-29.4206, -11.3315, 0.0), "ori": (0.0, 0.0, -0.999876, 0.0157406)},
            414: {"pos": (-24.4249, -11.2556, 0.0), "ori": (0.0, 0.0, -0.999549, 0.0300344)},
            413: {"pos": (-17.0926, -11.2228, 0.0), "ori": (0.0, 0.0, -0.999011, 0.0444624)},
            412: {"pos": (-13.8481, -11.1919, 0.0), "ori": (0.0, 0.0, -0.999564, 0.0295309)},
            411: {"pos": (-11.8641, -11.2024, 0.0), "ori": (0.0, 0.0, 0.999542, 0.0302628)},
            410: {"pos": (-3.74351, 0.518235, 0.0), "ori": (0.0, 0.0, -0.723858, 0.689949)},
            409: {"pos": (-3.90975, 6.9642, 0.0), "ori": (0.0, 0.0, -0.724083, 0.689713)},
            408: {"pos": (-4.72482, 6.87139, 0.0), "ori": (0.0, 0.0, -0.684434, 0.729075)},
            407: {"pos": (-4.82415, 9.03728, 0.0), "ori": (0.0, 0.0, -0.711209, 0.702981)},
            405: {"pos": (-4.85119, 13.0387, 0.0), "ori": (0.0, 0.0, -0.690643, 0.723196)},
            404: {"pos": (-5.20756, 29.1017, 0.0), "ori": (0.0, 0.0, -0.687375, 0.726302)},
            403: {"pos": (-5.36186, 31.1477, 0.0), "ori": (0.0, 0.0, -0.696566, 0.717493)},
            401: {"pos": (-5.70338, 42.0079, 0.0), "ori": (0.0, 0.0, -0.676338, 0.736592)},
        }

        self.current_goal_seq = 0
        self.latest_goal_seq = 0

        self.root = tk.Tk()
        self.root.title("Select Mode")
        self.root.geometry("1000x700")
        self.root.configure(bg="black")

        self.menu_frame = tk.Frame(self.root, bg="black")
        self.menu_frame.pack(fill="both", expand=True)

        center_frame = tk.Frame(self.menu_frame, bg="black")
        center_frame.pack(expand=True)

        self.btn_auto = tk.Button(
            center_frame, text="AutoDriving Mode",
            font=("Ubuntu", 36),
            fg="gold", bg="black",
            relief="raised",
            command=self.show_auto_menu
        )
        self.btn_auto.pack(pady=30, ipadx=20, ipady=15)

        self.btn_tracking = tk.Button(
            center_frame, text="UserTracking Mode",
            font=("Ubuntu", 36),
            fg="gold", bg="black",
            relief="raised",
            command=self.show_tracking_menu
        )
        self.btn_tracking.pack(pady=30, ipadx=20, ipady=15)

        self.auto_frame = tk.Frame(self.root, bg="black")

        self.rooms = [i for i in range(401, 428) if i not in [402, 406, 417, 418]]

        cols = 5
        for idx, room in enumerate(self.rooms):
            r = idx // cols
            c = idx % cols
            btn = tk.Button(
                self.auto_frame,
                text=f"{room}",
                font=("Liberation Sans", 20),
                fg="gold", bg="black",
                relief="raised",
                width=8, height=2,
                command=lambda n=room: self.handle_room_button(n)
            )
            btn.grid(row=r+1, column=c, padx=10, pady=10)

        for i in range(cols):
            self.auto_frame.columnconfigure(i, weight=1)
        self.auto_frame.rowconfigure(0, weight=1)
        self.auto_frame.rowconfigure((len(self.rooms) // cols) + 2, weight=1)

        back_btn = tk.Button(
            self.auto_frame, text="Back",
            font=("Ubuntu", 18), fg="tomato", bg="black",
            command=self.show_main_menu
        )
        back_btn.grid(row=(len(self.rooms) // cols) + 2, column=0, columnspan=cols, pady=20)

        self.delivery_frame = tk.Frame(self.root, bg="black")

        self.delivery_label = tk.Label(
            self.delivery_frame,
            text="",
            font=("Ubuntu", 50),
            fg="gold", bg="black"
        )
        self.delivery_label.pack(expand=True)

        self.back_btn_delivery = tk.Button(
            self.delivery_frame,
            text="Back",
            font=("Liberation Sans", 20),
            fg="tomato", bg="black",
            command=self.show_auto_menu
        )
        self.back_btn_delivery.pack(pady=20)

        self.user_frame = tk.Frame(self.root, bg="black")

        self.user_selected_frame = tk.Frame(self.root, bg="black")
        center_selected_frame = tk.Frame(self.user_selected_frame, bg="black")
        center_selected_frame.pack(expand=True)

        self.user_label = tk.Label(
            center_selected_frame,
            text="Person Tracking",
            font=("Ubuntu", 50, "bold"),
            fg="gold", bg="black"
        )
        self.user_label.pack(pady=(40, 20))

        self.state_label = tk.Label(
            center_selected_frame,
            text="State: ---",
            font=("Ubuntu", 36),
            fg="tomato", bg="black"
        )
        self.state_label.pack(pady=(0, 40))
        
        # 🔹 Retry 버튼: 항상 보이지만 기본은 비활성화
        self.retry_btn = tk.Button(
            self.user_selected_frame,
            text="Retry Tracking",
            font=("Ubuntu", 16),
            fg="gold",
            bg="black",
            state="disabled",          # idle일 때만 활성화
            command=self.retry_tracking
        )
        self.retry_btn.pack(pady=(0, 5))  # Back 버튼 바로 위에

        self.back_btn_user_selected = tk.Button(
            self.user_selected_frame,
            text="Back",
            font=("Liberation Sans", 20),
            fg="tomato", bg="black",
            command=self.show_main_menu
        )
        self.back_btn_user_selected.pack(pady=20)

    def show_main_menu(self):
        self.mode_pub.publish(String(data="idle"))
        self.auto_frame.pack_forget()
        self.delivery_frame.pack_forget()
        self.user_frame.pack_forget()
        self.user_selected_frame.pack_forget()
        self.menu_frame.pack(fill="both", expand=True)

    def show_auto_menu(self):
        self.mode_pub.publish(String(data="auto"))
        self.menu_frame.pack_forget()
        self.delivery_frame.pack_forget()
        self.user_frame.pack_forget()
        self.user_selected_frame.pack_forget()
        self.auto_frame.pack(fill="both", expand=True)

    def show_tracking_menu(self):
        self.mode_pub.publish(String(data="tracking"))
        self.menu_frame.pack_forget()
        self.auto_frame.pack_forget()
        self.delivery_frame.pack_forget()
        self.user_frame.pack_forget()
        self.user_label.config(text="Person Tracking")
        self.state_label.config(text="State: ---")
        self.retry_btn.config(state="disabled")
        self.user_selected_frame.pack(fill="both", expand=True)

    def show_delivery_screen(self, room_number):
        self.auto_frame.pack_forget()
        self.delivery_label.config(text=f"Delivering to Room {room_number}")
        self.delivery_frame.pack(fill="both", expand=True)
        print(f"AutoDriving {room_number} started")

    def handle_room_button(self, room_number):
        self.current_goal_seq += 1
        goal_seq = self.current_goal_seq
        self.latest_goal_seq = goal_seq

        self.show_delivery_screen(room_number)

        threading.Thread(
            target=self.send_goal,
            args=(room_number, goal_seq),
            daemon=True
        ).start()

    def send_goal(self, room_number, goal_seq):
        if room_number not in self.room_coords:
            self.get_logger().warn(f"No coordinates set for Room {room_number}")
            return

        pos = self.room_coords[room_number]["pos"]
        ori = self.room_coords[room_number]["ori"]

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z = pos
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = ori

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.nav_to_pose_client.wait_for_server()
        # self.get_logger().info(f"Sending NavigateToPose goal for Room {room_number} ...")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)

        send_goal_future.add_done_callback(
            lambda f, n=room_number, s=goal_seq: self.goal_response_callback(f, n, s)
        )

    def goal_response_callback(self, future, room_number, goal_seq):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if goal_seq == self.latest_goal_seq:
                self.root.after(0, lambda: self.delivery_label.config(
                    text=f"Room {room_number} X - Goal Rejected"
                ))
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda f, n=room_number, s=goal_seq: self.get_result_callback(f, n, s)
        )

    def get_result_callback(self, future, room_number, goal_seq):
        result = future.result().result
        status = future.result().status

        if goal_seq != self.latest_goal_seq:
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.root.after(0, lambda: self.delivery_label.config(
                text=f"Arrived at Room {room_number}"
            ))
        else:
            self.root.after(0, lambda: self.delivery_label.config(
                text=f"Goal FAILED for Room {room_number}"
            ))

    def retry_tracking(self):
        # idle 상태에서 다시 tracking 모드로 전환
        self.mode_pub.publish(String(data="tracking"))
        # UI 피드백 (선택)
        self.state_label.config(text="State: retry -> tracking")

    
    def state_callback(self, msg: String):
        text = msg.data
        self.root.after(0, lambda: self.update_state_ui(text))

    def update_state_ui(self, raw_state: str):
        self.state_label.config(text=f"State: {raw_state}")

        # idle 상태일 때만 Retry 버튼 활성화
        if "idle" in raw_state:
            self.retry_btn.config(state="normal")
        else:
            self.retry_btn.config(state="disabled")
    
    def run(self):
        self.root.mainloop()


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    gui = ModeGUI()

    ros_thread = threading.Thread(target=ros_spin, args=(gui,), daemon=True)
    ros_thread.start()

    gui.run()

    gui.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


