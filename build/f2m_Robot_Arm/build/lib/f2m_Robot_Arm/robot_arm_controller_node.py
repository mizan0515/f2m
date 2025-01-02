#!/usr/bin/env python3
# my_robot_arm_pkg/robot_arm_controller_node.py

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotArmControllerNode(Node):
    def __init__(self):
        super().__init__("robot_arm_controller_node")
        self.subscription = self.create_subscription(
            String, "/arm_command", self.on_arm_command, 10
        )
        self.feedback_pub = self.create_publisher(String, "/arm_feedback", 10)
        self.get_logger().info("RobotArmControllerNode started.")

    def on_arm_command(self, msg):
        data = msg.data
        if data.startswith("LOAD|"):
            json_str = data.replace("LOAD|", "")
            try:
                payload = json.loads(json_str)
                cart_id = payload["cart_id"]
                items = payload["items"]  # [{"fruit_name": "...", "quantity": 2}, ...]

                self.get_logger().info(f"[RobotArm] Start loading for cart_id={cart_id}")
                for item in items:
                    fname = item["fruit_name"]
                    qty = item["quantity"]
                    self.get_logger().info(f"  -> Loading {qty} x {fname}")

                # 실제 로봇팔 구동 대신 3초 뒤에 로딩 완료라고 피드백(타이머 사용)
                self.create_timer(3.0, lambda: self.loading_done(cart_id))

            except Exception as e:
                self.get_logger().error(f"Failed to parse arm command JSON: {e}")
        else:
            self.get_logger().warn(f"[RobotArm] Unknown command: {data}")

    def loading_done(self, cart_id):
        # 로딩이 끝나면 /arm_feedback으로 "LOADING_DONE|cart_id" 전송
        msg = String()
        msg.data = f"LOADING_DONE|{cart_id}"
        self.feedback_pub.publish(msg)
        self.get_logger().info(f"[RobotArm] Loading completed -> published {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
