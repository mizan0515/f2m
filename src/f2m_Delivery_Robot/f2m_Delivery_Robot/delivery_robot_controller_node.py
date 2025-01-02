#!/usr/bin/env python3
# my_delivery_robot_pkg/delivery_robot_controller_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DeliveryRobotControllerNode(Node):
    def __init__(self):
        super().__init__("delivery_robot_controller_node")
        self.subscription = self.create_subscription(
            String, "/delivery_command", self.on_delivery_command, 10
        )
        self.get_logger().info("DeliveryRobotControllerNode started.")

    def on_delivery_command(self, msg):
        data = msg.data  # 예: "DELIVER|123|37.1234|127.5678"
        if data.startswith("DELIVER|"):
            parts = data.split("|")
            if len(parts) == 4:
                cart_id, lat, lng = parts[1], parts[2], parts[3]
                self.deliver_to(cart_id, lat, lng)
            else:
                self.get_logger().warn("Invalid DELIVER format.")
        else:
            self.get_logger().warn(f"Unknown delivery command: {data}")

    def deliver_to(self, cart_id, lat, lng):
        self.get_logger().info(
            f"[DeliveryRobot] Starting delivery for cart_id={cart_id} to lat={lat}, lng={lng}..."
        )
        # 실제 네비게이션 로직 (Nav2 액션 등) 호출
        self.get_logger().info("... Simulating navigation & final delivery ...")
        # 배송 완료 후 상태 보고가 필요하다면 /delivery_feedback 등으로 알림 가능

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryRobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
