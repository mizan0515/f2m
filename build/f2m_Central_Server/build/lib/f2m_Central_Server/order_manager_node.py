#!/usr/bin/env python3
# my_server_pkg/order_manager_node.py

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque

from .db_utils import get_db_connection

class OrderManagerNode(Node):
    def __init__(self):
        super().__init__("order_manager_node")

        # (1) 구독 & 퍼블리셔
        self.subscription_new_order = self.create_subscription(
            String, "/new_order", self.on_new_order_received, 10
        )
        self.subscription_arm_feedback = self.create_subscription(
            String, "/arm_feedback", self.on_arm_feedback_received, 10
        )
        self.arm_pub = self.create_publisher(String, "/arm_command", 10)
        self.delivery_pub = self.create_publisher(String, "/delivery_command", 10)

        # (2) 주문 큐
        self.order_queue = deque()
        self.timer = self.create_timer(2.0, self.process_queue)  # 2초 주기로 큐 처리

        # (3) 로딩 완료를 기다리는 cart_id 임시 저장(예: {cart_id: items_info})
        self.loading_in_progress = {}

        self.get_logger().info("OrderManagerNode started.")

    def on_new_order_received(self, msg):
        """
        /new_order 로부터 cart_id 수신
        """
        cart_id_str = msg.data
        self.get_logger().info(f"Received new cart_id: {cart_id_str}")
        self.order_queue.append(cart_id_str)

    def process_queue(self):
        """
        큐에 cart_id가 있으면 하나 꺼내서:
        1) DB에서 cart_fruit, fruit 테이블 조회 -> 아이템 목록
        2) 로봇팔에게 적재 명령: LOAD|cart_id|{JSON형식의 아이템들}
        """
        if not self.order_queue:
            return

        cart_id_str = self.order_queue.popleft()
        self.get_logger().info(f"Processing cart_id={cart_id_str} from queue")

        try:
            items_data = self.get_items_from_db(cart_id_str)
            # items_data 예: [{"fruit_name": "apple", "quantity": 2}, ...]

            # 로봇팔 명령
            load_payload = {
                "cart_id": cart_id_str,
                "items": items_data
            }
            arm_msg = String()
            arm_msg.data = "LOAD|" + json.dumps(load_payload)
            self.arm_pub.publish(arm_msg)
            self.get_logger().info(f"Published to /arm_command: {arm_msg.data}")

            # loading_in_progress에 기록(로딩 완료 콜백을 기다릴 때 사용)
            self.loading_in_progress[cart_id_str] = load_payload

        except Exception as e:
            self.get_logger().error(f"Failed to process cart_id={cart_id_str}, error: {e}")

    def get_items_from_db(self, cart_id_str):
        """
        DB에서 cart_fruit, fruit 조인 조회 -> 아이템 리스트 반환
        """
        conn = get_db_connection()
        try:
            with conn.cursor() as cursor:
                sql = """
                SELECT f.fruit_name, cf.quantity
                FROM cart_fruit cf
                JOIN fruit f ON cf.fruit_id = f.fruit_id
                WHERE cf.cart_id=%s
                """
                cursor.execute(sql, (cart_id_str,))
                rows = cursor.fetchall()
                return [{"fruit_name": r["fruit_name"], "quantity": r["quantity"]} for r in rows]
        finally:
            conn.close()

    def on_arm_feedback_received(self, msg):
        """
        로봇팔이 로딩 완료 시 "/arm_feedback"으로 "LOADING_DONE|cart_id"를 보낸다고 가정.
        이를 수신하면 DB에서 배송 목적지 좌표 조회 -> 배달 로봇 명령
        """
        data = msg.data  # 예: "LOADING_DONE|123"
        if data.startswith("LOADING_DONE|"):
            cart_id_str = data.split("|")[1]
            self.get_logger().info(f"Arm feedback: LOADING_DONE for cart_id={cart_id_str}")

            # DB에서 member_id -> members.address_lat, address_lng 조회
            lat, lng = self.get_address_from_db(cart_id_str)

            # 배송 로봇 명령
            deliver_msg = String()
            deliver_msg.data = f"DELIVER|{cart_id_str}|{lat}|{lng}"
            self.delivery_pub.publish(deliver_msg)
            self.get_logger().info(f"Published /delivery_command: {deliver_msg.data}")

            # 로딩 중이던 cart_id를 메모리에서 제거(필요시)
            if cart_id_str in self.loading_in_progress:
                del self.loading_in_progress[cart_id_str]

    def get_address_from_db(self, cart_id_str):
        """
        cart_id -> cart.visit_id -> members.member_id -> (address_lat, address_lng)
        """
        conn = get_db_connection()
        lat, lng = 0.0, 0.0
        try:
            with conn.cursor() as cursor:
                sql = """
                SELECT c.visit_id, m.address_lat, m.address_lng
                FROM cart c
                JOIN members m ON c.visit_id = m.member_id
                WHERE c.cart_id = %s
                LIMIT 1
                """
                cursor.execute(sql, (cart_id_str,))
                row = cursor.fetchone()
                if row:
                    lat, lng = row["address_lat"], row["address_lng"]
        finally:
            conn.close()

        return (lat, lng)

def main(args=None):
    rclpy.init(args=args)
    node = OrderManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
