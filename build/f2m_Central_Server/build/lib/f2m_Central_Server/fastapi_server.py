#!/usr/bin/env python3
# my_server_pkg/fastapi_server.py

import uvicorn
from fastapi import FastAPI, HTTPException
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .db_utils import get_db_connection

app = FastAPI(title="Delivery Robot FastAPI")

# ROS2 노드 전역 초기화
rclpy.init(args=None)
ros_node = Node("fastapi_ros_node")
publisher_new_order = ros_node.create_publisher(String, "/new_order", 10)

@app.post("/cart/add")
def add_item_to_cart(member_id: int, fruit_id: int, quantity: int):
    """
    특정 회원(member_id)의 온라인 장바구니(cart)에 과일 추가
    (cart_fruit 테이블 업데이트)
    """
    conn = get_db_connection()
    try:
        with conn.cursor() as cursor:
            # 1) 미결제( purchased=0 ) 온라인 장바구니(cart) 찾기 or 없으면 생성
            sql_cart_find = """
                SELECT cart_id FROM cart
                WHERE purchased=0 AND cart_type='ONLINE' AND visit_id=%s
                LIMIT 1
            """
            cursor.execute(sql_cart_find, (member_id,))
            row = cursor.fetchone()
            if row:
                cart_id = row['cart_id']
            else:
                # 새로운 cart 생성
                sql_cart_insert = """
                    INSERT INTO cart (visit_id, purchased, cart_type, shipping_address)
                    VALUES (%s, 0, 'ONLINE', '')
                """
                cursor.execute(sql_cart_insert, (member_id,))
                cart_id = cursor.lastrowid

            # 2) cart_fruit에 과일 추가 or 수량 업데이트
            sql_cf_find = "SELECT quantity FROM cart_fruit WHERE cart_id=%s AND fruit_id=%s"
            cursor.execute(sql_cf_find, (cart_id, fruit_id))
            exist = cursor.fetchone()
            if exist:
                new_qty = exist['quantity'] + quantity
                sql_cf_update = """
                    UPDATE cart_fruit SET quantity=%s
                    WHERE cart_id=%s AND fruit_id=%s
                """
                cursor.execute(sql_cf_update, (new_qty, cart_id, fruit_id))
            else:
                sql_cf_insert = """
                    INSERT INTO cart_fruit (cart_id, fruit_id, quantity)
                    VALUES (%s, %s, %s)
                """
                cursor.execute(sql_cf_insert, (cart_id, fruit_id, quantity))

            conn.commit()
            return {"cart_id": cart_id, "fruit_id": fruit_id, "quantity": quantity}
    finally:
        conn.close()

@app.post("/cart/checkout")
def checkout_cart(member_id: int):
    """
    온라인 장바구니 결제 -> purchased=1 갱신 -> /new_order 로 cart_id 퍼블리시
    """
    conn = get_db_connection()
    try:
        with conn.cursor() as cursor:
            sql_cart_find = """
                SELECT cart_id FROM cart
                WHERE purchased=0 AND cart_type='ONLINE' AND visit_id=%s
            """
            cursor.execute(sql_cart_find, (member_id,))
            row = cursor.fetchone()
            if not row:
                raise HTTPException(status_code=400, detail="No open cart for this member.")

            cart_id = row['cart_id']
            sql_purchase = """
                UPDATE cart SET purchased=1, pur_dttm=NOW()
                WHERE cart_id=%s
            """
            cursor.execute(sql_purchase, (cart_id,))
            conn.commit()

            # ROS2 퍼블리시 (cart_id)
            msg = String()
            msg.data = str(cart_id)
            publisher_new_order.publish(msg)

            return {"message": "Checkout completed.", "cart_id": cart_id}
    finally:
        conn.close()

def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
