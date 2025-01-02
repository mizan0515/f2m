# my_server_pkg/db_utils.py

import pymysql

def get_db_connection():
    """
    MySQL DB 커넥션을 반환.
    실제 환경에 맞게 호스트, 유저, 비번, DB 스키마 등을 수정하세요.
    """
    conn = pymysql.connect(
        host='localhost',
        user='root',
        password='whdgh29k05',
        db='f2m',  # 예: my_fruit_db
        charset='utf8',
        cursorclass=pymysql.cursors.DictCursor
    )
    return conn
