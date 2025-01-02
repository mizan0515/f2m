import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/truman/ws/f2m/install/f2m_Delivery_Robot'
