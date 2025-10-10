import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chelbaev/NSU/robots/ros2_ws/install/ten_pub'
