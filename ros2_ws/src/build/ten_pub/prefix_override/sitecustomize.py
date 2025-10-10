import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chelbaev/NSU/robots/ros2_ws/src/install/ten_pub'
