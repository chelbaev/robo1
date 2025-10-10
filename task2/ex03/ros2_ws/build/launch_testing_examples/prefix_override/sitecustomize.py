import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chelbaev/NSU/robots/ex03/ros2_ws/install/launch_testing_examples'
