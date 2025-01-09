import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pi/mowbot_pkg_v2_lidar/install/mowbot_pkg'
