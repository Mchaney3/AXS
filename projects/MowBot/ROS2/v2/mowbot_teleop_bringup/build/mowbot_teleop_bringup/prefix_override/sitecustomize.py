import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dadmin/mowbot_teleop_bringup/install/mowbot_teleop_bringup'
