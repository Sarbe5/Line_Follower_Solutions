import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aryanbhalkikar/RISC_Hack_PS/ros2_ps_ws/install/my_py_pkg'
