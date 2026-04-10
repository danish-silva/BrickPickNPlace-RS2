import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/b/ws_moveit2/src/install/moveit_configs_utils'
