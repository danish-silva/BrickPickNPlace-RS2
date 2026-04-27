import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hari/git/BrickPickNPlace-RS2/src/install/ur3e_motion'
