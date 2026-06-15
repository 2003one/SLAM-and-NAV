import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/the2003onean/Downloads/Auto-main/ros/install/ros'
