import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/harold/Workspace/Quads_Project/install/aruco_detector_node'
