import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wormi/sprint4/v6/g06_prii3_wsv2/install/aruco_detector_pkg'
