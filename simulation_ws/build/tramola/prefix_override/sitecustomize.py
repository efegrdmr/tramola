import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/efe/Desktop/tramola/simulation_ws/install/tramola'
