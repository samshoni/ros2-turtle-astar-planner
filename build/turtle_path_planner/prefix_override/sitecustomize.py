import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sam/turtle_path_planner_ws/install/turtle_path_planner'
