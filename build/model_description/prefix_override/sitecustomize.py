import sys
if sys.prefix == '/home/darren/miniconda3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/darren/Desktop/Prob_Project_ws/install/model_description'
