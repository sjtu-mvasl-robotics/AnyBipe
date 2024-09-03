import rospy
import numpy as np
from gazebo_msgs.msg import LinkStates, ModelStates
from std_msgs.msg import Header
from copy import deepcopy
from nav_msgs.msg import Path

# Purpose: convert /gazebo/link_states and /gazebo/model_states to nav_msgs/Path

class State2Traj:
    def __init__(self):
        self.link_states = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)
        self.model_states = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        # link states names:
        '''
                - ground_plane::link
                - point_foot_robot::base_Link
                - point_foot_robot::abad_L_Link
                - point_foot_robot::hip_L_Link
                - point_foot_robot::knee_L_Link
                - point_foot_robot::foot_L_Link
                - point_foot_robot::abad_R_Link
                - point_foot_robot::hip_R_Link
                - point_foot_robot::knee_R_Link
                - point_foot_robot::foot_R_Link
                - point_foot_robot::limx_imu        
        '''