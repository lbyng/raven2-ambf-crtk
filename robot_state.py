import ambf_msgs.msg
import collections
import rospy
import numpy as np

from utils import raven_fk as fk
from utils import ambf_raven_def as ard

class robot_state:
    def __init__(self):
        self.joint_positions = np.zeros((7))
        self.joint_velocities = np.zeros((7))
        self.joint_efforts = np.zeros((7))
        
        self.joint_targets = np.zeros((7))
        self.joint_delta = np.zeros((7))
        
        self.cartesian_pose = None
        
        self.delta_pos = np.zeros((3))
        self.delta_ori = np.zeros((4))
        
        self.control_mode = "JOINT"

        # servo commands queued by the ROS callback thread, applied one per control cycle
        self.commands = collections.deque(maxlen = 1000)

        return None

    def update_from_ambf(self, arm):
        """
        update robot state from AMBF
        """
        for i in range(7):
            self.joint_positions[i] = arm.get_joint_pos(i)
            self.joint_velocities[i] = arm.get_joint_vel(i)
        
        self.cartesian_pose = fk.fwd_kinematics(0, self.joint_positions, ard)
           
        return None