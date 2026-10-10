import rospy
import numpy as np
import math as m
import tf.transformations as tft
import raven_2.msg

from utils import ambf_motor_def as amd

class ambf_raven_state:
    
    def __init__(self, robot_state):
        self.robot_state = robot_state
        self.add_pos = np.zeros((3))
        self.add_ori = np.zeros((4))
        self.old_mpos = np.zeros((8))
        self.tm = None
        
        # Encoder Definition
        self.ROTARY_JOINT_ENC_PER_REV = amd.ROTARY_JOINT_ENC_PER_REV
        self.LINEAR_JOINT_ENC_PER_M = amd.LINEAR_JOINT_ENC_PER_M
        
        # Motor Definition
        self.SHOULDER_TR_GREEN_ARM = amd.SHOULDER_TR_GREEN_ARM
        self.ELBOW_TR_GREEN_ARM = amd.ELBOW_TR_GREEN_ARM
        self.Z_INS_TR_GREEN_ARM = amd.Z_INS_TR_GREEN_ARM
        self.TOOL_ROT_TR_GREEN_ARM = amd.TOOL_ROT_TR_GREEN_ARM
        self.WRIST_TR_GREEN_ARM = amd.WRIST_TR_GREEN_ARM
        self.GRASP1_TR_GREEN_ARM = amd.GRASP1_TR_GREEN_ARM
        self.GRASP2_TR_GREEN_ARM = amd.GRASP2_TR_GREEN_ARM
        
        # Sequence
        self.pub_count = 0
        
        # End-Effector Pose, Pose Desired
        self.pos = np.zeros((3))
        self.pos_d = np.zeros((3))

        # End-Effector Orientation, Orientation Desired
        self.ori = np.zeros((9))
        self.ori_d = np.zeros((9))
        
        # Encoder Value, Offset
        self.enc_val = np.zeros((8))
        self.enc_off = np.zeros((8))

        # Motor Pose, Pose Desired, velocity
        self.mpos = np.zeros((8))
        self.mpos_d = np.zeros((8))
        self.mvel = np.zeros((8))
        
        # Joint Pose, Pose Desired, velocity
        self.jpos = np.zeros((8))
        self.jpos_d = np.zeros((8))
        self.jvel = np.zeros((8))
          
        # Desired Grasper Pose
        self.grasp_d = np.zeros((1))
          
        # Publisher
        self.__publisher_raven_state = rospy.Publisher('/ambf/raven_state', raven_2.msg.raven_state, latch = True, queue_size = 1)
        return None
        
    def __update_from_robot_state(self):
        self.tm = self.robot_state.cartesian_pose
        self.add_pos[:] = self.robot_state.delta_pos
        self.add_ori[:] = self.robot_state.delta_ori
        return None
    
    def __end_effector_pose(self):
        self.pos = [int(i) for i in (np.array(self.tm[:3,3]) * 1e6)]
        return None
    
    def __end_effector_pose_desired(self):
        self.pos_d = [p + int(ap * 1e6) for p, ap in zip(self.pos, self.add_pos)]
        return None
    
    def __end_effector_orientation(self):
        self.ori = list(self.tm[:3, :3].flat)
        return None

    def __end_effector_orientation_desired(self):
        q2r = tft.quaternion_matrix([self.add_ori[0], self.add_ori[1], self.add_ori[2], self.add_ori[3]])
        ori_mat = np.array(self.ori).reshape(3, 3)
        ori_d_mat = np.matmul(ori_mat, q2r[:3, :3])
        self.ori_d = list(ori_d_mat.flat)
        return None
    
    def __encoder_value(self):
        self.enc_val[0] = int(self.jpos[0] * (self.ROTARY_JOINT_ENC_PER_REV / m.pi) + self.enc_off[0])
        self.enc_val[1] = int(self.jpos[1] * (self.ROTARY_JOINT_ENC_PER_REV / m.pi) + self.enc_off[1])
        self.enc_val[2] = int(self.jpos[2] * (self.ROTARY_JOINT_ENC_PER_REV / m.pi) + self.enc_off[2])
        
        self.enc_val[4] = int(self.jpos[4] * self.LINEAR_JOINT_ENC_PER_M + self.enc_off[4])
        
        self.enc_val[5] = int(self.jpos[5] * (self.ROTARY_JOINT_ENC_PER_REV / m.pi) + self.enc_off[5])
        self.enc_val[6] = int(self.jpos[6] * (self.ROTARY_JOINT_ENC_PER_REV / m.pi) + self.enc_off[6])
        self.enc_val[7] = int(self.jpos[7] * (self.ROTARY_JOINT_ENC_PER_REV / m.pi) + self.enc_off[7])
        return None
    
    def __encoder_offset(self):
        self.enc_off[0:8] = 0
        return None
    
    def __motor_pose(self):
        self.old_mpos[:] = self.mpos
        self.mpos[0] = self.jpos[0] * self.SHOULDER_TR_GREEN_ARM
        self.mpos[1] = self.jpos[1] * self.ELBOW_TR_GREEN_ARM
        self.mpos[2] = self.jpos[2] * self.Z_INS_TR_GREEN_ARM
        self.mpos[4] = self.jpos[4] * self.TOOL_ROT_TR_GREEN_ARM
        self.mpos[5] = self.jpos[5] * self.WRIST_TR_GREEN_ARM
        self.mpos[6] = self.jpos[6] * self.GRASP1_TR_GREEN_ARM
        self.mpos[7] = self.jpos[7] * self.GRASP2_TR_GREEN_ARM
        return None
    
    def __motor_pose_desired(self):
        self.mpos_d[0] = self.jpos_d[0] * self.SHOULDER_TR_GREEN_ARM
        self.mpos_d[1] = self.jpos_d[1] * self.ELBOW_TR_GREEN_ARM
        self.mpos_d[2] = self.jpos_d[2] * self.Z_INS_TR_GREEN_ARM
        self.mpos_d[4] = self.jpos_d[4] * self.TOOL_ROT_TR_GREEN_ARM
        self.mpos_d[5] = self.jpos_d[5] * self.WRIST_TR_GREEN_ARM
        self.mpos_d[6] = self.jpos_d[6] * self.GRASP1_TR_GREEN_ARM
        self.mpos_d[7] = self.jpos_d[7] * self.GRASP2_TR_GREEN_ARM
        return None
    
    def __motor_velocity(self):
        self.mvel = (self.mpos - self.old_mpos) / 0.001
        return None
    
    def __joint_pose(self):
        self.jpos[0:3] = np.rad2deg(self.robot_state.joint_positions[0:3])
        self.jpos[4:8] = np.rad2deg(self.robot_state.joint_positions[3:7])
        return None
    
    def __joint_pose_desired(self):
        self.jpos_d[0:3] = np.rad2deg(self.robot_state.joint_targets[0:3])
        self.jpos_d[4:8] = np.rad2deg(self.robot_state.joint_targets[3:7])
        return None
    
    def __joint_velocity(self):
        self.jvel[0:3] = self.robot_state.joint_velocities[0:3]
        self.jvel[4:8] = self.robot_state.joint_velocities[3:7]
        return None
    
    def __grasper_pose_desired(self):
        self.grasp_d = self.jpos_d[6] + self.jpos_d[7]
        return None

    def pub_raven_state(self):
        self.__update_from_robot_state()
        
        self.__end_effector_pose()  # pos
        self.__end_effector_pose_desired()  # pos_d
        
        self.__end_effector_orientation()  # ori
        self.__end_effector_orientation_desired()  # ori_d
        
        self.__encoder_value() # encVals
        self.__encoder_offset() # encoffsets
        
        self.__motor_pose()  # mpos
        self.__motor_pose_desired()  # mpos_d
        self.__motor_velocity()  # mvel
        
        self.__joint_pose()  # jpos
        self.__joint_pose_desired()  # jpos_d
        self.__joint_velocity()  # jvel
        
        self.__grasper_pose_desired()  # grasp_d

        msg = raven_2.msg.raven_state()
        msg.hdr.seq = self.pub_count
        msg.hdr.stamp = rospy.Time.now()
        msg.hdr.frame_id = "base_link"
        msg.pos[0:3] = self.pos
        msg.ori[0:9] = self.ori
        msg.pos_d[0:3] = self.pos_d
        msg.ori_d[0:9] = self.ori_d
        msg.jpos[0:8] = self.jpos
        msg.jvel[0:8] = self.jvel
        msg.mvel[0:8] = self.mvel
        msg.encVals[0:8] = [int(val) for val in self.enc_val[:8]]
        msg.mpos[0:8] = self.mpos
        msg.jpos_d[0:8] = self.jpos_d
        msg.mpos_d[0:8] = self.mpos_d
        msg.grasp_d[0] = self.grasp_d

        self.__publisher_raven_state.publish(msg)
        
        self.pub_count += 1
        return None
    