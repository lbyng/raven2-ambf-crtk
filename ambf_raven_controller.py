import time
import rospy
import sensor_msgs.msg
import geometry_msgs.msg

class ambf_raven_controller:
    def __init__(self):
        self.__init_pub_sub()
        
        self.max_rate_move = 20
        self.min_interval_move = 1.0/self.max_rate_move
        self.time_last_pub_move = -1.0
        return None
    
    def __init_pub_sub(self):
        self.__publisher_servo_jr = rospy.Publisher('/arm1/servo_jr', sensor_msgs.msg.JointState, queue_size = 1)
        self.__publisher_servo_cr = rospy.Publisher('/arm1/servo_cr', geometry_msgs.msg.TransformStamped, queue_size = 1)
        return None
        
    def pub_servo_jr_command(self, joint_command):

        joint_command = joint_command[1:]
        msg = sensor_msgs.msg.JointState()
        msg.position = list(joint_command.flat)

        interval_pub = time.time() - self.time_last_pub_move

        if self.time_last_pub_move != -1.0:
            interval_pub = time.time() - self.time_last_pub_move
            if interval_pub < self.min_interval_move:
                time.sleep(self.min_interval_move - interval_pub)
                
        self.__publisher_servo_jr.publish(msg)
        self.time_last_pub_move = time.time()
        return 1
    
    def pub_servo_cr_command(self, position, rotation):
        msg = geometry_msgs.msg.TransformStamped()
        
        msg.transform.translation.x = position[0]
        msg.transform.translation.y = position[1]
        msg.transform.translation.z = position[2]
        
        msg.transform.rotation.x = rotation[0]
        msg.transform.rotation.y = rotation[1]
        msg.transform.rotation.z = rotation[2]
        msg.transform.rotation.w = rotation[3]

        self.__publisher_servo_cr.publish(msg)
        return 1