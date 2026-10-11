import rospy
import sys
import time

from . import crtk_api
from . import robot_state
from . import update_device_state
from . import ambf_raven_state
from . import raven_toolkits

from ambf_client import Client
from ..kinematics import ambf_raven_def as ard

# Rate of this loop and of the commands the AMBF client sends to the simulator. The motion logic uses the
# real elapsed time, so this only sets how often the setpoint and the measured/raven_state topics update.
LOOP_RATE = 500

class main:
    def __init__(self):
        rospy.init_node("AMBF_raven2_controller", anonymous=True)

        # The AMBF client only discovers objects when it connects, so wait for the arm first.
        deadline = time.time() + 30
        while not any(topic.endswith('/base_link_L/State') for topic, _ in rospy.get_published_topics()):
            if time.time() > deadline or rospy.is_shutdown():
                sys.exit("AMBF RAVEN-II not found (no base_link_L topic). Is ambf_simulator running with raven_straight.yaml?")
            rospy.logwarn_throttle(5, "Waiting for AMBF RAVEN-II to load...")
            time.sleep(0.5)

        self.ambf_client = Client()
        self.ambf_client.connect(default_publish_rate=LOOP_RATE)
        time.sleep(1)
        self.arm = None
        for _ in range(50):
            self.arm = self.ambf_client.get_obj_handle('base_link_L')
            if self.arm is not None:
                break
            rospy.logwarn("Waiting for AMBF object: base_link_L")
            time.sleep(0.1)
        if self.arm is None:
            sys.exit("AMBF object not found: base_link_L")

        # The client subscribes to every AMBF object (about 20 more State topics and Command publishers).
        # Only the arm is used, and the extra rospy threads starve this loop of the GIL. The AMBF 2.0
        # client has no public way to drop objects, hence the private _objects_dict; the objects stay in
        # it (its publisher thread iterates it, and only publishes active objects anyway).
        for obj in list(self.ambf_client._objects_dict.values()):
            if obj is not self.arm:
                for attr in ("_sub", "_pub", "_reset_pub", "_reset_bodies_pub"):
                    if getattr(obj, attr, None) is not None:
                        getattr(obj, attr).unregister()

        # Re-subscribe to the arm state with TCP_NODELAY; the client subscribes without it, so the
        # ~1 kHz state messages get batched by Nagle's algorithm.
        sub = self.arm._sub
        self.arm._sub = rospy.Subscriber(sub.resolved_name, sub.data_class, self.arm.ros_cb, queue_size=1, tcp_nodelay=True)
        sub.unregister()

        self.robot_state = robot_state.robot_state()
        self.crtk_api = crtk_api.crtk_api(self.robot_state)
        self.update_device_state = update_device_state.update_device_state(self.robot_state, self.arm)
        self.ambf_raven_state = ambf_raven_state.ambf_raven_state(self.robot_state)
        self.raven_toolkits = raven_toolkits.raven_toolkits(self.arm)
        
        rospy.loginfo("CRTK to AMBF Adapter is connecting...")
        time.sleep(1)
        rospy.loginfo("CRTK to AMBF Adapter initialized and connected to AMBF.")
        
        return None
        
        
    def run(self):
        rate = rospy.Rate(LOOP_RATE)
        while not rospy.is_shutdown():
            
            self.robot_state.update_from_ambf(self.arm)
            
            self.crtk_api.pub_measured_js()
            self.crtk_api.pub_measured_cp()
            
            self.ambf_raven_state.pub_raven_state()
            
            self.update_device_state.update_crtk_motion()

            rate.sleep()
            
        return None
    
    def home(self):
        self.raven_toolkits.set_home()
        self.robot_state.joint_targets[:] = ard.HOME_JOINTS
        print("Set Home")
        return None
        

def start():
    controller = main()
    controller.home()
    controller.run()


if __name__ == "__main__":
    start()
