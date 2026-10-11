import tty, sys, termios
from . import ambf_raven_controller
import rospy
import numpy as np
from . import utils_keyboard_controller as utils
import tf.transformations as tft

velocity_joint_1 = 3 # degree/s
velocity_joint_2 = 3 # degree/s
velocity_joint_3 = 0.01 # m/s
velocity_joint_4 = 10 # degree/s
velocity_joint_5 = 10 # degree/s
velocity_joint_6 = 10 # degree/s
velocity_joint_7 = 10 # degree/s


def main():
    filedescriptors = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)

    rospy.init_node('raven_keyboard_controller', anonymous=True)

    controller = ambf_raven_controller.ambf_raven_controller()

    utils.print_menu()

    while True:

        input_key = sys.stdin.read(1)[0]
        termios.tcflush(sys.stdin, termios.TCIOFLUSH)

        if input_key == '9':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)
            sys.exit('\nClosing AMBF RAVEN 2 Keyboard Controller...')

        # Joint control----------------------------------------------

        if input_key == '1':
            utils.print_no_newline(" Moving: Joint 1 +++         ")
            cmd = np.zeros((16))
            cmd[1] = np.deg2rad(velocity_joint_1)
            controller.pub_servo_jr_command(cmd)

        elif input_key == 'q':
            utils.print_no_newline(" Moving: Joint 1 ---         ")
            cmd = np.zeros((16))
            cmd[1] = -np.deg2rad(velocity_joint_1)
            controller.pub_servo_jr_command(cmd)

        elif input_key == '2':
            utils.print_no_newline(" Moving: Joint 2 +++         ")
            cmd = np.zeros((16))
            cmd[2] = np.deg2rad(velocity_joint_2)
            controller.pub_servo_jr_command(cmd)

        elif input_key == 'w':
            utils.print_no_newline(" Moving: Joint 2 ---         ")
            cmd = np.zeros((16))
            cmd[2] = -np.deg2rad(velocity_joint_2)
            controller.pub_servo_jr_command(cmd)

        elif input_key == '3':
            utils.print_no_newline(" Moving: Joint 3 +++         ")
            cmd = np.zeros((16))
            cmd[3] = velocity_joint_3
            controller.pub_servo_jr_command(cmd)

        elif input_key == 'e':
            utils.print_no_newline(" Moving: Joint 3 ---         ")
            cmd = np.zeros((16))
            cmd[3] = -velocity_joint_3
            controller.pub_servo_jr_command(cmd)

        elif input_key == '4':
            utils.print_no_newline(" Moving: Joint 4 +++         ")
            cmd = np.zeros((16))
            cmd[4] = np.deg2rad(velocity_joint_4)
            controller.pub_servo_jr_command(cmd)

        elif input_key == 'r':
            utils.print_no_newline(" Moving: Joint 4 ---         ")
            cmd = np.zeros((16))
            cmd[4] = -np.deg2rad(velocity_joint_4)
            controller.pub_servo_jr_command(cmd)

        elif input_key == '5':
            utils.print_no_newline(" Moving: Joint 5 +++         ")
            cmd = np.zeros((16))
            cmd[5] = np.deg2rad(velocity_joint_5)
            controller.pub_servo_jr_command(cmd)

        elif input_key == 't':
            utils.print_no_newline(" Moving: Joint 5 ---         ")
            cmd = np.zeros((16))
            cmd[5] = -np.deg2rad(velocity_joint_5)
            controller.pub_servo_jr_command(cmd)

        elif input_key == '6':
            utils.print_no_newline(" Moving: Joint 6 +++         ")
            cmd = np.zeros((16))
            cmd[6] = np.deg2rad(velocity_joint_6)
            controller.pub_servo_jr_command(cmd)

        elif input_key == 'y':
            utils.print_no_newline(" Moving: Joint 6 ---         ")
            cmd = np.zeros((16))
            cmd[6] = -np.deg2rad(velocity_joint_6)
            controller.pub_servo_jr_command(cmd)

        elif input_key == '7':
            utils.print_no_newline(" Moving: Joint 7 +++         ")
            cmd = np.zeros((16))
            cmd[7] = np.deg2rad(velocity_joint_7)
            controller.pub_servo_jr_command(cmd)

        elif input_key == 'u':
            utils.print_no_newline(" Moving: Joint 7 ---         ")
            cmd = np.zeros((16))
            cmd[7] = -np.deg2rad(velocity_joint_7)
            controller.pub_servo_jr_command(cmd)

        elif input_key == 'o':
            utils.print_no_newline(" Moving: Grasper open         ")
            cmd = np.zeros((16))
            cmd[6] = np.deg2rad(velocity_joint_6)
            cmd[7] = np.deg2rad(velocity_joint_7)
            controller.pub_servo_jr_command(cmd)

        elif input_key == 'p':
            utils.print_no_newline(" Moving: Grasper close         ")
            cmd = np.zeros((16))
            cmd[6] = -np.deg2rad(velocity_joint_6)
            cmd[7] = -np.deg2rad(velocity_joint_7)
            controller.pub_servo_jr_command(cmd)

        # Cartisian control----------------------------------------------

        elif input_key == 's':
            utils.print_no_newline(' Moving: Grasper forward    ')
            pos = np.zeros((3))
            rot = np.zeros((4))
            pos[0] = 0.01
            controller.pub_servo_cr_command(pos, rot)

        elif input_key == 'x':
            utils.print_no_newline(' Moving: Grasper backward    ')
            pos = np.zeros((3))
            rot = np.zeros((4))
            pos[0] = -0.01
            controller.pub_servo_cr_command(pos, rot)

        elif input_key == 'z':
            utils.print_no_newline(' Moving: Grasper left    ')
            pos = np.zeros((3))
            rot = np.zeros((4))
            pos[1] = 0.01
            controller.pub_servo_cr_command(pos, rot)

        elif input_key == 'c':
            utils.print_no_newline(' Moving: Grasper right    ')
            pos = np.zeros((3))
            rot = np.zeros((4))
            pos[1] = -0.01
            controller.pub_servo_cr_command(pos, rot)

        elif input_key == 'f':
            utils.print_no_newline(' Moving: Grasper up    ')
            pos = np.zeros((3))
            rot = np.zeros((4))
            pos[2] = -0.01
            controller.pub_servo_cr_command(pos, rot)

        elif input_key == 'v':
            utils.print_no_newline(' Moving: Grasper down    ')
            pos = np.zeros((3))
            rot = np.zeros((4))
            pos[2] = 0.01
            controller.pub_servo_cr_command(pos, rot)

        elif input_key == 'h':
            utils.print_no_newline(' Moving: Grasper P -    ')
            pos = np.zeros((3))
            rot = tft.quaternion_from_euler(0, -0.01, 0)
            controller.pub_servo_cr_command(pos, rot)

        elif input_key == 'n':
            pos = np.zeros((3))
            rot = tft.quaternion_from_euler(0, 0.01, 0)
            controller.pub_servo_cr_command(pos, rot)
            utils.print_no_newline(' Moving: Grasper P +    ')

        elif input_key == 'b':
            pos = np.zeros((3))
            rot = tft.quaternion_from_euler(-0.01, 0, 0)
            controller.pub_servo_cr_command(pos, rot)
            utils.print_no_newline(' Moving: Grasper R -    ')

        elif input_key == 'm':
            pos = np.zeros((3))
            rot = tft.quaternion_from_euler(0.01, 0, 0)
            controller.pub_servo_cr_command(pos, rot)
            utils.print_no_newline(' Moving: Grasper R +    ')

        elif input_key == 'g':
            pos = np.zeros((3))
            rot = tft.quaternion_from_euler(0, 0, -0.01)
            controller.pub_servo_cr_command(pos, rot)
            utils.print_no_newline(' Moving: Grasper Y -    ')

        elif input_key == 'j':
            pos = np.zeros((3))
            rot = tft.quaternion_from_euler(0, 0, 0.01)
            controller.pub_servo_cr_command(pos, rot)
            utils.print_no_newline(' Moving: Grasper Y +    ')

        else:
            utils.print_no_newline(' Unknown command         ')


if __name__ == "__main__":
    main()
