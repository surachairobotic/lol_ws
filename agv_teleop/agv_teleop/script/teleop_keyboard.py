import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# Define maximum linear and angular velocities for your AGV model here
MAX_LIN_VEL = 0.26
MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.04
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your AGV!
---------------------------
Moving around:
     G
  V  B  N

G/B : increase/decrease linear velocity ( limit 0.04 m/s -0.26 m/s )
V/N : increase/decrease angular velocity ( limit 0.1 m/s -2.84 m/s

Spacebar, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'g':  # Arrow Up
                target_linear_velocity += LIN_VEL_STEP_SIZE
                target_linear_velocity = min(target_linear_velocity, MAX_LIN_VEL)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'b':  # Arrow Down
                target_linear_velocity -= LIN_VEL_STEP_SIZE
                target_linear_velocity = max(target_linear_velocity, -MAX_LIN_VEL)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'v':  # Arrow Left
                target_angular_velocity += ANG_VEL_STEP_SIZE
                target_angular_velocity = min(target_angular_velocity, MAX_ANG_VEL)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'n':  # Arrow Right
                target_angular_velocity -= ANG_VEL_STEP_SIZE
                target_angular_velocity = max(target_angular_velocity, -MAX_ANG_VEL)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':  # Spacebar or 's'
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()

