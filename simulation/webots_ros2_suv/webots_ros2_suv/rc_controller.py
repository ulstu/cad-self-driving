#!/usr/bin/env python

import os
import select
import sys
import rclpy

from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

channels = [1500, 1500, 1500, 1500, 1000, 1000, 1500, 1500, 1000, 1000]

msg = """
FS-I6 controller simulation node!
---------------------------

            U       I      
T     Y                    O     P
            J       K


     8                     W      

4          6          A          D  

     5                     S      




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


pub = None


def tx_timer_callback():
    rc_msg = Int32MultiArray()
    rc_msg.data = channels
    pub.publish(rc_msg)


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def print_channels():
    for i in range(10):
        print("\033[%d;%dH  %2d [" % (i + 5, 50, i + 1), end="")
        axisStr = ""
        for j in range(10, 0, -1):
            if 1500 - j * 50 >= channels[i]:
                axisStr += "#"
            else:
                axisStr += "-"
        axisStr += "|"
        for j in range(10):
            if 1500 + j * 50 >= channels[i]:
                axisStr += "-"
            else:
                axisStr += "#"
        axisStr += "]"
        print(axisStr)


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('rc_controller')
    global pub
    pub = node.create_publisher(Int32MultiArray, 'cmd_rc', qos)
    timer = node.create_timer(0.1, tx_timer_callback)
    status = 1

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0)

            if status == 1:
                os.system('clear')
                print(msg)
                print_channels()
                status = 0

            key = get_key(settings)
            if key == 'd':
                channels[0] = constrain(channels[0] + 50, 1000, 2000)
                status = 1
            elif key == 's':
                channels[1] = constrain(channels[1] - 50, 1000, 2000)
                status = 1
            elif key == 'w':
                channels[1] = constrain(channels[1] + 50, 1000, 2000)
                status = 1
            elif key == 'a':
                channels[0] = constrain(channels[0] - 50, 1000, 2000)
                status = 1

            elif key == 'i':
                channels[7] = constrain(channels[8] + 50, 1000, 2000)
                status = 1
            elif key == 'j':
                channels[6] = constrain(channels[9] - 50, 1000, 2000)
                status = 1
            elif key == 'u':
                channels[6] = constrain(channels[9] + 50, 1000, 2000)
                status = 1
            elif key == 'k':
                channels[7] = constrain(channels[8] - 50, 1000, 2000)
                status = 1

            elif key == '8':
                channels[2] = constrain(channels[2] + 50, 1000, 2000)
                status = 1
            elif key == '5':
                channels[2] = constrain(channels[2] - 50, 1000, 2000)
                status = 1
            elif key == '6':
                channels[3] = constrain(channels[3] + 50, 1000, 2000)
                status = 1
            elif key == '4':
                channels[3] = constrain(channels[3] - 50, 1000, 2000)
                status = 1
            elif key == 'p':
                if channels[5] < 1500:
                    channels[5] = 2000
                else:
                    channels[5] = 1000
                status = 1
            elif key == 'o':
                channels[4] += 500
                if channels[4] > 2000:
                    channels[4] = 1000
                status = 1
            elif key == 'y':
                if channels[9] < 1500:
                    channels[9] = 2000
                else:
                    channels[9] = 1000
                status = 1
            elif key == 't':
                if channels[8] < 1500:
                    channels[8] = 2000
                else:
                    channels[8] = 1000
                status = 1
            else:
                if key == '\x03':
                    break

    except Exception as e:
        print(e)

    finally:
        rcMsg = Int32MultiArray()
        rcMsg.data = channels
        pub.publish(rcMsg)
        os.system('clear')

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
