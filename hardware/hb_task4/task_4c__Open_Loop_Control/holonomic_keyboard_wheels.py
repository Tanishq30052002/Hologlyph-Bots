import sys
import threading

import geometry_msgs.msg
import std_msgs.msg
import rclpy

import termios
import tty

import math

msg = """
Holonomic Keyboard Coontrols
---------------------------
Linear Motion:
    i
j       l
    ,

Angular Motion:
p       ]    

Pen Motion:
+   -

Anything Else: 
    Stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'l': (1, 0, 0, 0),
    'j': (-1, 0, 0, 0),

    'i': (0, 1, 0, 0),
    ',': (0, -1, 0, 0),

    ']': (0, 0, 0, -1),
    'p': (0, 0, 0, 1),
}

speedBindings = {
    'q': (0.1, 0.1),
    'z': (-0.1, -0.1),
    'w': (0.1, 1),
    'x': (-0.1, 1),
    'e': (1, 0.1),
    'c': (1, -0.1),
}

penBindings = {
    '+': (True, ),
    '-': (False, ),
}

def bound(value):
    if(value<0):
        value=0.0
    elif(value>180):
        value=180.0
    return round(value,1)

def inverse_kinematics(vx, vy, w):
    f_left = w/3 - vx/2 - vy*math.cos(math.pi/6)
    f_right = w/3 - vx/2 + vy*math.cos(math.pi/6)
    f_rear = w/3 + 0.7*vx


    left = 50 * f_left
    right = 50 * f_right
    rear = 50 * f_rear

    return [left, right, rear]

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('hb_keyboard')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    motion_pub1 = node.create_publisher(TwistMsg, '/cmd_vel/bot1', 10)
    motion_pub2 = node.create_publisher(TwistMsg, '/cmd_vel/bot2', 10)
    motion_pub3 = node.create_publisher(TwistMsg, '/cmd_vel/bot3', 10)
    pen_pub1 = node.create_publisher(std_msgs.msg.Bool, '/pen1_down', 10)
    pen_pub2 = node.create_publisher(std_msgs.msg.Bool, '/pen2_down', 10)
    pen_pub3 = node.create_publisher(std_msgs.msg.Bool, '/pen3_down', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    pen_status = False
    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    th = 0.0

    twist_msg = TwistMsg()
    pen_msg = std_msgs.msg.Bool()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][3]

            elif key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
                turn = turn + speedBindings[key][1]

                print(vels(speed, turn))

            elif key in penBindings.keys():
                pen_status = penBindings[key][0]

            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                pen_status = False
                if (key == '\x03'):
                    break

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            left, right, rear = inverse_kinematics(x*speed, y*speed, th*turn)
            twist.linear.x = left
            twist.linear.y = right
            twist.linear.z = rear
            motion_pub1.publish(twist_msg)
            motion_pub2.publish(twist_msg)
            motion_pub3.publish(twist_msg)
            print("BOT1: left: ", left, " right: ",right, "rear: ", rear)
            print("BOT2: left: ", left, " right: ",right, "rear: ", rear)
            print("BOT3: left: ", left, " right: ",right, "rear: ", rear)

            pen_msg.data = pen_status
            pen_pub1.publish(pen_msg)
            pen_pub2.publish(pen_msg)
            pen_pub3.publish(pen_msg)

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        motion_pub1.publish(twist_msg)
        motion_pub2.publish(twist_msg)
        motion_pub3.publish(twist_msg)

        pen_msg.data = False
        pen_pub1.publish(pen_msg)
        pen_pub2.publish(pen_msg)
        pen_pub3.publish(pen_msg)

        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()