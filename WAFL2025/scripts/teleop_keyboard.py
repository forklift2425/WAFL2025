#!/usr/bin/env python3
import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

# Key mappings
move_bindings = {
    'w': (1.0,  0.0, 0.0),   # forward
    's': (-1.0, 0.0, 0.0),   # backward
    'a': (0.0,  0.0, 1.0),   # turn left
    'd': (0.0,  0.0, -1.0),  # turn right
    'q': (0.0,  1.0, 0.0),   # strafe left
    'e': (0.0, -1.0, 0.0),   # strafe right
    'z': (0.0, 0.0, 0.0),   # strafe right
    ' ': (0.0,  0.0, 0.0),   # stop
}

speed = 1.0    # linear speed (m/s)
turn  = 1.0    # angular speed (rad/s)

def getKey(timeout):
    """Read single keypress from stdin with timeout."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        else:
            return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def teleop():
    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.loginfo("Use WASD to drive, Q/E to strafe, Space to stop, Ctrl-C to quit")

    twist = Twist()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        key = getKey(0.1)
        if key in move_bindings:
            lin_x, lin_y, ang_z = move_bindings[key]
            twist.linear.x  = lin_x * speed
            twist.linear.y  = lin_y * speed
            twist.angular.z = ang_z * turn
            pub.publish(twist)
            if(key=='z'):
                twist.linear.z  = 1.0
            else :
                twist.linear.z  = 0.0

        elif key == '\x03':  # Ctrl-C
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
