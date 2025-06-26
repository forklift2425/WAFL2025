#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def adjust_and_move():
    # Initialize the node
    rospy.init_node('adjust_and_move_robot', anonymous=True)

    # Create publishers for steering and drive joints
    leftweb_pub = rospy.Publisher('/leftweb_steer_controller/command', Float64, queue_size=10)
    rightweb_pub = rospy.Publisher('/rightweb_steer_controller/command', Float64, queue_size=10)
    castorjoint_pub = rospy.Publisher('/castorjoint_steer_controller/command', Float64, queue_size=10)

    leftwheel_pub = rospy.Publisher('/leftwheel_drive_controller/command', Float64, queue_size=10)
    rightwheel_pub = rospy.Publisher('/rightwheel_drive_controller/command', Float64, queue_size=10)
    castorwheel_pub = rospy.Publisher('/castorwheel_drive_controller/command', Float64, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 10 Hz

    # Adjust steering joints to their specified positions
    rospy.loginfo("Adjusting steering joints...")
    leftweb_pub.publish(2.4)  # Left web joint to -0.6 rad
    rightweb_pub.publish(0.4)  # Right web joint to 0.4 rad
    castorjoint_pub.publish(0.4)  # Castor joint to 0.4 rad

    # Wait for the steering joints to reach their positions
    rospy.sleep(5)  # Adjust this delay as needed

    # Move the robot forward by setting drive joint velocities
    forward_velocity = -(18.0)  # Velocity in radians per second
    rospy.loginfo("Moving robot backward...")

    while not rospy.is_shutdown():
        # Set drive joint velocities
        leftwheel_pub.publish(forward_velocity)
        rightwheel_pub.publish(-(forward_velocity))
        castorwheel_pub.publish(forward_velocity)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        adjust_and_move()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program terminated.")
