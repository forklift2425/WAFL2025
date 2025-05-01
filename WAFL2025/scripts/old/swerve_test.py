#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import math
import time

def test_publisher():
    rospy.init_node("swerve_test_publisher")
    pub = rospy.Publisher("/swerve/raw_data", Float32MultiArray, queue_size=10)
    rate = rospy.Rate(20)  # 20 Hz

    # Movement pattern parameters
    move_duration = 1   # seconds for each movement
    straight_speed = 1.0   # m/s
    turn_speed = 0.5       # m/s
    turn_angle = math.pi/4 # 45 degrees for turns

    start_time = time.time()
    phase = 0  # 0=straight, 1=right, 2=straight, 3=left, etc.

    while not rospy.is_shutdown():
        current_time = time.time() - start_time
        cycle_time = current_time % (move_duration * 4)  # 4-phase cycle

        if cycle_time < move_duration:
            # Phase 1: Move straight forward
            left_drive = straight_speed
            right_drive = straight_speed
            left_steer = 0.0
            right_steer = 0.0
            phase = 0
        elif cycle_time < move_duration * 2:
            # Phase 2: Turn right
            left_drive = turn_speed
            right_drive = turn_speed
            left_steer = turn_angle
            right_steer = -turn_angle  # Opposite angle for right turn
            phase = 1
        elif cycle_time < move_duration * 3:
            # Phase 3: Move straight forward
            left_drive = straight_speed
            right_drive = straight_speed
            left_steer = 0.0
            right_steer = 0.0
            phase = 2
        else:
            # Phase 4: Turn left
            left_drive = turn_speed
            right_drive = turn_speed
            left_steer = -turn_angle
            right_steer = turn_angle  # Opposite angle for left turn
            phase = 3

        # Create and publish message
        msg = Float32MultiArray()
        msg.data = [left_drive, left_steer, right_drive, right_steer]
        pub.publish(msg)
        
        # Debug output
        if cycle_time < 0.1:  # Print only when phase changes
            phases = ["STRAIGHT", "RIGHT TURN", "STRAIGHT", "LEFT TURN"]
            rospy.loginfo(f"Current phase: {phases[phase]}")
        
        rate.sleep()

if __name__ == "__main__":
    try:
        test_publisher()
    except rospy.ROSInterruptException:
        pass
