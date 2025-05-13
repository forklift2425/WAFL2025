#!/usr/bin/env python3

import numpy as np
from numpy import array, pi
import rospy
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8, Float64
from std_msgs.msg import Float32MultiArray

class Controller():
    def __init__(self):
        rospy.init_node('path_controller')
        
        # Subscribers
        rospy.Subscriber("/path_curvature", Float32MultiArray, self.curvature_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        
        # Publisher for control commands
        self.control_pub = rospy.Publisher("/swerve/raw_data", Float32MultiArray, queue_size=10)
        
        self.dt = 0.1
        self.setpointVelocity = 1.0  # m/s or PWM value
        self.currentVelocity = 0.0
        self.lookaheadDistance = 0.8
        self.brakingMargin = 0.3
        self.L = 0.5  # Wheelbase (adjust based on your vehicle)
        
        # Vehicle state
        self.pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.path = []
        self.targetIndex = 0
        self.brake = False
        
        # Control mode (1: Front, 2: AFRS, 3: Crab)
        self.control_mode = 0
        
        # Timer for control loop
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_callback)

    def odom_callback(self, msg):
        # Update current pose
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.pose[2] = 2 * np.arctan2(orientation.z, orientation.w)  # Simplified yaw

    def path_callback(self, msg):
        # Convert Path message to list of [x, y] points
        self.path = [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses]

    def curvature_callback(self, msg):
        if len(msg.data) == 0:
            rospy.logwarn("Empty curvature data received")
            return
        
        # Get the first curvature value (or average if multiple)
        self.curvatures = abs(float(msg.data[0]))  # Convert to float first
        
        if self.curvatures > 2.1:
            self.control_mode = 1
        elif 0.2 <= self.curvatures <= 2.1:
            self.control_mode = 2
        elif self.curvatures < 0.2:
            self.control_mode = 3
        else:
            self.control_mode = 0
        rospy.loginfo(f"Control Mode: {self.control_mode}, Curvature: {self.curvatures:.3f}")

    def searchTargetPoint(self):
        if not self.path:
            return 0
        distances = np.linalg.norm(np.array(self.path) - np.array(self.pose[:2]), axis=1)
        min_index = np.argmin(distances)
        self.brake = distances[-1] < self.brakingMargin
        for i in range(min_index, len(distances)):
            if distances[i] >= self.lookaheadDistance:
                return i
        return min_index

    def control_callback(self, event):
        if not self.path:
            return

        self.targetIndex = self.searchTargetPoint()
        target_point = self.path[self.targetIndex]
        dx = target_point[0] - self.pose[0]
        dy = target_point[1] - self.pose[1]
        alpha = np.arctan2(dy, dx) - self.pose[2]
        steering_angle = np.arctan2(2 * self.L * np.sin(alpha), self.lookaheadDistance)

        # Determine steering angles based on mode
        if self.control_mode == 1:
            front_angle = steering_angle
            rear_angle = 0.0
        elif self.control_mode == 2:
            front_angle = steering_angle
            rear_angle = -0.5 * front_angle  # Adjust ratio as needed
        elif self.control_mode == 3:
            front_angle = alpha  # Align wheels with target direction
            rear_angle = alpha
        else:
            front_angle = 0.0
            rear_angle = 0.0

        # Determine wheel speeds
        if self.brake:
            left_speed = 0.0
            right_speed = 0.0
        else:
            left_speed = self.setpointVelocity
            right_speed = self.setpointVelocity

        # Publish control commands
        raw_msg = Float32MultiArray()
        raw_msg.data = [left_speed, right_speed, rear_angle, front_angle]
        self.control_pub.publish(raw_msg)

if __name__ == '__main__':
    try:
        Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass