#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

class CurvatureCalculator:
    def __init__(self):
        rospy.init_node('curvature_calculator_live')

        self.path_np = None  # Nx2 array of [x, y]
        self.pose = [0.0, 0.0, 0.0]  # x, y, yaw
        self.last_target_index = 0

        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.pub = rospy.Publisher('/path_curvature', Float32MultiArray, queue_size=10)

        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.loginfo("Live curvature calculator running.")

    def path_callback(self, msg):
        self.path_np = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        self.last_target_index = 0

    def odom_callback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.pose[2] = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def timer_callback(self, event):
        if self.path_np is None or len(self.path_np) < 3:
            return

        idx = self.get_target_index(self.pose)
        # Try to form a triplet (p0, p1, p2) around idx
        if idx < 1:
            idx = 1
        if idx > len(self.path_np) - 2:
            idx = len(self.path_np) - 2

        p0 = self.path_np[idx - 1]
        p1 = self.path_np[idx]
        p2 = self.path_np[idx + 1]

        curvature = self.calculate_curvature(p0, p1, p2)
        self.pub.publish(Float32MultiArray(data=[curvature]))

    def get_target_index(self, pose):
        pos = np.array(pose[:2])
        slice_ = self.path_np[self.last_target_index:]
        dists = np.linalg.norm(slice_ - pos, axis=1)
        i_min = np.argmin(dists)
        global_i = self.last_target_index + i_min
        self.last_target_index = global_i
        return global_i

    def calculate_curvature(self, p0, p1, p2):
        dx1 = p1[0] - p0[0]
        dy1 = p1[1] - p0[1]
        dx2 = p2[0] - p1[0]
        dy2 = p2[1] - p1[1]

        cross = dx1 * dy2 - dy1 * dx2
        mag1_sq = dx1**2 + dy1**2
        mag2_sq = dx2**2 + dy2**2

        if mag1_sq == 0 or mag2_sq == 0:
            return 0.0

        curvature = 2 * cross / (
            math.sqrt(mag1_sq) * math.sqrt(mag2_sq) *
            (math.sqrt(mag1_sq) + math.sqrt(mag2_sq))
        )
        return curvature

if __name__ == '__main__':
    try:
        CurvatureCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
