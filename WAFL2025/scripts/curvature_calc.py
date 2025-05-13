#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class CurvatureCalculator:
    def __init__(self):
        rospy.init_node('curvature_calculator_py', anonymous=True)
        
        # Subscribe to the local planner's trajectory
        self.sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan', 
            Path, 
            self.path_callback
        )
        
        # Publish curvature values for visualization or control
        self.pub = rospy.Publisher(
            '/path_curvature', 
            Float32MultiArray, 
            queue_size=10
        )
        
        rospy.loginfo("Curvature calculator node started!")

    def path_callback(self, msg):
        curvatures = []
        poses = msg.poses  # List of PoseStamped messages
        
        if len(poses) < 3:
            rospy.logwarn("Not enough points to compute curvature.")
            return
        
        # Iterate through triplets of poses
        for i in range(1, len(poses) - 1):
            p0 = poses[i-1].pose.position
            p1 = poses[i].pose.position
            p2 = poses[i+1].pose.position
            
            curvature = self.calculate_curvature(p0, p1, p2)
            curvatures.append(curvature)
        
        # Publish curvatures
        curvature_msg = Float32MultiArray(data=curvatures)
        self.pub.publish(curvature_msg)

    def calculate_curvature(self, p0, p1, p2):
        # Vectors between points
        dx1 = p1.x - p0.x
        dy1 = p1.y - p0.y
        dx2 = p2.x - p1.x
        dy2 = p2.y - p1.y
        
        # Cross product and magnitudes
        cross = dx1 * dy2 - dy1 * dx2
        mag1_sq = dx1**2 + dy1**2
        mag2_sq = dx2**2 + dy2**2
        
        # Handle edge cases (e.g., straight lines)
        if mag1_sq == 0 or mag2_sq == 0:
            return 0.0
        
        # Curvature formula: 2 * cross / (|v1| * |v2| * (|v1| + |v2|))
        curvature = 2 * cross / (
            math.sqrt(mag1_sq) * math.sqrt(mag2_sq) * 
            (math.sqrt(mag1_sq) + math.sqrt(mag2_sq)) )
        
        rospy.loginfo("Curvature is: %f", curvature) 
        
        return curvature

if __name__ == '__main__':
    try:
        calculator = CurvatureCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass