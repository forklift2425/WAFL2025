#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int8
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32MultiArray
from move_base_msgs.msg import MoveBaseActionGoal  # Add this import

class SupervisorNode:
    def __init__(self):
        rospy.init_node("supervisor_node", anonymous=True)
        
        # High-level subscriptions
        rospy.Subscriber("/pallet_detection", PoseStamped, self.pallet_callback)
        rospy.Subscriber("/destenation", MoveBaseActionGoal, self.destination_callback)  # Changed to destination
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # Low-level subscriptions
        rospy.Subscriber("/fork_state", Bool, self.fork_state_callback)
        rospy.Subscriber("/swerve/raw_data", Float32MultiArray, self.sensor_callback)
        
        # Publishers
        self.fork_cmd_pub = rospy.Publisher("/fork_command", Bool, queue_size=10)
        self.light_cmd_pub = rospy.Publisher("/light_command", Int8, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)  # New publisher
        
        # State variables
        self.pallet_pose = None
        self.robot_pose = [0.0, 0.0, 0.0]
        self.fork_up = False
        self.current_speed = 0.0
        self.current_direction = 1
        self.front_steer_angle = 0.0
        self.last_light_cmd = -1
        
        # Parameters
        self.pallet_proximity_threshold = 0.5
        self.steering_threshold = np.radians(10)
        self.rate = rospy.Rate(10)
        
    def pallet_callback(self, msg):
        self.pallet_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y,
                                  msg.pose.orientation.z, msg.pose.orientation.w])[2]
        ]
        
    def destination_callback(self, msg):
        """Handle destination goals and remap to move_base_simple/goal"""
        try:
            # Extract position from the destination message
            x = msg.goal.target_pose.pose.position.x
            y = msg.goal.target_pose.pose.position.y
            
            # Create new PoseStamped with zero yaw orientation
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            
            # Set orientation to zero yaw
            q = quaternion_from_euler(0, 0, 0)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            
            self.goal_pub.publish(goal)
            rospy.loginfo(f"Published new goal to move_base_simple/goal: x={x}, y={y}, yaw=0")
            
        except Exception as e:
            rospy.logerr(f"Error processing destination: {str(e)}")
            
    def odom_callback(self, msg):
        self.robot_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        ]
        self.current_speed = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.current_direction = 1 if msg.twist.twist.linear.x >= 0 else -1
        
    def fork_state_callback(self, msg):
        self.fork_up = msg.data
        
    def sensor_callback(self, msg):
        self.front_steer_angle = msg.data[3]
        self.control_lights()
        
    def control_fork(self):
        if self.pallet_pose and self.robot_pose:
            dist_to_pallet = np.hypot(
                self.pallet_pose[0] - self.robot_pose[0],
                self.pallet_pose[1] - self.robot_pose[1]
            )
            if dist_to_pallet < self.pallet_proximity_threshold and not self.fork_up:
                self.fork_cmd_pub.publish(Bool(True))
            elif dist_to_pallet >= self.pallet_proximity_threshold and self.fork_up:
                self.fork_cmd_pub.publish(Bool(False))
                
    def control_lights(self):
        light_cmd = Int8()
        if abs(self.current_speed) < 0.1:
            light_cmd.data = 0
        else:
            if abs(self.front_steer_angle) > self.steering_threshold:
                light_cmd.data = 3 if self.front_steer_angle > 0 else 4
            else:
                light_cmd.data = 1 if self.current_direction > 0 else 5
        
        if light_cmd.data != self.last_light_cmd:
            self.last_light_cmd = light_cmd.data
        self.light_cmd_pub.publish(light_cmd)
        
    def run(self):
        while not rospy.is_shutdown():
            self.control_fork()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = SupervisorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass