#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import tf
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler

class AckermannOdometry:
    def __init__(self):
        rospy.init_node('swerve_odometry')
        
        self.wheel_radius = 0.1  # meters
        self.wheelbase = 0.50     # distance between front and rear axles
        self.track_width = 0.30   # distance between left/right wheels
        
        # Subscribers
        rospy.Subscriber("/swerve/raw_data", Float32MultiArray, self.encoder_callback) ####
        
        # Publishers
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()
        
        # Current measurements
        self.front_steer = 0.0
        self.rear_steer = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0

    def encoder_callback(self, msg):
        self.left_speed = msg.data[0]	# Left wheel velocity (m/s)
        self.right_speed = msg.data[1]	# Right wheel velocity (m/s)
        self.front_steer = msg.data[3] # Front steering angle (rad)
        self.rear_steer = msg.data[2]	# Rear Potentiometer reading (rad)

    def update_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        # Convert wheel speeds to m/s (adjust for your gear ratio)
        v_left = self.left_speed * self.wheel_radius
        v_right = self.right_speed * self.wheel_radius
        
        # Calculate ICC (Instantaneous Center of Curvature)
        if abs(self.front_steer - self.rear_steer) > 0.1:  # Not pure translation
        #    R = self.wheelbase / (math.tan(self.front_steer) - math.tan(self.rear_steer))
            omega = self.front_steer - self.rear_steer
            angular = omega
            #print("Pure Pursuit Mode")
        else:  #CRAB MODE
        #    R = float('inf')
            #print("CRAB Mode")
            omega = (self.front_steer+self.rear_steer)/2
            angular = 0

        
        # Calculate robot velocities
        linear = (v_left + v_right) / 2.0
        # Update pose (using bicycle model)
        delta_theta = angular * dt
        if abs(delta_theta) < 0.01:  # Straight line motion
            self.x += linear * math.cos(self.theta+omega) * dt
            self.y += linear * math.sin(self.theta+omega) * dt
        else:  # Curved motion
            self.x += linear * (math.sin(self.theta+delta_theta))  *  dt
            self.y += linear * (math.cos(self.theta+delta_theta)) *  dt
        
        if linear >0 :
            self.theta += delta_theta
        else:
            self.theta =self.theta
        
        # Normalize angle
        #self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish TF and Odometry
        self.publish_odom(current_time)

    def publish_odom(self, current_time):
        # TF Broadcast (odom->base_link)
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quaternion_from_euler(0, 0, self.theta),
            current_time,
            "base_link",
            "odom"
        )
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.theta))
        
        self.odom_pub.publish(odom)
        self.last_time = current_time

    def run(self):
        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            self.update_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = AckermannOdometry()
        node.run()
    except rospy.ROSInterruptException:
        pass