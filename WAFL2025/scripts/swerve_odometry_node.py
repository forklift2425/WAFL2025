#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import tf
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler

class SwerveOdometry:
    def __init__(self):
        rospy.init_node('swerve_odometry')
        
        # Robot parameters (adjust to match your robot)
        self.wheel_radius = 0.1     # meters
        self.wheelbase = 0.40       # front-rear wheel distance
        self.track_width = 0.30     # left-right wheel distance
        self.max_steer_angle = math.radians(45)  # 45 degrees
        
        # Subscriber
        rospy.Subscriber("/swerve/raw_data", Float32MultiArray, self.encoder_callback)
        
        # Publishers
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()
        
        # Sensor inputs
        self.front_steer = 0.0
        self.rear_steer = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0

    def encoder_callback(self, msg):
        self.left_speed = msg.data[0]    # Left wheel velocity (RPM)
        self.right_speed = msg.data[1]   # Right wheel velocity (RPM)
        self.front_steer = msg.data[3]   # Front steering angle (rad)
        self.rear_steer = msg.data[2]    # Rear steering angle (rad)

    def update_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        # Apply deadzone to wheel speeds
        if abs(self.left_speed) < 0.7: self.left_speed = 0.0
        if abs(self.right_speed) < 0.7: self.right_speed = 0.0
        
        # Calculate effective wheel speeds
        v_left = self.left_speed * self.wheel_radius * 0.3
        v_right = self.right_speed * self.wheel_radius * 0.3
        
        # Average forward speed
        v_avg = (v_left + v_right) / 2.0
        
        # Steering angles (clamped to max)
        delta_f = math.atan(math.tan(min(max(self.front_steer, -self.max_steer_angle), 
                                    self.max_steer_angle)))
        delta_r = math.atan(math.tan(min(max(self.rear_steer, -self.max_steer_angle), 
                                    self.max_steer_angle)))

        # Determine steering mode
        steering_diff = abs(delta_f - delta_r)
        
        if steering_diff < 0.05:  # Crab mode
            omega = 0.0
            heading = (delta_f + delta_r) / 2.0
        else:  # Active/Pure Pursuit mode
            # Calculate instantaneous curvature
            denominator = math.tan(delta_f) - math.tan(delta_r)
            if abs(denominator) > 0.001:
                R = self.wheelbase / denominator
                omega = v_avg / R if R != 0 else 0.0
            else:  # Straight line
                omega = 0.0
            heading = delta_f

        # Calculate velocity components
        dx = v_avg * math.cos(self.theta + heading)
        dy = v_avg * math.sin(self.theta + heading)
        dtheta = -omega

        # Update pose
        self.x += dx * dt
        self.y += dy * dt
        self.theta += dtheta * dt * 0.3
        
        # Normalize orientation
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry
        self.publish_odom(current_time)

    def publish_odom(self, current_time):
        # TF Broadcast
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
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.update_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SwerveOdometry()
        node.run()
    except rospy.ROSInterruptException:
        pass