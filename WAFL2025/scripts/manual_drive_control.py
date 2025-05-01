#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from WAFL2025.msg import WheelSpeeds, SteeringAngles  # Custom messages
from std_msgs.msg import Float32MultiArray

class ManualDriveController:
    def __init__(self):
        rospy.init_node('swerve_drive_control')

        # Subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        # Publishers
        self.wheel_speeds_pub = rospy.Publisher('/wheel_speeds', WheelSpeeds, queue_size=10)
        self.steering_angles_pub = rospy.Publisher('/steering_angles', SteeringAngles, queue_size=10)
       # self.actions_pub=rospy.Publisher("/swerve/raw_data", Float32MultiArray,queue_size=10)

        self.speeds = WheelSpeeds()
        self.angles = SteeringAngles()
        self.avgPWM=100
        self.RS=0
        self.LS=0
        
    def cmd_vel_callback(self, msg):
        speed = msg.linear.x
        if msg.linear.x > 0 or msg.linear.x < 0:
            self.speeds.left_speed  = self.avgPWM * speed
            self.speeds.right_speed = self.avgPWM * speed
            self.RS=speed
            self.LS=speed
            

        elif msg.angular.z > 0:
            self.speeds.left_speed  = 0
            self.speeds.right_speed = 0

            self.angles.front_angle +=0.2
            self.angles.rear_angle = 0

            self.RS=speed
            self.LS=speed
            

        elif msg.angular.z < 0:
            self.speeds.left_speed  = 0
            self.speeds.right_speed = 0

            self.angles.front_angle -=0.2
            self.angles.rear_angle = 0

            self.RS=speed
            self.LS=speed

        elif msg.linear.y > 0:
            self.angles.front_angle = 1.57
            self.angles.rear_angle = 1.57

        elif msg.linear.y < 0:
            self.angles.front_angle = -1.57
            self.angles.rear_angle = -1.57

        elif msg.linear.z == 1.0:
            self.angles.front_angle = 0
            self.angles.rear_angle = 0

        elif msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0:
            self.speeds.left_speed  = 0
            self.speeds.right_speed = 0

            self.angles.front_angle = self.angles.front_angle
            self.angles.rear_angle = self.angles.rear_angle 

            self.RS=speed
            self.LS=speed

        # publish
        self.wheel_speeds_pub.publish(self.speeds)
        self.steering_angles_pub.publish(self.angles)

        # Publish raw data as array [FL speed, FR speed, front steer, rear steer]
        raw_msg = Float32MultiArray()
        raw_msg.data = [
            self.LS,
            self.RS,
            self.angles.rear_angle,
            self.angles.front_angle]
        
      #  self.actions_pub.publish(raw_msg)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ManualDriveController()
        node.run()
    except rospy.ROSInterruptException:
        pass