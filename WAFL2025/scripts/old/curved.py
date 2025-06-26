#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float64

class SwerveController:
    def __init__(self):
        self.wheelbase = 1.2  # Distance between front and rear axles
        self.track_width = 0.5  # Distance between front wheels
        
        self.steer_pubs = {
            'left': rospy.Publisher('/leftweb_steer_controller/command', Float64, queue_size=1),
            'right': rospy.Publisher('/rightweb_steer_controller/command', Float64, queue_size=1),
            'rear': rospy.Publisher('/castorjoint_steer_controller/command', Float64, queue_size=1)
        }
        
        self.drive_pubs = {
            'left': rospy.Publisher('/leftwheel_drive_controller/command', Float64, queue_size=1),
            'right': rospy.Publisher('/rightwheel_drive_controller/command', Float64, queue_size=1)
        }

    def ackermann_steering(self, speed, steering_angle):
        """Convert steering angle to individual wheel angles"""
        if steering_angle == 0:
            return {'left': 0.0, 'right': 0.0, 'rear': 0.0}
            
        radius = self.wheelbase / math.tan(steering_angle)
        return {
            'left': math.atan(self.wheelbase / (radius - self.track_width/2)),
            'right': math.atan(self.wheelbase / (radius + self.track_width/2)),
            'rear': -math.atan(self.wheelbase / radius)
        }

    def move(self, speed, steering_angle, duration):
        angles = self.ackermann_steering(speed, steering_angle)
        
        # Publish steering commands
        for joint, angle in angles.items():
            self.steer_pubs[joint].publish(Float64(angle))
        
        # Publish drive commands
        self.drive_pubs['left'].publish(Float64(speed))
        self.drive_pubs['right'].publish(Float64(-speed))
        
        rospy.sleep(duration)
        self.stop()

    def stop(self):
        for pub in self.drive_pubs.values():
            pub.publish(Float64(0.0))

if __name__ == '__main__':
    try:
        rospy.init_node('swerve_controller')
        controller = SwerveController()
        
        # Example maneuvers
        controller.move(2.0, math.radians(30), 5.0)  # Curved motion
        controller.move(1.5, math.radians(-45), 3.0)
        controller.move(2.5, 0.0, 4.0)  # Straight line
        
    except rospy.ROSInterruptException:
        pass