#!/usr/bin/env python3

import numpy as np
from numpy import array, pi
import rospy
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8 , Float64
from std_msgs.msg import Float32MultiArray

class Controller():
    def __init__(self):

        rospy.Subscriber("/path_curvature", Float32MultiArray, self.curvature_callback)

        self.dt = 0.1
        self.setpointVelocity = 1
        self.currentVelocity = 0.0
        self.lookaheadDistance = 0.8
        self.brakingMargin = 0.3

        self.pose= [0,0,0]
        self.my_pos_x = 0
        self.my_pos_y = 0
        self.my_orien = 0
        self.my_wheel_heading = 0

        self.control_mode=0         # Mode 1 : Ackreman mode    Mode 2 : AFRS       Mode 3 : CRAB

        
    def curvature_callback(self, msg):
        self.curvatures = abs(msg.data)
        if self.curvatures > 2.1 :
            self.control_mode=1
        elif self.curvatures < 2.1 and  self.curvatures > 0.2 :
            self.control_mode=2

        elif self.curvatures < 0.2 :
            self.control_mode=3
        else:
            self.control_mode=0
            
        rospy.loginfo("my current mode is : %f",self.control_mode)



    def purePursuit(self):
        
        ###              Implement pure pursuit controller here               ###
        #            Take as an input the current state of the car,             #
        #                     waypoints & targetIndex                           #
        #                      Return steering angle                            #
        #       which we will use at Ackreman Steering Mode & AFRS Mode         #
        if self.stop or self.brake:
            return 0.0
        
        self.targetIndex = self.searchTargetPoint()
        targetPoint = self.path[self.targetIndex]


        myx = self.my_pos_x
        myy = self.my_pos_y
        theta = self.my_orien
        target_x = targetPoint[0]
        target_y = targetPoint[1]


        alpha = np.arctan2(target_y - myy, target_x - myx) - theta
        steering_angle = np.arctan2(2*self.L*np.sin(alpha) , self.lookaheadDistance)

        #action = self.PID(self.steer_PID_Selector,self.kp_steer, self.ki_steer, self.kd_steer, steering_angle, self.my_wheel_heading, clip = self.angleLimit)

    def PID(self,selector ,kp, ki, kd, setpoint, feedback, **kwargs):
        error = setpoint - feedback

        error_dot = (error - self.error_previous)/self.dt


        self.error_previous = error

        if selector == self.steer_PID_Selector:
            self.error_integral_steer += error*self.dt
            action = kp*error + ki*self.error_integral_steer + kd*error_dot

        else:
            self.error_integral_drive += error*self.dt
            action = kp*error + ki*self.error_integral_drive + kd*error_dot

        if abs(error) > (2*pi/180) or selector == self.drive_PID_Selector :
            return action
        else:
            self.error_integral_steer = 0
            self.error_integral_drive = 0
            return 0    
        
    def searchTargetPoint(self):
        ###           Search for target point in the given path               ###
        #       Take as an input the current state of the car & waypoints       #
        #                   Return index of target point                        #

        distances = np.linalg.norm(array(self.path[self.targetIndex:]) - array([self.my_pos_x, self.my_pos_y]).transpose(), axis=1)
        min_index = np.argmin(distances)

        if distances[-1] < self.brakingMargin:
            self.brake = True
        else:
            self.brake = False

        for i in range(min_index,len(distances)):
            if distances[i] >= self.lookaheadDistance:
                return i + self.targetIndex
        return min_index + self.targetIndex
    


if __name__ == '__main__':
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass