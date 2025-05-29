import rospy
from std_msgs.msg import Float32MultiArray, Float32

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = rospy.get_time()

    def update(self, current_value):
        error = self.setpoint - current_value
        current_time = rospy.get_time()
        dt = current_time - self.last_time if self.last_time is not None else 0.0

        P_out = self.Kp * error
        self.integral += error * dt
        I_out = self.Ki * self.integral
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        D_out = self.Kd * derivative

        output = P_out + I_out + D_out

        self.prev_error = error
        self.last_time = current_time

        return output

class SwerveMotorController:
    def __init__(self):
        rospy.init_node('swerve_motor_controller')

        self.motor_pub = rospy.Publisher('/motor_speed', Float32MultiArray, queue_size=10)
        self.front_pub = rospy.Publisher('/steering/front', Float32, queue_size=10)
        self.rear_pub  = rospy.Publisher('/steering/rear',  Float32, queue_size=10)
        self.left_pub  = rospy.Publisher('/wheel_speed_left',  Float32, queue_size=10)
        self.right_pub = rospy.Publisher('/wheel_speed_right', Float32, queue_size=10)

        rospy.Subscriber('/swerve/raw_data', Float32MultiArray, self.control_callback)

        self.Kp = rospy.get_param('~Kp', 1.0)
        self.Ki = rospy.get_param('~Ki', 0.0)
        self.Kd = rospy.get_param('~Kd', 0.0)
        self.pid = PIDController(self.Kp, self.Ki, self.Kd)

    def control_callback(self, msg):
        if len(msg.data) < 4:
            rospy.logwarn("Invalid swerve/raw_data message")
            return

        vL = msg.data[0]
        vR = msg.data[1]
        rear_steer  = msg.data[2]
        front_steer = msg.data[3]

        velocity_target = (vL + vR) / 2.0
        direction = 1 if velocity_target >= 0 else 0
        speed = abs(velocity_target)

        self.pid.setpoint = speed
        effort = self.pid.update(current_value=speed)

        pwm = int(abs(effort))
        pwm = max(min(pwm, 255), 0)

        self.motor_pub.publish(Float32MultiArray(data=[float(pwm), float(direction)]))
        self.front_pub.publish(float(front_steer))
        self.rear_pub.publish(float(rear_steer))
        self.left_pub.publish(float(vL))
        self.right_pub.publish(float(vR))

if __name__ == '__main__':
    try:
        controller = SwerveMotorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
