#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion

####################################
# Handles subscription to /odom and stores current robot pose
####################################
class PoseHandler:
    def __init__(self):
        self.pose = [0.0, 0.0, 0.0]  # x, y, yaw
        rospy.Subscriber("/odom", Odometry, self.callback)

    def callback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self.pose[2] = yaw

####################################
# Manages path updates and target point selection
####################################
class PathManager:
    def __init__(self, braking_margin=0.3, lookahead=1.0):
        self.raw_path = []   # Full path: [[x, y, yaw], ...]
        self.path_np = None  # Numpy array of just x,y for fast lookup
        self.braking_margin = braking_margin
        self.lookahead = lookahead
        self.last_target_index = 0
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.callback)

    def callback(self, msg):
        self.raw_path = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            q = p.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            _, _, yaw = euler_from_quaternion(quat)
            self.raw_path.append([x, y, yaw])
        if self.raw_path:
            self.path_np = np.array([[p[0], p[1]] for p in self.raw_path])
        else:
            self.path_np = None
        self.last_target_index = 0
        rospy.loginfo(f"[PathManager] Received new path with {len(self.raw_path)} points.")

    def get_target_index(self, pose):
        if self.path_np is None or len(self.path_np) == 0:
            return 0, True

        current_position = np.array(pose[:2])
        path_slice = self.path_np[self.last_target_index:]
        dists = np.linalg.norm(path_slice - current_position, axis=1)

        min_idx = np.argmin(dists)
        global_min_idx = self.last_target_index + min_idx
        braking = np.linalg.norm(self.path_np[-1] - current_position) < self.braking_margin

        for i in range(min_idx, len(dists)):
            if dists[i] >= self.lookahead:
                self.last_target_index = global_min_idx + i - min_idx
                return self.last_target_index, braking

        self.last_target_index = global_min_idx
        return global_min_idx, braking

####################################
# Selects control mode based on curvature
####################################
class ControlModeSelector:
    def __init__(self):
        self.mode = 0
        rospy.Subscriber("/path_curvature", Float32MultiArray, self.callback)

    def callback(self, msg):
        if not msg.data:
            return
        k = abs(float(msg.data[0]))
        if k > 2.1:
            self.mode = 1  # Front steer
        elif 0.2 <= k <= 2.1:
            self.mode = 2  # Counter steer
        else:
            self.mode = 3  # Crab

####################################
# PID controller for velocity control (closed loop)
####################################
class SpeedPID:
    def __init__(self, kp=2.0, ki=0.0, kd=0.5, max_output=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0
        self.prev_error = 0
        self.prev_time = None

    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.prev_time = None

    def compute(self, v_target, v_actual):
        error = v_target - v_actual
        now = rospy.get_time()
        dt = 0.1 if self.prev_time is None else now - self.prev_time
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        self.prev_time = now
        output = np.clip(output, -self.max_output, self.max_output)
        rospy.loginfo_throttle(1.0, f"[SpeedPID] error={error:.2f}, output={output:.2f}")
        return output

####################################
# Computes steering angles and velocity based on pose, target, and mode
####################################
class SteeringController:
    def __init__(self, wheelbase=0.4, base_speed=1.0):
        self.L = wheelbase
        self.v_setpoint = base_speed
        self.speed_pid = SpeedPID()
        self.last_pose = None
        self.heading_error_threshold = np.radians(10)  # New threshold for small heading errors
        self.alpha_threshold = np.radians(95)  # Threshold for alpha angle

    def estimate_velocity(self, pose):
        if self.last_pose is None:
            self.last_pose = (pose, rospy.get_time())
            return 0.0
        prev_pose, prev_time = self.last_pose
        dt = rospy.get_time() - prev_time
        if dt == 0:
            return 0.0
        dx = pose[0] - prev_pose[0]
        dy = pose[1] - prev_pose[1]
        v = np.hypot(dx, dy) / dt
        self.last_pose = (pose, rospy.get_time())
        return v


    def compute(self, pose, target_pose, mode, braking, lookahead):
        dx = target_pose[0] - pose[0]
        dy = target_pose[1] - pose[1]
        target_angle = np.arctan2(dy, dx)

        heading_error = target_pose[2] - pose[2]
        rospy.loginfo_throttle(1.0,f"Target Angle {np.degrees(target_pose[2])}, Robot Pose={np.degrees(pose[2])}, ")
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        alpha = target_angle - pose[2]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        steer = np.arctan2(2 * self.L * np.sin(alpha), lookahead)
        steer = - steer  # sign flip as per your original code
        steer = np.clip(steer, -np.radians(90), np.radians(90))
        
        # Modified direction logic
        if mode == 3:
            # In crab mode, prefer forward motion unless heading error is large
            if abs(heading_error) < self.heading_error_threshold:
                direction = 1  # Always forward for small heading errors
            else:
                direction = 1 if abs(alpha) < self.alpha_threshold else -1
        else:
            direction = 1 if abs(alpha) < np.radians(115) else -1

        v_measured = self.estimate_velocity(pose)
        v_control = self.speed_pid.compute(self.v_setpoint * direction, v_measured)

        if mode == 1:
            front, rear = steer, 0.0
        elif mode == 2:
            front = 1.5*steer
            rear = -1.5/0.75 * front
        else:
            front = rear = - steer

        rospy.loginfo_throttle(1.0, f"Alpha: {np.degrees(alpha):.2f} deg, Heading Error: {np.degrees(heading_error):.2f} deg")

        if braking:
            rospy.loginfo_throttle(1.0, "Braking!")
            return [0.0, 0.0, rear, front]
        else:
            rospy.loginfo_throttle(
                1.0,
                f"Mode {mode}, v_cmd={v_control:.2f}, "
                f"rear={rear:.2f}, front={front:.2f}, direction={'F' if direction == 1 else 'B'}"
            )
            return [v_control, v_control, rear, front]

####################################
# Publishes computed command to /swerve/raw_data
####################################
class CommandPublisher:
    def __init__(self):
        self.pub = rospy.Publisher("/swerve/raw_data", Float32MultiArray, queue_size=10)

    def publish(self, command):
        msg = Float32MultiArray()
        msg.data = command
        self.pub.publish(msg)

####################################
# Main node that coordinates everything
####################################
class ControllerNode:
    def __init__(self):
        rospy.init_node("path_controller_pid")
        self.pose_handler = PoseHandler()
        self.path_manager = PathManager()
        self.mode_selector = ControlModeSelector()
        self.controller = SteeringController()
        self.publisher = CommandPublisher()
        self.lookahead = 1.0
        self.timer = rospy.Timer(rospy.Duration(0.1), self.loop)

    def loop(self, event):
        if not self.path_manager.raw_path:
            rospy.logwarn_throttle(1.0, "[ControllerNode] No path to follow.")
            return

        idx, braking = self.path_manager.get_target_index(self.pose_handler.pose)
        target = self.path_manager.raw_path[idx]
        if len(target) < 3:
            rospy.logwarn_throttle(1.0, "[ControllerNode] Target pose missing yaw, skipping.")
            return

        rospy.loginfo_throttle(1.0, f"[ControllerNode] Target idx: {idx}, Braking: {braking}")
        cmd = self.controller.compute(
            self.pose_handler.pose,
            target,
            self.mode_selector.mode,
            braking,
            self.lookahead
        )
        self.publisher.publish(cmd)

####################################
# Entry point
####################################
if __name__ == '__main__':
    try:
        ControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
