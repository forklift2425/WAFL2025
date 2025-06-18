#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion
from WAFL2025.msg import WheelSpeeds, SteeringAngles
from geometry_msgs.msg import PoseWithCovarianceStamped

####################################
# Handles subscription to /odom and stores current robot pose
####################################
class PoseHandler:
    def __init__(self):
        self.pose = [0.0, 0.0, 0.0]  # [x, y, yaw]
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback)

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
        self.path_np = None  # Nx2 array of just [x,y]
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
            self.path_np = np.array([[pt[0], pt[1]] for pt in self.raw_path])
        else:
            self.path_np = None

        self.last_target_index = 0

    def get_target_index(self, pose):
        """
        Returns (index, braking_flag).
        - index: index into raw_path for the current lookahead target.
        - braking_flag: True if distance to final path point < braking_margin.
        """
        if self.path_np is None or len(self.path_np) == 0:
            return 0, True

        current = np.array(pose[:2])
        path_slice = self.path_np[self.last_target_index :]
        dists = np.linalg.norm(path_slice - current, axis=1)

        min_idx = np.argmin(dists)
        global_min_idx = self.last_target_index + min_idx
        braking = np.linalg.norm(self.path_np[-1] - current) < self.braking_margin

        for i in range(min_idx, len(dists)):
            if dists[i] >= self.lookahead:
                self.last_target_index = global_min_idx + (i - min_idx)
                return self.last_target_index, braking

        # If no lookahead distance found, just use the closest point
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
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, v_target, v_actual):
        """
        Standard PID: output = kp*e + ki*∫e + kd*(de/dt), clipped to ±max_output.
        """
        error = v_target - v_actual
        now = rospy.get_time()
        dt = 0.1 if self.prev_time is None else (now - self.prev_time)
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        self.prev_time = now
        output = np.clip(output, -self.max_output, self.max_output)

        rospy.loginfo_throttle(1.0, f"[SpeedPID] error={error:.2f}, output={output:.2f}")
        return output

####################################
# Computes steering angles and velocities based on pose, target, and mode
####################################
class SteeringController:
    def __init__(self, wheelbase=0.4, base_speed=1.0):
        self.L = wheelbase
        self.v_setpoint = base_speed
        self.speed_pid = SpeedPID()

        # Hysteresis thresholds for crab mode (radians)
        self.alpha_forward_to_backward = np.radians(110)   # only reverse once |α| > 110°
        self.alpha_backward_to_forward = np.radians(70)    # only go forward once |α| < 70°
        self.heading_error_threshold = np.radians(10)      # if heading_error < 10°, force forward

        # Pose‐goal tolerances (bigger to account for path shift during spin)
        self.position_tolerance = 0.8        # [m]
        self.orientation_tolerance = np.radians(5)  # [rad]

        # Steering angles for “approximate in‐place spin” (Active Front+Rear)
        self.spin_steer_angle = np.radians(45)   # ±45° works for many 4WS platforms

        # State‐holders
        self.last_pose = None
        self.last_direction = +1     # assume we start moving forward
        self.goal_pose = None        # [x_g, y_g, θ_g], set externally by ControllerNode

    def estimate_velocity(self, pose):
        """
        Estimate the (linear) speed of the robot by finite difference.
        """
        if self.last_pose is None:
            self.last_pose = (pose, rospy.get_time())
            return 0.0

        prev_pose, prev_time = self.last_pose
        now = rospy.get_time()
        dt = now - prev_time
        if dt <= 0.0:
            return 0.0

        dx = pose[0] - prev_pose[0]
        dy = pose[1] - prev_pose[1]
        v = np.hypot(dx, dy) / dt

        self.last_pose = (pose, now)
        return v

    def compute(self, pose, target_pose, mode, braking, lookahead):
        """
        pose         : [x_r, y_r, θ_r]
        target_pose  : [x_t, y_t, θ_t]   # current lookahead point
        self.goal_pose: [x_g, y_g, θ_g]  # final path point

        Returns: [v_left, v_right, steer_rear, steer_front]
        """
        x_r, y_r, theta_r = pose
        x_t, y_t, theta_t = target_pose
        
        # ------------------------------------------------------------
        # 1) “Rotate‐in‐place to fix orientation” if already close to (x_g, y_g)
        if self.goal_pose is not None:
            x_g, y_g, theta_g = self.goal_pose
            dxg = x_g - x_r
            dyg = y_g - y_r
            dist_to_goal = np.hypot(dxg, dyg)
            heading_error_goal = np.arctan2(np.sin(theta_g - theta_r),
                                            np.cos(theta_g - theta_r))
             #heading_error_goal = -heading_error_goal
            '''
            # Log distance and heading error to goal
            rospy.loginfo_throttle(
                1.0,
                f"[Debug] dist_to_goal={dist_to_goal:.3f}, heading_error_goal={np.degrees(heading_error_goal):.2f}°"
            )
            '''
            if dist_to_goal < self.position_tolerance:
                # We’re essentially at the final (x, y). Now check θ.
                if abs(heading_error_goal) < self.orientation_tolerance:
                    # Instead of stopping, exit pose control and allow other modes to run.
                    rospy.loginfo_throttle(1.0, "[Debug] Pose within tolerance, continuing normal control.")
                else:
                    # Approximate in‐place spin using Active Front+Rear Steering.
                    turn_dir = 1 if heading_error_goal > 0 else -1
                    steer_front = turn_dir * self.spin_steer_angle
                    steer_rear  = -turn_dir * self.spin_steer_angle

                    # Choose a modest spin speed (m/s) proportional to heading_error_goal
                    K_rot = 10
                    v_rot = np.clip(K_rot * heading_error_goal, -2, +2)

                    v_left  = +v_rot * turn_dir   # left wheels forward/back
                    v_right = +v_rot * turn_dir   # right wheels same direction to pivot

                    rospy.loginfo_throttle(
                        1.0,
                        f"steer_rear={np.degrees(steer_rear):.1f}°, steer_front={np.degrees(steer_front):.1f}°")
                    return [v_left, v_right, steer_rear, steer_front]
        else:
            rospy.logwarn_throttle(1.0, "No goal pose set! Skipping spin check")

        # ------------------------------------------------------------
        # 2) Otherwise, “not at goal yet” → do Pure Pursuit + multi‐mode logic

        #  2a) Compute Pure Pursuit geometry:
        dx = x_t - x_r
        dy = y_t - y_r
        target_angle = np.arctan2(dy, dx)
        alpha = np.arctan2(np.sin(target_angle - theta_r),
                           np.cos(target_angle - theta_r))

        #  2b) Heading‐error toward the path segment’s yaw (for logging/hysteresis)
        raw_heading_error = np.arctan2(np.sin(theta_t - theta_r),
                                       np.cos(theta_t - theta_r))

        # Log alpha and heading_error for debugging
        rospy.loginfo_throttle(
            1.0,
            f"[Debug] alpha={np.degrees(alpha):.2f}°"
        )

        #  2c) Pure Pursuit steering angle
        steer = np.arctan2(self.L * np.sin(alpha), lookahead)
        steer = -steer  # flipped sign as robot’s kinematics expect it
        steer = np.clip(steer, -np.radians(90), +np.radians(90))

        #  2d) Crab‐mode direction with hysteresis
        if mode == 3:
            abs_alpha = abs(alpha)
            direction = self.last_direction

            if self.last_direction > 0:
              # Currently forward → only reverse if α > alpha_forward_to_backward
               if abs_alpha > self.alpha_forward_to_backward:
                    direction = -1
               else:
                  # Currently backward → only go forward if α < alpha_backward_to_forward
                  if abs_alpha < self.alpha_backward_to_forward:
                        direction = +1

            # Log hysteresis decision
            # rospy.loginfo_throttle(
            #     1.0,
            #     f"[Debug-Crab] last_direction={self.last_direction}, abs_alpha={np.degrees(abs_alpha):.2f}°, "
            #     f"alpha_fwd2bck={np.degrees(self.alpha_forward_to_backward):.1f}°, "
            #     f"alpha_bck2fwd={np.degrees(self.alpha_backward_to_forward):.1f}°, direction={direction}"
            # )

            self.last_direction = direction

        else:
            # Modes 1/2 (front‐steer / counter‐steer) use the old “flip at 115°” logic:
            if abs(alpha) < np.radians(115):
                direction = +1
            else:
                direction = -1
            self.last_direction = direction

        #  2e) Closed‐loop velocity (both wheels same speed in normal driving)
        v_measured = self.estimate_velocity(pose)
        v_target = self.v_setpoint * direction
        v_control = self.speed_pid.compute(v_target, v_measured)

        #  2f) Convert to wheel‐angles / motor commands
        #      In normal driving, we set v_left = v_right = v_control
        if mode == 1:
            # Front‐steer only
            front = -steer
            rear  = 0.0
        elif mode == 2:
            # Counter‐steer: front = 1.5*steer; rear = –(1.5/0.75)*front
            front = - steer
            rear  = +0.75 * steer
        else:
            # mode == 3 (crab): front = –steer * 1.3, rear = –steer * 1.3
         #   front = -steer * 1.3
         #   rear  = -steer * 1.3
            front = -steer
            rear  = -steer 
        # Log steering angles and velocities
        rospy.loginfo_throttle(
            1.0,
            f"[Debug-Drive] mode={mode}, v_control={v_control:.2f}, steer_rear={np.degrees(rear):.1f}°, "
            f"steer_front={np.degrees(front):.1f}°, direction={'F' if self.last_direction > 0 else 'B'}"
        )

        if braking:
            rospy.loginfo_throttle(1.0, "Braking !!!")
            return [0.0, 0.0, rear, front]
        else:
            return [v_control, v_control, rear, front]

####################################
# Publishes computed command to /swerve/raw_data
####################################
class CommandPublisher:
    def __init__(self):  # FIXED: Changed from init() to __init__()
        self.wheel_speed_pub = rospy.Publisher("/wheel_speeds", WheelSpeeds, queue_size=10)
        self.steering_angle_pub = rospy.Publisher("/steering_angles", SteeringAngles, queue_size=10)

    def publish(self, command):
        """
        Splits command into:
        - wheelspeeds: [v_left, v_right]
        - steeringangles: [steer_rear, steer_front]
        """
        if len(command) != 4:
            rospy.logerr(f"Invalid command length: {len(command)}. Expected 4 elements")
            return
            
        v_left, v_right, steer_rear, steer_front = command
        
        # Publish wheel speeds
        speed_msg = WheelSpeeds()
        speed_msg.left_speed = v_left
        speed_msg.right_speed = v_right
        self.wheel_speed_pub.publish(speed_msg)
        
        # Publish steering angles
        steer_msg = SteeringAngles()
        steer_msg.rear_angle = steer_rear
        steer_msg.front_angle = steer_front
        self.steering_angle_pub.publish(steer_msg)

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
        self.publisher = CommandPublisher()  # Now properly initialized
        self.lookahead = 1.0
        # Loop at 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.loop)

    def loop(self, event):
        if not self.path_manager.raw_path:
            self.controller.goal_pose = None  # Clear goal when no path
            return

        idx, braking = self.path_manager.get_target_index(self.pose_handler.pose)
        target = self.path_manager.raw_path[idx]

        # Provide the final path point to the controller
        self.controller.goal_pose = self.path_manager.raw_path[-1]

        if len(target) < 3:
            rospy.logwarn_throttle(1.0, "[ControllerNode] Target pose missing yaw, skipping.")
            return

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
        node = ControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass