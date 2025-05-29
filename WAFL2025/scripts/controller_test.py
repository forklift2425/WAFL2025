#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PoseHandler:
    def __init__(self):
        self.pose = [0.0, 0.0, 0.0]
        rospy.Subscriber("/odom", Odometry, self.callback)

    def callback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose[2] = 2 * np.arctan2(q.z, q.w)  # Yaw
        rospy.loginfo_throttle(1.0, f"[PoseHandler] Updated Pose: {self.pose}")

class PathManager:
    def __init__(self, braking_margin=0.3, lookahead=1.0):
        self.path = []
        self.braking_margin = braking_margin
        self.lookahead = lookahead
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.callback)

    def callback(self, msg):
        self.path = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        rospy.loginfo_throttle(1.0, f"[PathManager] Received new path with {len(self.path)} points")

    def get_target_index(self, pose):
        if not self.path:
            return 0, True
        dists = np.linalg.norm(np.array(self.path) - np.array(pose[:2]), axis=1)
        min_idx = np.argmin(dists)
        braking = dists[-1] < self.braking_margin
        for i in range(min_idx + 1, len(dists)):
            if dists[i] > self.lookahead:
                return i, braking
        return min_idx, braking

class ControlModeSelector:
    def __init__(self):
        self.mode = 0
        rospy.Subscriber("/path_curvature", Float32MultiArray, self.callback)

    def callback(self, msg):
        if not msg.data:
            return
        k = abs(float(msg.data[0]))
        if k > 2.1:
            self.mode = 1
        elif 0.2 <= k <= 2.1:
            self.mode = 2
        else:
            self.mode = 3
        rospy.loginfo_throttle(1.0, f"[ControlModeSelector] Curvature: {k:.3f}, Selected Mode: {self.mode}")

class SteeringController:
    def __init__(self, wheelbase=0.5, v_setpoint=1.0):
        self.L = wheelbase
        self.v = v_setpoint

    def compute(self, pose, target, mode, braking, lookahead):
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        alpha = np.arctan2(dy, dx) - pose[2]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Normalize
        steer = np.arctan2(2 * self.L * np.sin(alpha), lookahead)

        rospy.loginfo_throttle(
            1.0,
            f"[SteeringController DEBUG] dx: {dx:.3f}, dy: {dy:.3f}, "
            f"alpha: {alpha:.3f}, steer: {steer:.3f}"
        )

        if mode == 1:
            front, rear = steer, 0.0
        elif mode == 2:
            front = steer
            rear = -0.5 * front
        elif mode == 3:
            front = rear = alpha

        if braking:
            rospy.loginfo_throttle(1.0, "[SteeringController] Braking active")
            return [0.0, 0.0, rear, front]
        else:
            rospy.loginfo_throttle(
                1.0,
                f"[SteeringController] Mode: {mode}, Left/Right: {self.v:.2f}, "
                f"Rear: {rear:.2f}, Front: {front:.2f}"
            )
            return [self.v, self.v, rear, front]

class CommandPublisher:
    def __init__(self):
        self.pub = rospy.Publisher("/swerve/raw_data", Float32MultiArray, queue_size=10)

    def publish(self, command):
        msg = Float32MultiArray()
        msg.data = command
        rospy.loginfo_throttle(1.0, f"[CommandPublisher] Publishing: {command}")
        self.pub.publish(msg)

class ControllerNode:
    def __init__(self):
        rospy.init_node("path_controller_modular")
        self.pose_handler = PoseHandler()
        self.path_manager = PathManager()
        self.mode_selector = ControlModeSelector()
        self.controller = SteeringController()
        self.publisher = CommandPublisher()
        self.lookahead = 1.0
        self.timer = rospy.Timer(rospy.Duration(0.1), self.loop)

    def loop(self, event):
        if not self.path_manager.path:
            rospy.logwarn_throttle(1.0, "[ControllerNode] No path available")
            return

        idx, braking = self.path_manager.get_target_index(self.pose_handler.pose)
        target = self.path_manager.path[idx]
        rospy.loginfo_throttle(1.0, f"[ControllerNode] Target Index: {idx}, Target: {target}, Braking: {braking}")
        cmd = self.controller.compute(
            self.pose_handler.pose,
            target,
            self.mode_selector.mode,
            braking,
            self.lookahead
        )
        self.publisher.publish(cmd)

if __name__ == '__main__':
    try:
        ControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
