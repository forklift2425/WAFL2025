#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
import tf_conversions

class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_publisher')
        self.map = None

        # Subscribers & Publishers
        rospy.Subscriber('/map', OccupancyGrid, self.map_cb, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',
                                        PoseStamped,
                                        queue_size=1)

        # Parameters (you can override on the command line)
        self.x = rospy.get_param('~x', 1.0)
        self.y = rospy.get_param('~y', 2.0)
        self.yaw = rospy.get_param('~yaw', 0.0)  # in radians
        self.wait_map_sec = rospy.get_param('~wait_map_sec', 1.0)

    def map_cb(self, msg):
        """Save latest map for validity checks."""
        self.map = msg

    def is_free(self, x, y):
        """Return True if the (x,y) lies in a free cell of the occupancy grid."""
        if self.map is None:
            return False

        mi = self.map.info
        # Convert world coords to map indices
        ix = int((x - mi.origin.position.x) / mi.resolution)
        iy = int((y - mi.origin.position.y) / mi.resolution)

        # Out of bounds = invalid
        if ix < 0 or iy < 0 or ix >= mi.width or iy >= mi.height:
            return False

        idx = iy * mi.width + ix
        val = self.map.data[idx]
        # 0 = free, 100 = occupied, -1 = unknown
        return (val == 0)

    def send_goal(self, x, y, yaw):
        """Check validity, then publish a PoseStamped goal."""
        if self.is_free(x, y):
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'map'

            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0

            # Convert yaw to quaternion
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
            goal.pose.orientation = Quaternion(*q)

            self.goal_pub.publish(goal)
            rospy.loginfo(f"[GoalPublisher] Published valid goal at ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)")
        else:
            rospy.logwarn(f"[GoalPublisher] Goal ({x:.2f}, {y:.2f}) is invalid (occupied or out of bounds).")

    def run(self):
        # Give the map topic a chance to publish at least once
        rospy.loginfo(f"[GoalPublisher] Waiting up to {self.wait_map_sec}s for map...")
        rospy.sleep(self.wait_map_sec)

        self.send_goal(self.x, self.y, self.yaw)
        rospy.spin()


if __name__ == '__main__':
    try:
        gp = GoalPublisher()
        gp.run()
    except rospy.ROSInterruptException:
        pass