#!/usr/bin/env python3

import rospy
import can
from can_msgs.msg import Frame

def can_receiver():
    # Initialize the ROS node
    rospy.init_node('can_receiver_node', anonymous=True)
    
    # Get the CAN interface parameter (default to vcan0 for debugging)
    can_interface = rospy.get_param('~can_interface', 'vcan0')
    rospy.loginfo("Using CAN interface: %s", can_interface)
    
    # Publisher for CAN messages
    can_pub = rospy.Publisher('can_rx', Frame, queue_size=10)
    
    # Initialize the CAN bus using the specified interface and socketcan
    try:
        bus = can.interface.Bus(channel=can_interface, interface='socketcan')
        rospy.loginfo("CAN bus initialized on %s", can_interface)
    except Exception as e:
        rospy.logerr("Failed to initialize CAN bus on %s: %s", can_interface, str(e))
        return

    rate = rospy.Rate(100)  # Loop rate in Hz
    rospy.loginfo("Listening for CAN messages on %s...", can_interface)

    while not rospy.is_shutdown():
        try:
            # Receive a CAN message with a timeout of 1 second
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue  # No message received; continue loop

            # Create a can_msgs/Frame message
            frame = Frame()
            frame.id = msg.arbitration_id
            frame.dlc = msg.dlc if hasattr(msg, 'dlc') else len(msg.data)
            frame.data = list(msg.data)
            frame.is_extended = msg.is_extended_id
            frame.is_rtr = msg.is_remote_frame
            frame.is_error = msg.is_error_frame
            
            bytes = list(msg.data) 
            print("data in decimal ", bytes)

            # Publish the CAN frame
            can_pub.publish(bytes)
        except Exception as ex:
            rospy.logerr("Error while receiving CAN message: %s", str(ex))
        rate.sleep()

if __name__ == '__main__':
    try:
        can_receiver()
    except rospy.ROSInterruptException:
        pass
