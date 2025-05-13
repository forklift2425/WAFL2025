#!/usr/bin/env python3
import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

# Key mappings
move_bindings = {
    'w': (1.0,  0.0, 0.0),   # forward
    's': (-1.0, 0.0, 0.0),   # backward
    'a': (0.0,  0.0, -1.0),   # turn left
    'd': (0.0,  0.0, +1.0),  # turn right
    'q': (0.0,  -1.0, 0.0),   # strafe left
    'e': (0.0, +1.0, 0.0),   # strafe right
    ' ': (0.0,  0.0, 0.0),   # stop
}

msg = """
WAFL Teleop Controller
---------------------------
Moving:
    W       - Move forward
    S       - Move backward
    A       - Turn left
    D       - Turn right
    Q       - Strafe left
    E       - Strafe right
    Space   - Emergency stop
    Z       - Zero steering
    +/-     - Adjust speed
    CTRL-C  - Exit
"""

def getKey(timeout):
    """Read single keypress from stdin with timeout."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        return sys.stdin.read(1) if rlist else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def teleop():
    global speed, turn  # Declare global variables for modification
    
    rospy.init_node('wafl_teleop_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Initialize speeds with safe defaults
    speed = 0.5  # m/s
    turn = 0.5    # rad/s
    steering = 0.0
    
    print(msg)
    print(f"Initial speeds - Linear: {speed} m/s, Angular: {turn} rad/s")
    
    try:
        while not rospy.is_shutdown():
            key = getKey(0.1)
            twist = Twist()  # Create new Twist message each iteration
            
            if key in move_bindings:
                # Handle movement commands
                lin_x, lin_y, ang_z = move_bindings[key]
                twist.linear.x = lin_x * speed
                twist.linear.y = lin_y * speed
                twist.angular.z = ang_z * turn
                
                if key == ' ':
                    print("\nEMERGENCY STOP!")
                    speed = 0.5  # Reset to safe speed after stop
                    turn = 0.5
                
                pub.publish(twist)
                
            elif key == 'z':
                # Zero steering command
                twist.linear.z = 1.0
                pub.publish(twist)
                print("\nSteering zeroed!")
                
            elif key == '+':
                # Speed increase
                speed = min(2.0, speed + 0.1)
                print(f"\nSpeed increased: {speed:.1f} m/s")
                
            elif key == '-':
                # Speed decrease
                speed = max(0.1, speed - 0.1)
                print(f"\nSpeed decreased: {speed:.1f} m/s")
                
            elif key == '\x03':  # Ctrl-C
                break
                
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Cleanup on exit
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nTeleop shutdown complete.")

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass