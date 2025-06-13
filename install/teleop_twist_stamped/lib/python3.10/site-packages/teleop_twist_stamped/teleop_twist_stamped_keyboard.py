#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty
import time

# Save terminal settings at startup for restoration on exit
settings = termios.tcgetattr(sys.stdin)

msg = """
Custom Teleop: Press keys to increment or decrement motion in specific directions.
------------------------------------------------
f: +Forward      v: -Forward
l: +Turn Left    r: -Turn Right
u: +Up (Z)       d: -Up (Z)
s: STOP ALL MOTION

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %.2f\tturn %.2f" % (speed, turn)

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', QoSProfile(depth=10))

    speed = 0.5
    turn = 1.0
    x = 0.0
    th = 0.0
    z = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while rclpy.ok():
            key = getKey()
            print(f"Key: {key}, x: {x}, th: {th}, z: {z}")  # Debug output

            # Reset all motion variables except when adjusting speed
            if key not in speedBindings and key != '\x03':
                x = 0.0
                th = 0.0
                z = 0.0

            if key == 'f':     # Forward
                x = 1.0
            elif key == 'v':   # Backward
                x = -1.0
            elif key == 'l':   # Turn Left
                th = 1.0
            elif key == 'r':   # Turn Right
                th = -1.0
            elif key == 'u':   # Up
                z = 1.0
            elif key == 'd':   # Down
                z = -1.0
            elif key == 's':   # STOP
                x = 0.0
                th = 0.0
                z = 0.0
                print("🛑 STOP issued: All velocities set to zero.")
            elif key in speedBindings:
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
                print(vels(speed, turn))
                continue
            elif key == '\x03':  # Ctrl+C
                break
            else:
                # Any other key: stop all motion
                x = 0.0
                th = 0.0
                z = 0.0

            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = node.get_clock().now().to_msg()
            twist_stamped.header.frame_id = 'base_link'
            twist_stamped.twist.linear.x = x * speed
            twist_stamped.twist.linear.y = 0.0
            twist_stamped.twist.linear.z = z * speed
            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.angular.z = th * turn
            pub.publish(twist_stamped)

            time.sleep(0.05)

    except Exception as e:
        print(e)

    finally:
        # Always send a stop command and restore terminal settings
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = node.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = 0.0
        pub.publish(twist_stamped)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
