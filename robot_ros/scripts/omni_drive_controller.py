#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Setup publishers
front_pub = rospy.Publisher('/frontwheel_velocity_controller/command', Float64, queue_size=1)
back_pub = rospy.Publisher('/backwheel_velocity_controller/command', Float64, queue_size=1)
left_pub = rospy.Publisher('/leftwheel_velocity_controller/command', Float64, queue_size=1)
right_pub = rospy.Publisher('/rightwheel_velocity_controller/command', Float64, queue_size=1)

def cmd_vel_callback(msg):
    vx = msg.linear.x      # forward/backward
    vy = msg.linear.y      # right/left (crab style)
    omega = msg.angular.z  # rotation (optional future)

    # Initialize wheel speeds
    front_speed = 0.0
    back_speed = 0.0
    left_speed = 0.0
    right_speed = 0.0

    # Forward/backward (Left & Right wheels)
    if vx != 0:
        left_speed = vx
        right_speed = vx

    # Sideways crab (Front & Back wheels)
    if vy != 0:
        front_speed = vy
        back_speed = vy

    # (Optional) Add angular.z support here if needed later

    # Publish to each motor
    front_pub.publish(front_speed)
    back_pub.publish(back_speed)
    left_pub.publish(left_speed)
    right_pub.publish(right_speed)

def main():
    rospy.init_node('omni_drive_controller')
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
