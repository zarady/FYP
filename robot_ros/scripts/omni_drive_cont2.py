#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Publishers
front_pub = rospy.Publisher('/frontwheel_velocity_controller/command', Float64, queue_size=1)
right_pub = rospy.Publisher('/rightwheel_velocity_controller/command', Float64, queue_size=1)
back_pub = rospy.Publisher('/backwheel_velocity_controller/command', Float64, queue_size=1)
left_pub = rospy.Publisher('/leftwheel_velocity_controller/command', Float64, queue_size=1)

SPEED = 1.0
last_cmd_time = 0
TIMEOUT = 0.2  # seconds

motor_values = [0, 0, 0, 0]  # Global to be reused in watchdog

def cmd_vel_callback(msg):
    global last_cmd_time, motor_values

    lx = msg.linear.x
    az = msg.angular.z
    last_cmd_time = rospy.get_time()

    # Default: stop
    motor_values = [0, 0, 0, 0]

    if lx > 0 and az == 0:
        motor_values = [+1, +1, -1, -1]
    elif lx < 0 and az == 0:
        motor_values = [-1, -1, +1, +1]
    elif lx == 0 and az > 0:
        motor_values = [+1, -1, -1, +1]
    elif lx == 0 and az < 0:
        motor_values = [-1, +1, +1, -1]
    elif lx == 0 and az == 0:
        motor_values = [+1, +1, +1, +1]
    elif lx > 0 and az > 0:
        motor_values = [+1, 0, +1, 0]
    elif lx > 0 and az < 0:
        motor_values = [0, +1, 0, +1]
    elif lx < 0 and az > 0:
        motor_values = [0, -1, 0, -1]
    elif lx < 0 and az < 0:
        motor_values = [-1, 0, -1, 0]

    motor_values = [v * SPEED for v in motor_values]
    publish_to_motors()

def publish_to_motors():
    front_pub.publish(Float64(motor_values[0]))
    right_pub.publish(Float64(motor_values[1]))
    back_pub.publish(Float64(motor_values[2]))
    left_pub.publish(Float64(motor_values[3]))

def stop_motors():
    front_pub.publish(Float64(0))
    right_pub.publish(Float64(0))
    back_pub.publish(Float64(0))
    left_pub.publish(Float64(0))

def watchdog_timer(event):
    if rospy.get_time() - last_cmd_time > TIMEOUT:
        stop_motors()

def main():
    rospy.init_node('omni_drive_controller')
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.Timer(rospy.Duration(0.1), watchdog_timer)  # Check for timeout every 100ms
    rospy.spin()

if __name__ == '__main__':
    main()
