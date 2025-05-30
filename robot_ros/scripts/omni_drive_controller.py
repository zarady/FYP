#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point
import tf
import math

# Wheel velocity commands (we'll use these to simulate odometry for RViz)
motor_values = [0.0, 0.0, 0.0, 0.0]  # [front, right, back, left]

# Setup publishers for each wheel
front_pub = None
right_pub = None
back_pub = None
left_pub = None

odom_pub = None
odom_broadcaster = None

# Speed scaling factor
SPEED = 200
last_cmd_time = 0
TIMEOUT = 0.2  # seconds

# Robot pose
x = 0.0
y = 0.0
theta = 0.0

last_time = None

# === Robot physical parameters ===
WHEEL_RADIUS = 0.05  # in meters (change according to your robot)
BASE_RADIUS = 0.2    # distance from center to wheel (adjust as needed)


def publish_odometry(event):
    global x, y, theta, last_time, motor_values

    current_time = rospy.Time.now()
    if last_time is None:
        last_time = current_time
        return

    dt = (current_time - last_time).to_sec()
    last_time = current_time

    # === Simulated inverse kinematics from motor values ===
    # Assume order: [front, right, back, left]
    w1, w2, w3, w4 = motor_values
    w1 /= SPEED
    w2 /= SPEED
    w3 /= SPEED
    w4 /= SPEED

    # Omni-directional kinematics (simplified)
    vx = (w1 + w2 + w3 + w4) * (WHEEL_RADIUS / 4.0)
    vy = (-w1 + w2 - w3 + w4) * (WHEEL_RADIUS / 4.0)
    vth = (-w1 + w2 + w3 - w4) * (WHEEL_RADIUS / (4.0 * BASE_RADIUS))

    # Update robot pose
    delta_x = (vx * math.cos(theta) - vy * math.sin(theta)) * dt
    delta_y = (vx * math.sin(theta) + vy * math.cos(theta)) * dt
    delta_theta = vth * dt

    x += delta_x
    y += delta_y
    theta += delta_theta

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

    # Broadcast transform: odom -> base_link
    odom_broadcaster.sendTransform(
        (x, y, 0.0),
        odom_quat,
        current_time,
        "base_footprint",
        "odom"
    )

    # Publish odometry
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"

    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = vy
    odom.twist.twist.angular.z = vth

    odom_pub.publish(odom)


def cmd_vel_callback(msg):
    global last_cmd_time, motor_values

    last_cmd_time = rospy.get_time()

    lx = msg.linear.x
    ly = msg.linear.y
    az = msg.angular.z

    # Default: stop all motors
    motor_values = [0, 0, 0, 0]  # [front, right, back, left]

    # Movement mappings
    if lx > 0 and az == 0:
        motor_values = [+1, +1, -1, -1]
    elif lx < 0 and az == 0:
        motor_values = [-1, -1, +1, +1]
    elif lx == 0 and az > 0:
        motor_values = [-1, +1, +1, -1]
    elif lx == 0 and az < 0:
        motor_values = [+1, -1, -1, +1]
    elif lx == 0 and az == 0 and ly == 0:
        motor_values = [+1, +1, +1, +1]
    elif lx > 0 and az > 0:
        motor_values = [0, +1, 0, -1]
    elif lx > 0 and az < 0:
        motor_values = [+1, 0, -1, 0]
    elif lx < 0 and az > 0:
        motor_values = [0, -1, 0, +1]
    elif lx < 0 and az < 0:
        motor_values = [-1, 0, +1, 0]

    motor_values = [v * SPEED for v in motor_values]

    # Send to motors
    front_pub.publish(Float64(motor_values[0]))
    right_pub.publish(Float64(motor_values[1]))
    back_pub.publish(Float64(motor_values[2]))
    left_pub.publish(Float64(motor_values[3]))


def stop_motors():
    global motor_values
    motor_values = [0, 0, 0, 0]
    front_pub.publish(Float64(0))
    right_pub.publish(Float64(0))
    back_pub.publish(Float64(0))
    left_pub.publish(Float64(0))


def watchdog_timer(event):
    if rospy.get_time() - last_cmd_time > TIMEOUT:
        stop_motors()


def main():
    global front_pub, right_pub, back_pub, left_pub
    global odom_pub, odom_broadcaster

    rospy.init_node('omni_drive_controller')

    front_pub = rospy.Publisher('/frontwheel_velocity_controller/command', Float64, queue_size=1)
    right_pub = rospy.Publisher('/rightwheel_velocity_controller/command', Float64, queue_size=1)
    back_pub = rospy.Publisher('/backwheel_velocity_controller/command', Float64, queue_size=1)
    left_pub = rospy.Publisher('/leftwheel_velocity_controller/command', Float64, queue_size=1)

    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()

    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    rospy.Timer(rospy.Duration(0.05), publish_odometry)  # 20 Hz
    rospy.Timer(rospy.Duration(0.1), watchdog_timer)

    rospy.spin()


if __name__ == '__main__':
    main()
