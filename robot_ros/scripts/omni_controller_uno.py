#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import serial

# === Serial to Arduino ===
try:
    arduino_serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    rospy.loginfo("Connected to Arduino on /dev/ttyUSB0")
except serial.SerialException as e:
    rospy.logerr(f"Could not connect to Arduino: {e}")
    arduino_serial = None

# === Setup publishers for Gazebo simulation ===
front_pub = rospy.Publisher('/frontwheel_velocity_controller/command', Float64, queue_size=1)
right_pub = rospy.Publisher('/rightwheel_velocity_controller/command', Float64, queue_size=1)
back_pub = rospy.Publisher('/backwheel_velocity_controller/command', Float64, queue_size=1)
left_pub = rospy.Publisher('/leftwheel_velocity_controller/command', Float64, queue_size=1)

# === Speed scaling factor ===
SPEED = 1.0
PWM_SCALE = 255  # Max Arduino PWM value

def cmd_vel_callback(msg):
    lx = msg.linear.x
    az = msg.angular.z

    # Default: stop all motors [front, right, back, left]
    motor_values = [0, 0, 0, 0]

    # Movement mappings (based on your pattern)
    if lx > 0 and az == 0:            # 'i' forward
        motor_values = [+1, +1, -1, -1]
    elif lx < 0 and az == 0:          # '<' backward
        motor_values = [-1, -1, +1, +1]
    elif lx == 0 and az > 0:          # 'j' left
        motor_values = [+1, -1, -1, +1]
    elif lx == 0 and az < 0:          # 'l' right
        motor_values = [-1, +1, +1, -1]
    elif lx == 0 and az == 0:         # 'k' rotate in place
        motor_values = [+1, +1, +1, +1]
    elif lx > 0 and az > 0:           # 'u' diagonal left forward
        motor_values = [+1,  0, +1,  0]
    elif lx > 0 and az < 0:           # 'o' diagonal right forward
        motor_values = [ 0, +1,  0, +1]
    elif lx < 0 and az > 0:           # 'm' diagonal left backward
        motor_values = [ 0, -1,  0, -1]
    elif lx < 0 and az < 0:           # '>' diagonal right backward
        motor_values = [-1,  0, -1,  0]

    # === Apply speed scaling ===
    scaled = [v * SPEED for v in motor_values]

    # === Publish to Gazebo controllers ===
    front_pub.publish(Float64(scaled[0]))
    right_pub.publish(Float64(scaled[1]))
    back_pub.publish(Float64(scaled[2]))
    left_pub.publish(Float64(scaled[3]))

    # === Send to Arduino if available ===
    if arduino_serial:
        pwm_vals = [int(v * PWM_SCALE) for v in motor_values]
        cmd_str = f"V{pwm_vals[0]},{pwm_vals[1]},{pwm_vals[2]},{pwm_vals[3]}\n"
        try:
            arduino_serial.write(cmd_str.encode())
        except Exception as e:
            rospy.logerr(f"Failed to send command to Arduino: {e}")

def main():
    rospy.init_node('omni_drive_controller')
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.loginfo("Omni drive controller running...")
    rospy.spin()

if __name__ == '__main__':
    main()
