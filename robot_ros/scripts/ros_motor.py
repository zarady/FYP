#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

# === Configuration ===
SERIAL_PORT = '/dev/ttyUSB0'  # Update if needed
BAUD_RATE = 9600

def ros_to_arduino_callback(msg):
    # Only accept single-char commands
    if msg.data in ['w', 'a', 's', 'd', 'c']:
        ser.write((msg.data + '\n').encode())
        rospy.loginfo(f"Sent to Arduino: {msg.data}")

if __name__ == '__main__':
    rospy.init_node('arduino_bridge_node')
    
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    rospy.sleep(2)  # Wait for Arduino reset

    rospy.Subscriber('/cmd_motor', String, ros_to_arduino_callback)
    encoder_pub = rospy.Publisher('/encoder_counts', Int32MultiArray, queue_size=10)

    rate = rospy.Rate(10)
    buffer = ""

    while not rospy.is_shutdown():
        try:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                lines = buffer.split("\n")
                buffer = lines[-1]  # Keep the incomplete line
                for line in lines[:-1]:
                    if "Enc1" in line:
                        parts = line.replace("Enc", "").replace(":", "").split()
                        counts = [int(parts[1]), int(parts[3]), int(parts[5]), int(parts[7])]
                        encoder_pub.publish(Int32MultiArray(data=counts))
        except Exception as e:
            rospy.logwarn(f"Serial read error: {e}")
        
        rate.sleep()
