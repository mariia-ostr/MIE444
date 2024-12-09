#!/usr/bin/env python3
import rospy
import serial
import time
from std_msgs.msg import String

class SerialNode:
    def __init__(self):
        rospy.init_node('serial_node')
        self.ser = None
        self.connect_serial()

        self.pub = rospy.Publisher('esp32_data', String, queue_size=10)
        rospy.Subscriber('to_esp32', String, self.send_callback)

    def connect_serial(self):
        while not rospy.is_shutdown():
            try:
                self.ser = serial.Serial(
                    port='/dev/ttyUSB0',
                    baudrate=115200,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                )
                rospy.loginfo("Successfully connected to ESP32")
                break
            except serial.SerialException:
                rospy.logwarn("ESP32 not found. Please connect ESP32 to USB port. Retrying in 5 seconds...")
                time.sleep(5)

    def send_callback(self, msg):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((msg.data + '\n').encode())
            except serial.SerialException as e:
                rospy.logerr(f"Error sending data: {e}")
                self.connect_serial()

    def read_serial(self):
        while not rospy.is_shutdown():
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting:
                        data = self.ser.readline().decode().strip()
                        msg = String()
                        msg.data = data
                        self.pub.publish(msg)
                except serial.SerialException as e:
                    rospy.logerr(f"Error reading data: {e}")
                    self.connect_serial()
            else:
                self.connect_serial()
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        node = SerialNode()
        node.read_serial()
    except rospy.ROSInterruptException:
        pass