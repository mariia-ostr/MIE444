#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
import board
import busio
import adafruit_vl53l0x  # Import the ToF sensor library

class DistanceSensors:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('distance_sensors', anonymous=True)
        
        # Publishers for each sensor
        self.us1_pub = rospy.Publisher('ultrasonic1_distance', Float32, queue_size=10)
        self.us2_pub = rospy.Publisher('ultrasonic2_distance', Float32, queue_size=10)
        self.tof_pub = rospy.Publisher('tof_distance', Float32, queue_size=10)
        
        # GPIO Setup for Ultrasonic Sensors
        GPIO.setmode(GPIO.BCM)
        
        # Ultrasonic 1 pins
        self.TRIG1 = 17
        self.ECHO1 = 27
        
        # Ultrasonic 2 pins
        self.TRIG2 = 22
        self.ECHO2 = 23
        
        # Setup GPIO pins
        GPIO.setup(self.TRIG1, GPIO.OUT)
        GPIO.setup(self.ECHO1, GPIO.IN)
        GPIO.setup(self.TRIG2, GPIO.OUT)
        GPIO.setup(self.ECHO2, GPIO.IN)
        
        # Initialize ToF sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.tof = adafruit_vl53l0x.VL53L0X(i2c)
        
        # Set measurement timing budget
        self.tof.measurement_timing_budget = 33000  # 33ms, increased speed
        
        rospy.on_shutdown(self.cleanup)

    def get_ultrasonic_distance(self, trig, echo):
        # Send trigger pulse
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)
        
        pulse_start = time.time()
        pulse_end = time.time()
        timeout = pulse_start + 0.1  # 100ms timeout
        
        # Wait for echo to go high
        while GPIO.input(echo) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return None
        
        # Wait for echo to go low
        while GPIO.input(echo) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return None
        
        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound * time / 2
        return round(distance, 2)

    def run(self):
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown():
            try:
                # Read Ultrasonic 1
                dist1 = self.get_ultrasonic_distance(self.TRIG1, self.ECHO1)
                if dist1 is not None:
                    self.us1_pub.publish(Float32(dist1))
                
                # Read Ultrasonic 2
                dist2 = self.get_ultrasonic_distance(self.TRIG2, self.ECHO2)
                if dist2 is not None:
                    self.us2_pub.publish(Float32(dist2))
                
                # Read ToF sensor
                tof_dist = self.tof.range
                self.tof_pub.publish(Float32(tof_dist))
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error reading sensors: {str(e)}")
                continue

    def cleanup(self):
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        sensor_node = DistanceSensors()
        sensor_node.run()
    except rospy.ROSInterruptException:
        pass