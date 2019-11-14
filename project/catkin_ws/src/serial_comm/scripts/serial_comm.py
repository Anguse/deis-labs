#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

try:
    arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
except serial.SerialException:
    print('Arduino not connected')
    exit()

def arduino_write_cb(data):
    arduino.write(data.data.encode())

if __name__ == '__main__':
    rospy.init_node('serial_comm', anonymous=True)
    pub = rospy.Publisher('bigboy_arduino_read', String, queue_size=10)
    rospy.Subscriber('bigboy_arduino_write', String, arduino_write_cb)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        arduino_data = arduino.read_until('\n')
        pub.publish(arduino_data[:-1])
        r.sleep()
        

        
