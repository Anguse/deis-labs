#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

def arduino_write_cb(data):
    serial.write(data.data.encode())

if __name__ == '__main__':
    rospy.init_node('serial_comm', anonymous=True)
    pub = rospy.Publisher('bigboy_arduino_read', String, queue_size=10)
    rospy.Subscriber('bigboy_arduino_write', String, arduino_write_cb)
    try:
        serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    except serial.SerialException:
        print('Arduino not connected')
        exit()        
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        arduino_data = serial.read_until('\n')
        pub.publish(arduino_data[:-1])
        r.sleep()
        

        
