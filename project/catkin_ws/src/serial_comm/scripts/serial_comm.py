#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('serial_comm', anonymous=True)
    pub = rospy.Publisher('backstreet_arduino', String, queue_size=10)
    serial = serial.Serial('/dev/ttyUSB0', 9600)

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        arduino_data = serial.read_until('\n')
        print(arduino_data)
        #pub.publish(arduino_data[:-1])
        r.sleep()
        

        
