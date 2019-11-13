#!/usr/bin/env python

import rospy
from std_msgs.msg import String

wb_mm = 160 ## wheelbase

def arduino_cb(data):
    params = data.data.split(',')
    busy = params[0]
    left_enc = params[1]
    right_enc = params[2]
    ult_sonic = params[3]
    offset_angle = (left_enc - right_enc)/wb_mm
    pub.publish(offset_angle)
    

if __name__ == "__main__":
    rospy.init_node('serial_comm', anonymous=True)
    pub = rospy.Publisher('backstreet_odometry', String, queue_size=10)
    rospy.Subscriber('backstreet_arduino', String, arduino_cb)



