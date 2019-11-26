#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from math import pi, modf

wb_cm = 15 ## wheelbase
countsPerRev = 192
wheelCirc = pi*6.5

def arduino_cb(data):
    params = data.data.split(',')
    if len(params) < 4:
        print("not enough params for odom")
        return
    busy = int(params[0])
    left_enc = float(float(params[1])/countsPerRev)*wheelCirc
    right_enc = float(float(params[2])/countsPerRev)*wheelCirc
    ult_sonic = float(params[3])
    offset_angle = 1.5*((left_enc - right_enc)/(wb_cm))%(2*pi)
    pub.publish(str(offset_angle))

if __name__ == "__main__":
    rospy.init_node('serial_comm', anonymous=True)
    pub = rospy.Publisher('bigboy_odom', String, queue_size=10)
    rospy.Subscriber('bigboy_arduino_read', String, arduino_cb)
    rospy.spin()


