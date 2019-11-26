#!/usr/bin/env python

import serial
import rospy
import sys
from std_msgs.msg import String


# Changed serial from timeout = 1 to write timeout = .5 to try and reduce how often
# we can send data to the arduino. This is because the arduino needs time to make own adjustments.
# For example when in lane following mode. Will this cause trouble due to buffered up messages being delivered all at once?
# Try preventing this by checking arduino.writeable()

try:
    arduino = serial.Serial('/dev/ttyUSB0', 9600, write_timeout=1)
except serial.SerialException:
    rospy.logerr('arduino not connected')
    exit()

def arduino_write_cb(data):
    #nrof_params = len(data.data.split(','))
    sizeof_payload = len(data.data.encode('utf-8'))
    data.data = str(sizeof_payload) + ',' + data.data
    #data.data = str(nrof_params) + ',' + data.data
    #data.data = data.data.replace('\n', ','+str(nrof_params)) 
    #data.data += '\n'
    arduino.write(data.data.encode())
    arduino.flush()
    rospy.loginfo(data)

if __name__ == '__main__':
    rospy.init_node('serial_comm', anonymous=True, log_level=rospy.DEBUG)
    pub = rospy.Publisher('bigboy_arduino_read', String, queue_size=10)
    rospy.Subscriber('bigboy_arduino_write', String, arduino_write_cb)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        if arduino.in_waiting > 0:
            arduino_data = arduino.read_until('\n')
            rospy.logdebug(arduino_data)
            pub.publish(arduino_data[:-1])
        r.sleep()
        

        
