#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String


# Changed serial from timeout = 1 to write timeout = .5 to try and reduce how often
# we can send data to the arduino. This is because the arduino needs time to make own adjustments.
# For example when in lane following mode. Will this cause trouble due to buffered up messages being delivered all at once?
# Try preventing this by checking arduino.writeable()

try:
    arduino = serial.Serial('/dev/ttyUSB0', 9600)
except serial.SerialException:
    rospy.logerr('arduino not connected')
    exit()

def arduino_write_cb(data):
    rospy.loginfo(data)
    arduino.flush()
    arduino.write(data.data.encode())
    arduino.flush()

if __name__ == '__main__':
    rospy.init_node('serial_comm', anonymous=True)
    pub = rospy.Publisher('bigboy_arduino_read', String, queue_size=10)
    rospy.Subscriber('bigboy_arduino_write', String, arduino_write_cb)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        arduino_data = arduino.read_until('\n')
        pub.publish(arduino_data[:-1])
        r.sleep()
        

        
