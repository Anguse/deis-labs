#!/usr/bin/env python

import time
import rospy

class Logger:
    def __init__(self):        
        self.state = open("/home/ubuntu/catkin_ws/src/controller/logs/state.txt", "w+")
        self.gps = open("/home/ubuntu/catkin_ws/src/controller/logs/gps.txt", "w+")
    def log_state(self, state):
        timestamp = rospy.get_time()
        row = {'timestamp': timestamp, 'state': state}
        self.state.write(str(row))
        self.state.write('\n')
    def log_gps(self, gps_frame):
        timestamp = rospy.get_time()
        row = {'timestamp':timestamp, 'gps0':gps_frame[0], 'gps1':gps_frame[1], 'gps2':gps_frame[2], 'gps3':gps_frame[3], 'gps4':gps_frame[4], 'gps5':gps_frame[5], 'gps6':gps_frame[6]}
        self.gps.write(str(row))
        self.gps.write('\n')
    def close(self):
        self.gps.close()
        self.state.close()
