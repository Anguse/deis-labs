#!/usr/bin/env python

import serial
import rospy
import math
from math import cos, sin, atan2, pi
from logger import Logger
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

LINE_FOLLOWING_MODE   = 0
LISTENING_MODE        = 1
SHRIMP_FOLLOWING_MODE = 2

class Controller:
    
    def __init__(self, state):
        self.state = state
        self.logger = Logger()
        self.serial = None 
        self.busy = False
        self.heartbeat_pub = rospy.Publisher('heartbeat', String, queue_size=10)
        self.feedback_pub = rospy.Publisher('feedback', String, queue_size=10)
        self.action_pub = rospy.Publisher('action', String, queue_size=10)
        rospy.Subscriber('josefoutput', String, self.gps_cb)
        rospy.Subscriber('action', String, self.action_cb)
        rospy.Subscriber('heartbeat', String, self.heartbeat_cb)
        rospy.Subscriber('feedback', String, self.feedback_cb)
        rospy.Subscribe('shrimp', String, self.shrimp_cb)
        rospy.Subscribe('odom', Odometry, self.odom_cb)
        rospy.Subscribe('arduino', String, self.arduino_cb)

    def init_arduino(self):
        self.serial = serial.Serial('/dev/ttyUSB0', 9600)
    
    def gps_cb(self,data):
        states = data.data.split(';')
        state = states[self.state['ID']].replace('[', '')
        params = state.split(' ')

        if params[0] != str(-1) and params[1] != str(-1) and params[2] != str(-1) and params[4] != str(-1):
            self.state['x'] = params[0]
            self.state['y'] = params[1]
            self.state['z'] = params[2]
            self.state['theta'] = params[3]
            self.logger.log_state(self.state)

            print("")
            print("              x: %s" %self.state['x'])
            print("              y: %s" %self.state['y'])
            print("              z: %s" %self.state['z'])
            print("             ID: %s" %self.state['ID'])
            print("          theta: %s" %self.state['theta'])
            print("")

        gps_frame = []
        for i in range(len(states)):
            state = states[i].replace('[', '')
            gps_frame.append(state)
        self.logger.log_gps(gps_frame)

    def action_cb(self,data):
        params = data.data.split(',')	
        timestamp = params[0]
        action_id = params[1]
        source_robot_id = int(params[2])
        target_platoon_id = int(params[3])
        target_robot_id = int(params[4])
        msg = params[5]

        if target_robot_id != self.state['ID']:
            print('action msg to another id')
            return

        print("")
        print("        timestamp: %s" %timestamp)
        print("        action_id: %s" %action_id)
        print("  source_robot_id: %d" %source_robot_id)
        print("target_platoon_id: %d" %target_platoon_id)
        print("  target_robot_id: %d" %target_robot_id)
        print("              msg: %s" %msg)
        print("")

        if(action_id == 'a'):
            print("setPlatoon")
            self.state['platoon'] = int(msg)
        elif(action_id == 'b'):
            print("setID")
            self.state['ID'] = int(msg)
        elif(action_id == 'c'):
            print("setType")	
            self.state['type'] = msg
        elif(action_id == 'd'):
            print("setLane")
            self.state['lane'] = int(msg)
        elif(action_id == 'e'):
            print("setRole")
            self.state['role'] = msg
        elif(action_id == 'f'):
            print("setPosition")
            # This doesnt make any sense
        elif(action_id == 'g'):
            print("setSpeed")
            payload_params = msg.split(';')
            leftWheelSpeed = int(payload_params[0])
            rightWheelSpeed = int(payload_params[1])
            # Add sign to byte to express negative values
            if(leftWheelSpeed < 0):
                leftWheelSpeed = -leftWheelSpeed+128
            if(rightWheelSpeed < 0):
                rightWheelSpeed = -rightWheelSpeed+128
            endMarker = '\n'
            self.state['speed'] = (leftWheelSpeed,rightWheelSpeed)
            if self.serial:
                self.serial.write(action_id+chr(leftWheelSpeed)+chr(rightWheelSpeed)+endMarker)
        elif(action_id == 'h'):
            print("setMode")
            payload_params = msg
            newMode = int(msg[0])
            endMarker = '\n'
            self.state['mode'] = newMode
            if self.serial:
                self.serial.write(chr(action_id)+chr(newMode)+endMarker)
            mode = newMode
        elif(action_id == 'i'):
            print("turnAndTravel")       ## Never called from here
        elif(action_id == 'j'):
            print('free')
        elif(action_id == 'k'):
            print("free")
        elif(action_id == 'l'):
            print("free")
        elif(action_id == 'm'):
            print("changeLane")
        elif(action_id == 'n'):
            print("changeRole")
        elif(action_id == 'o'):
            print("moveToPosition")
        elif(action_id == 'p'):
            print("changeSpeed")
            payload_params = msg.split(';')
            leftWheelSpeed = int(payload_params[0])
            rightWheelSpeed = int(payload_params[1])
            # Add sign to byte to express negative values
            if(leftWheelSpeed < 0):
                leftWheelSpeed = -leftWheelSpeed+128
            if(rightWheelSpeed < 0):
                rightWheelSpeed = -rightWheelSpeed+128
            endMarker = '\n'
            self.state['speed'] = (leftWheelSpeed,rightWheelSpeed)
            if self.serial:
                self.serial.write(action_id+chr(leftWheelSpeed)+chr(rightWheelSpeed)+endMarker)
        elif(action_id == 'q'):
            print("specialRequest")
        elif(action_id == 'r'):
            print("mergeRequest")
        elif(action_id == 's'):
            print("free")
        elif(action_id == 't'):
            print("free")
        elif(action_id == 'u'):
            print("free")
        elif(action_id == 'v'):
            print("free")
        elif(action_id == 'w'):
            print("free")
        elif(action_id == 'x'):
            print("free")
        elif(action_id == 'y'):
            print("free")
        elif(action_id == 'z'):
            print("free")

    def heartbeat_cb(self,data):
        params = data.data.split(',')	
        timestamp = params[0]
        platoon = params[1]
        robot_id = int(params[2])
        robot_type = int(params[3])
        lane = int(params[4])
        platoon_pos = int(params[5])
        pose = params[6].split(';')
        speed = params[7].split(';')
        if robot_id != self.state['ID']:
            rospy.loginfo(rospy.get_caller_id() + 'heartbeat %s', data.data)

    def feedback_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + 'feedback %s', data.data)
    
    def shrimp_cb(self, data):
        params = data.data.split(',')
        timestamp = shrimp_pos[0]
        x = shrimp_pos[1]
        y = shrimp_pos[2]
        dx = x - self.state['x']
        dy = y - self.state['y']
        if abs(dx) < 5 or abs(dy) < 5 or self.state['mode'] != SHRIMP_FOLLOWING_MODE or self.busy:
            return
        dist = sqrt(pow(dx,2)+pow(dy,2))
        direction = atan2(dx,dy)
        if self.serial:
            # turn and travel
            speed = 50
            action_id = 'i'
            self.serial.write(action_id+chr(self.state['theta'])+chr(direction)+chr(dist)+chr(speed)+endMarker)
            self.busy = True

    def odom_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' odom %s', data)

    def arduino_cb(self, data):
        params = data.data.split(',')
        busy = params[0]
        left_enc = params[1]
        right_enc = params[2]
        ult_sonic = params[3]
        self.busy = bool(busy)

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    init_state = {
        'ID':0, 
        'platoon':-1, 
        'platoon_pos':1, 
        'type':-1, 'lane':-1, 
        'role':None, 
        'mode':0,
        'speed':(0,0), 
        'x':0, 
        'y':0, 
        'z':0, 
        'theta':0
        }
    ctrl = Controller(init_state)
    #ctrl.init_arduino()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        heartbeat_msg = str(rospy.get_time()) + ',' + \
                        str(ctrl.state['platoon']) + ',' + \
                        str(ctrl.state['ID']) + ',' + \
                        str(ctrl.state['type']) + ',' + \
                        str(ctrl.state['lane']) + ',' + \
                        str(ctrl.state['platoon_pos']) + ',' + \
                        str(ctrl.state['x']) + ';' + \
                        str(ctrl.state['y']) + ';' + \
                        str(ctrl.state['z']) + ';' + \
                        str(ctrl.state['theta']) + ',' + \
                        str(ctrl.state['speed'][0]) + ';' + \
                        str(ctrl.state['speed'][1])

        ctrl.heartbeat_pub.publish(heartbeat_msg)
        r.sleep()
