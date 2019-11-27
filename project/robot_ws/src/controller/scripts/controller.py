#!/usr/bin/env python

import rospy
import math
from math import cos, sin, atan2, pi, sqrt
from std_msgs.msg import String, Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Range, Illuminance

LINE_FOLLOWING_MODE   = 0
LISTENING_MODE        = 1
SHRIMP_FOLLOWING_MODE = 2
SIDE_FORMATION_MODE   = 3

OUTSIDE_BOUNDARY = 98
OUTER_LANE_BOUNDARY = 76
INNER_LANE_BOUNDARY = 55

INIT_ANGLE    = 0.0
DEFAULT_SPEED = 50

# 2. Side formation needs testing, leader now updates the speed
#    to half of default speed when follower is behind. 
# 3. Speed is updated according to state['speed'] every heartbeat as long as the arduino is not busy
# 4. Tries to switch to the other lane if its in the same lane as follower. Initially both are in 0(inner)

#TODO: add side formation mode in robot. Remove hardcoded case from action cb
#      check why the communication to the arduino is so bad
#      change time on raspberry

class Controller:
    
    def __init__(self, state, platoons):

        self.state = state
        self.busy = False
        self.break_applied = 0.0
        self.platoons = platoons
        self.heartbeat_pub = rospy.Publisher('heartbeat', String, queue_size=10)
        self.feedback_pub = rospy.Publisher('feedback', String, queue_size=10)
        self.action_pub = rospy.Publisher('action', String, queue_size=10)
        self.leftWheel_pub = rospy.Publisher('bigboy/arduino/leftWheel', Int16, queue_size=-1)
        self.rightWheel_pub = rospy.Publisher('bigboy/arduino/rightWheel', Int16, queue_size=-1)
        self.left_ir = 0.0
        self.right_ir = 0.0

        rospy.Subscriber('bigboy/ultrasound', Range, self.ultrasound_cb)
        rospy.Subscriber('bigboy/left', Illuminance, self.left_outer_cb)
        rospy.Subscriber('bigboy/right', Illuminance, self.right_outer_cb)

        rospy.Subscriber('josefoutput', String, self.gps_cb)
        rospy.Subscriber('action', String, self.action_cb)
        rospy.Subscriber('heartbeat', String, self.heartbeat_cb)
        rospy.Subscriber('feedback', String, self.feedback_cb)
        rospy.Subscriber('bigboy_arduino_read', String, self.arduino_cb)
        rospy.Subscriber('bigboy_shrimp', String, self.shrimp_cb)
        rospy.Subscriber('bigboy_odom', String, self.odom_cb)

    def gps_cb(self,data):
        states = data.data.split(';')
        state = states[self.state['ID']].replace('[', '')
        params = state.split(' ')
        map_center = {'x':302, 'y':417}

        if params[0] != str(-1) and params[1] != str(-1) and params[2] != str(-1) and params[4] != str(-1):
            self.state['x'] = params[0]
            self.state['y'] = params[1]
            self.state['z'] = params[2]

            # Calculate polar coordinates
            x = float(self.state['x']) - map_center['x']
            y = float(self.state['y']) - map_center['y']
            self.state['polar_angle'] = atan2(y,x)
            self.state['polar_r'] = sqrt(pow(x,2)+pow(y,2))

        gps_frame = []
        adjusted_speed = 0
        for i in range(1,len(states)):
            state = states[i].replace('[', '')
            params = state.split(' ')
            x = float(params[0]) - map_center['x']
            y = float(params[1]) - map_center['y']
            polar_angle = atan2(y,x)
            polar_r = sqrt(pow(x,2)+pow(y,2))

            ## Only avoid robots in platoon
            if i in self.platoons[self.state['platoon']]['robots']:
                robot_state = self.platoons[self.state['platoon']]['robot_states'][i]
                robot_state['x'] = params[0]
                robot_state['y'] = params[1]
                robot_state['z'] = params[2]
                robot_state['polar_angle'] = polar_angle
                robot_state['polar_r'] = polar_r
                
                if(self.state['mode'] == SIDE_FORMATION_MODE):
                    if self.state['platoon_pos'] == 1:
                        if self.state['lane'] == robot_state['lane'] and not self.busy:
                            if self.state['lane'] == 0:
                                # Shift to left lane
                                self.state['lane'] = 1
                            else:
                                # Shift to right lane
                                self.state['lane'] = 0
                        # Break if leader and too far ahead
                        elif abs(polar_angle - self.state['polar_angle']) > pi/8:
                            ## Wait for robot behind
                            self.break_applied = .5
                        elif self.break_applied != 0:
                            self.break_applied = 0
                elif(self.state['mode'] == LINE_FOLLOWING_MODE):
                    if abs(polar_r - self.state['polar_r']) < 35 and abs(polar_angle - self.state['polar_angle']) < pi/8:
                        self.break_applied = abs(polar_angle - self.state['polar_angle'])/(pi/8)
                    elif self.break_applied != 0:
                        self.break_applied = 0
                else:
                    self.break_applied = 0
            leftWheelSpeed = self.state['speed'][0] - self.state['speed'][0]*self.break_applied
            rightWheelSpeed = self.state['speed'][1] - self.state['speed'][1]*self.break_applied
            self.state['speed'] = (leftWheelSpeed,rightWheelSpeed)
            gps_frame.append(state)
                
    def action_cb(self,data):
        params = data.data.split(',')	
        timestamp = params[0]
        action_id = params[1]
        source_robot_id = int(params[2])
        target_platoon_id = int(params[3])
        target_robot_id = int(params[4])
        msg = params[5]

        if target_robot_id != self.state['ID'] and source_robot_id != -1:
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
            newLane = int(msg)
            rospy.loginfo("current lane:%i, new_lane:%i"%(self.state['lane'], newLane))
            if self.state['lane'] < newLane:
                self.busy = True
                # stop
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(0)
                # switch left
                self.rightWheel_pub.publish(50)
                self.leftWheel_pub.publish(0)
                rospy.sleep(1.5)
                self.rightWheel_pub.publish(50)
                self.leftWheel_pub.publish(50)
                rospy.sleep(1.7)
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(50)
                rospy.sleep(1)
                self.busy = False
            elif self.state['lane'] > newLane:
                self.busy = True
                # stop
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(0)
                # switch right
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(50)
                rospy.sleep(1.5)
                self.rightWheel_pub.publish(50)
                self.leftWheel_pub.publish(50)
                rospy.sleep(1.7)
                self.rightWheel_pub.publish(50)
                self.leftWheel_pub.publish(0)
                rospy.sleep(1)
                self.busy = False
            else:
                rospy.loginfo("already in lane %i"%newLane)
            self.state['lane'] = newLane
        elif(action_id == 'e'):
            print("setRole")
            self.state['role'] = msg
        elif(action_id == 'f'):
            print("setPosition")
        elif(action_id == 'g'):
            if not self.busy:
                rospy.loginfo("setSpeed %s"%msg)
                payload_params = msg.split(';')
                leftWheelSpeed = int(payload_params[0])
                rightWheelSpeed = int(payload_params[1])
                self.state['speed'] = (leftWheelSpeed,rightWheelSpeed)
                self.leftWheel_pub.publish(leftWheelSpeed)
                self.rightWheel_pub.publish(rightWheelSpeed)
            else:
                rospy.loginfo("Busy, can't set speed now.")
        elif(action_id == 'h'):
            print("setMode")
            payload_params = msg
            newMode = int(msg)
            self.state['mode'] = newMode
            if newMode == LINE_FOLLOWING_MODE:
                self.state['speed'] = (50,50)
        elif(action_id == 'i'):
            print("turnAndTravel")
        elif(action_id == 'j'):
            print("intersection")
        elif(action_id == 'k'):
            print('free')
        elif(action_id == 'l'):
            print("free")
        elif(action_id == 'm'):
            print("changeLane")
        elif(action_id == 'n'):
            print("changeRole")
        elif(action_id == 'o'):
            print("moveToPosition")
        elif(action_id == 'p'):
            if not self.busy:
                rospy.loginfo("changeSpeed %s"%msg)
                payload_params = msg.split(';')
                leftWheelSpeed = int(payload_params[0])
                rightWheelSpeed = int(payload_params[1])
                self.state['speed'] = (leftWheelSpeed,rightWheelSpeed)
                self.leftWheel_pub.publish(leftWheelSpeed)
                self.rightWheel_pub.publish(rightWheelSpeed)
            else:
                rospy.loginfo("Busy, can't set speed now.")
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
        if robot_id in platoons[self.state['platoon']]['robots'] and robot_id != self.state['ID']:
            print "heartbeat from %i" %robot_id

    def feedback_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + 'feedback %s', data.data)

    def shrimp_cb(self, data):
        print(data)
        params = data.data.split(',')
        timestamp = params[0]
        x = int(params[1])
        y = int(params[2])
        if self.state['mode'] == SHRIMP_FOLLOWING_MODE:
            if x > 210:
                # Turn right
                leftWheelSpeed = 50
                rightWheelSpeed = 0
                action_id = 'g'
                self.arduino_write_pub.publish(action_id+','+str(leftWheelSpeed)+','+str(rightWheelSpeed)+'\n')
                self.state['speed'] = (leftWheelSpeed,rightWheelSpeed)
            else:
                # left
                leftWheelSpeed = 0
                rightWheelSpeed = 50
                action_id = 'g'
                self.arduino_write_pub.publish(action_id+','+str(leftWheelSpeed)+','+str(rightWheelSpeed)+'\n')
                self.state['speed'] = (leftWheelSpeed,rightWheelSpeed)
        #dx = x - self.state['x']
        #dy = y - self.state['y']
        #if abs(dx) < 5 or abs(dy) < 5 or self.state['mode'] != SHRIMP_FOLLOWING_MODE or self.busy:
        #    return
        #dist = sqrt(pow(dx,2)+pow(dy,2))
        #direction = atan2(dx,dy)
        #print("")
        #print("         distance: %f" %float(dist))
        #print("        direction: %f" %float(direction))
        # turn and travel

    def odom_cb(self, data):
        angle = float(data.data)
        self.state['theta'] = INIT_ANGLE + float(data.data)

    def arduino_cb(self, data):
        params = data.data.split(',')
        if len(params) < 4:
            return
        busy = params[0]
        left_enc = params[1]
        right_enc = params[2]
        ult_sonic = params[3]
        try:
            self.busy = bool(int(busy))
        except ValueError:
            pass

    def ultrasound_cb(self, data):
        rospy.logdebug(data.range)

    def left_outer_cb(self, data):
        rospy.logdebug(data.illuminance)
        self.left_ir = data.illuminance
    def right_outer_cb(self, data):
        rospy.logdebug(data.illuminance)
        self.right_ir = data.illuminance


if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)

    tinyboy_state = {
        'ID':1, 
        'platoon':0, 
        'platoon_pos':2,
        'type':-1, 
        'lane':0,
        'role':None, 
        'mode':LISTENING_MODE,
        'speed':(0,0),
        'x':0, 
        'y':0, 
        'z':0, 
        'theta':INIT_ANGLE,
        'polar_r':0.0,
        'polar_angle':0.0
    }
    bigboy_state = {
        'ID':0, 
        'platoon':0, 
        'platoon_pos':1,
        'type':-1, 
        'lane':0,
        'role':None, 
        'mode':LISTENING_MODE,
        'speed':(0,0),
        'x':0, 
        'y':0, 
        'z':0, 
        'theta':INIT_ANGLE,
        'polar_r':0.0,  
        'polar_angle':  0.0
    }
    platoons = [
            {
            'ID':0,
            'robots':[
                0,
                1
            ],
            'robot_states':[
                bigboy_state,
                tinyboy_state
            ],
            'leader':1
            }
    ]
    ctrl = Controller(state=bigboy_state, platoons=platoons)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not ctrl.busy:
            if ctrl.state['mode'] == LINE_FOLLOWING_MODE:
                if ctrl.left_ir > 700 and ctrl.right_ir < 700:
                    ctrl.rightWheel_pub.publish(0)
                    rospy.sleep(.01)
                if ctrl.right_ir > 700 and ctrl.left_ir < 700:
                    ctrl.leftWheel_pub.publish(0)
                    rospy.sleep(.01)
            ctrl.leftWheel_pub.publish(int(ctrl.state['speed'][0]))
            ctrl.rightWheel_pub.publish(int(ctrl.state['speed'][1]))
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