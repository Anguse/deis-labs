#!/usr/bin/env python

import rospy
import math
from math import cos, sin, atan2, pi, sqrt
from std_msgs.msg import String, Int16, Int32
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

LEADER_LANE = 1
FOLLOWER_LANE = 0

INIT_ANGLE    = 0.0
DEFAULT_SPEED = 50

LINE_TRESHOLD = 700

# 2. Side formation needs testing, leader now updates the speed
#    to half of default speed when follower is behind. 
# 3. Speed is updated according to state['speed'] every heartbeat as long as the arduino is not busy
# 4. Tries to switch to the other lane if its in the same lane as follower. Initially both are in 0(inner)

#TODO: add side formation mode in robot. Remove hardcoded case from action cb
#      check why the communication to the arduino is so bad
#      change time on raspberry

class Controller:
    
    def __init__(self, state, platoons, robot):

        self.state = state
        self.busy = False
        self.break_applied = 0.0
        self.platoons = platoons
        self.heartbeat_pub = rospy.Publisher('heartbeat', String, queue_size=10)
        self.feedback_pub = rospy.Publisher('feedback', String, queue_size=10)
        self.action_pub = rospy.Publisher('action', String, queue_size=10)
        self.leftWheel_pub = rospy.Publisher(robot+'/arduino/leftWheel', Int16, queue_size=10)
        self.rightWheel_pub = rospy.Publisher(robot+'/arduino/rightWheel', Int16, queue_size=10)
        self.lineFollow_pub = rospy.Publisher(robot+'/arduino/linefollow', Int16, queue_size=10)
        self.laneSwitch_pub = rospy.Publisher(robot+'/arduino/laneswitch', Int16, queue_size=10)

        self.left_ir = 0.0
        self.left_inner_ir = 0.0
        self.right_ir = 0.0
        self.right_inner_ir = 0.0

        rospy.Subscriber(robot+'/ultrasound', Range, self.ultrasound_cb)
        rospy.Subscriber(robot+'/left', Illuminance, self.left_outer_cb)
        rospy.Subscriber(robot+'/left_inner', Illuminance, self.left_inner_cb)
        rospy.Subscriber(robot+'/right', Illuminance, self.right_outer_cb)
        rospy.Subscriber(robot+'/right_inner', Illuminance, self.right_inner_cb)
        rospy.Subscriber(robot+'/right_enc', Int32, self.right_enc_cb)
        rospy.Subscriber(robot+'/left_enc', Int32, self.left_enc_cb)

        rospy.Subscriber('josefoutput', String, self.gps_cb)
        rospy.Subscriber('action', String, self.action_cb)
        rospy.Subscriber('heartbeat', String, self.heartbeat_cb)
        rospy.Subscriber('feedback', String, self.feedback_cb)
        rospy.Subscriber(robot+'_arduino_read', String, self.arduino_cb)
        rospy.Subscriber(robot+'_shrimp', String, self.shrimp_cb)

    def gps_cb(self,data):
        states = data.data.split(';')
        nose_state_params = states[1].replace('[', '').split(' ')
        tail_state_params = states[0].replace('[', '').split(' ')

        # Calculate pose
        '''
        dx = float(nose_state_params[0]) - float(tail_state_params[0])
        dy = float(nose_state_params[1]) - float(tail_state_params[1])
        self.state['theta'] = atan2(dy,dx)
        

        params = tail_state_params
        '''
        params = states[self.state['ID']].replace('[', '').split(' ')
        map_center = {'x':302, 'y':417}

        if params[0] != str(-1) and params[1] != str(-1) and params[2] != str(-1) and params[4] != str(-1):
            self.state['x'] = params[0]
            self.state['y'] = params[1]
            # angle = params[2]
            # id_digit = params[3]
            # certainty = params[4]

            # Calculate polar coordinates
            x = float(self.state['x']) - map_center['x']
            y = float(self.state['y']) - map_center['y']
            self.state['polar_angle'] = atan2(y,x)
            self.state['polar_r'] = sqrt(pow(x,2)+pow(y,2))

        gps_frame = []
        adjusted_speed = 0
        for i in range(len(states)):
            if i == self.state['ID']:
                continue
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
                #print("robot %s in platoon in position (%s,%s)"%(robot_state['ID'], robot_state['x'], robot_state['y']))
                
                if(self.state['mode'] == SIDE_FORMATION_MODE):
                    if self.state['platoon_pos'] == 1:
                        if self.state['lane'] == robot_state['lane'] and not self.busy:
                            if self.state['lane'] == FOLLOWER_LANE:
                                # Shift to left lane
                                self.state['lane'] = LEADER_LANE
                        # Break if leader and too far ahead
                        elif abs(polar_angle - self.state['polar_angle']) > pi/8:
                            ## Wait for robot behind
                            print("waiting for robot behind")
                            self.break_applied = .25
                        elif self.break_applied != 0:
                            self.break_applied = 0
                            self.state['speed'] = (50,50)
                elif(self.state['mode'] == LINE_FOLLOWING_MODE):
                    if self.state['platoon_pos'] > 1:
                        if abs(polar_r - self.state['polar_r']) < 35 and (polar_angle - self.state['polar_angle']) < pi/8:
                            self.break_applied = abs(polar_angle - self.state['polar_angle'])/(pi/8)
                            print("apply break for %s in position (%s,%s)"%(str(robot_state['ID']), robot_state['x'], robot_state['y']))
                            print("my position (%s,%s)"%(self.state['x'], self.state['y']))
                        elif self.break_applied != 0:
                            self.break_applied = 0
                            self.state['speed'] = (50,50)
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
                if self.state['mode'] == LINE_FOLLOWING_MODE or self.state['mode'] == SIDE_FORMATION_MODE:
                    self.lineFollow_pub.publish(-1)
                self.busy = True
                self.laneSwitch_pub.publish(1)
                '''
                # stop
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(0)
                # switch left
                self.rightWheel_pub.publish(80)
                self.leftWheel_pub.publish(0)
                rospy.sleep(.58)
                self.rightWheel_pub.publish(80)
                self.leftWheel_pub.publish(80)
                rospy.sleep(1)
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(80)
                rospy.sleep(.58)
                '''
                self.busy = False
                if self.state['mode'] == LINE_FOLLOWING_MODE or self.state['mode'] == SIDE_FORMATION_MODE:
                    self.lineFollow_pub.publish(50)
            elif self.state['lane'] > newLane:
                if self.state['mode'] == LINE_FOLLOWING_MODE or self.state['mode'] == SIDE_FORMATION_MODE:
                    self.lineFollow_pub.publish(-1)
                self.busy = True
                self.laneSwitch_pub.publish(0)
                '''
                # stop
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(0)
                # switch right
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(80)
                rospy.sleep(.58)
                self.rightWheel_pub.publish(80)
                self.leftWheel_pub.publish(80)
                rospy.sleep(1)
                self.rightWheel_pub.publish(80)
                self.leftWheel_pub.publish(0)
                rospy.sleep(.58)
                '''
                self.busy = False
                if self.state['mode'] == LINE_FOLLOWING_MODE or self.state['mode'] == SIDE_FORMATION_MODE:
                    self.lineFollow_pub.publish(50)
            else:
                rospy.loginfo("already in lane %i"%newLane)
            self.state['lane'] = newLane
        elif(action_id == 'e'):
            print("setRole")
            self.state['role'] = msg
        elif(action_id == 'f'):
            print("setPosition")
            # only pos 1 or 2
            newPos = msg
            if newPos < self.state['platoon_pos']:
                print("become follower")
            elif newPos > self.state['platoon_pos']:
                print("become leader")
            else:
                print("no change")
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
                self.lineFollow_pub.publish(50)
                self.state['speed'] = (50,50)
            else:
                self.lineFollow_pub.publish(-1)
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
            #print "heartbeat from %i" %robot_id
            return

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
    
    def left_inner_cb(self, data):
        rospy.logdebug(data.illuminance)
        self.left_inner_ir = data.illuminance
    
    def right_outer_cb(self, data):
        rospy.logdebug(data.illuminance)
        self.right_ir = data.illuminance
    
    def right_inner_cb(self, data):
        rospy.logdebug(data.illuminance)
        self.right_inner_ir = data.illuminance

    def left_enc_cb(self, data):
        rospy.loginfo("left: %s"%data.data)

    def right_enc_cb(self, data):
        rospy.loginfo("right: %s"%data.data)

if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    robot = 'tinyboy'
    tinyboy_state = {
        'ID':1, 
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
        'polar_angle':0.0
    }
    bigboy_state = {
        'ID':0, 
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
    ctrl = Controller(state=tinyboy_state, platoons=platoons, robot=robot)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not ctrl.busy:
            if ctrl.state['mode'] == LINE_FOLLOWING_MODE or ctrl.state['mode'] == SIDE_FORMATION_MODE:
                ctrl.lineFollow_pub.publish(int(ctrl.state['speed'][0]))
            else:
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