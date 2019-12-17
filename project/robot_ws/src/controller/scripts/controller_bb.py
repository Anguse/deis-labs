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
yrange_max1 = 440
yrange_max2 = 370
yrange_min1 = 420
yrange_min2 = 350

class Controller:
    
    def __init__(self, state, platoons, robot):

        self.state = state
        self.busy = False
        self.break_applied = 0.0
        self.platoons = platoons
        self.heartbeat_pub = rospy.Publisher('heartbeat', String, queue_size=10)
        self.feedback_pub = rospy.Publisher('feedback', String, queue_size=10)
        self.action_pub = rospy.Publisher('action', String, queue_size=10)
        self.leftWheel_pub = rospy.Publisher(robot+'/arduino/leftWheel', Int16, queue_size=-1)
        self.rightWheel_pub = rospy.Publisher(robot+'/arduino/rightWheel', Int16, queue_size=-1)
        self.linefollow_pub = rospy.Publisher(robot+'/arduino/linefollow', Int16, queue_size=-1)
        self.stop_pub = rospy.Publisher(robot+'/arduino/stop', Int16, queue_size=-1)
        self.resetStop_pub = rospy.Publisher(robot+'/arduino/resetStop', Int16, queue_size=-1)
        self.pubAction = rospy.Publisher('action', String, queue_size=10)
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
        dx = float(nose_state_params[0]) - float(tail_state_params[0])
        dy = float(nose_state_params[1]) - float(tail_state_params[1])
        self.state['theta'] = atan2(dy, dx)
        print(self.state['theta'])

        params = states[self.state['ID']].replace('[', '').split(' ')
        map_center = {'x': 302, 'y': 417}

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
            if i == int(self.state['ID']):
                continue
            state = states[i].replace('[', '')
            params = state.split(' ')
            x = float(params[0]) - map_center['x']
            y = float(params[1]) - map_center['y']
            polar_angle = atan2(y, x)
            polar_r = sqrt(pow(x, 2) + pow(y, 2))

            ## Only avoid robots in platoon
            if i in self.platoons[self.state['platoon']]['robots']:
                robot_state = self.platoons[self.state['platoon']]['robot_states'][i]
                robot_state['x'] = params[0]
                robot_state['y'] = params[1]
                robot_state['z'] = params[2]
                robot_state['polar_angle'] = polar_angle
                robot_state['polar_r'] = polar_r

                if (self.state['mode'] == SIDE_FORMATION_MODE):
                    if self.state['platoon_pos'] == 1:
                        if self.state['lane'] == robot_state['lane']:
                            if self.state['lane'] == FOLLOWER_LANE:
                                # Shift to left lane
                                '''
                                self.busy = True
                                self.linefollow_pub.publish(int(-1))
                                # stop
                                self.rightWheel_pub.publish(0)
                                self.leftWheel_pub.publish(0)
                                # switch left
                                self.rightWheel_pub.publish(80)
                                self.leftWheel_pub.publish(0)
                                rospy.sleep(.59)
                                self.rightWheel_pub.publish(80)
                                self.leftWheel_pub.publish(80)
                                rospy.sleep(1)
                                self.rightWheel_pub.publish(0)
                                self.leftWheel_pub.publish(80)
                                rospy.sleep(.59)
                                self.busy = False
                                self.linefollow_pub.publish(int(self.state['speed'][0]))
                                '''
                                self.state['lane'] = FOLLOWER_LANE
                            else:
                                # Shift to right lane
                                '''
                                self.busy = True
                                self.linefollow_pub.publish(int(-1))
                                # stop
                                self.rightWheel_pub.publish(0)
                                self.leftWheel_pub.publish(0)
                                # switch right
                                self.rightWheel_pub.publish(0)
                                self.leftWheel_pub.publish(80)
                                rospy.sleep(.59)
                                self.rightWheel_pub.publish(80)
                                self.leftWheel_pub.publish(80)
                                rospy.sleep(1)
                                self.rightWheel_pub.publish(80)
                                self.leftWheel_pub.publish(0)
                                rospy.sleep(.59)
                                self.busy = False
                                self.linefollow_pub.publish(int(self.state['speed'][0]))
                                '''
                                self.state['lane'] = LEADER_LANE
                        # Break if leader and too far ahead
                        elif abs(polar_angle - self.state['polar_angle']) > pi / 8:
                            ## Wait for robot behind
                            print("waiting for robot behind")
                            self.break_applied = .25
                        elif self.break_applied != 0:
                            self.break_applied = 0
                            self.state['speed'] = (40,40)
                elif(self.state['mode'] == LINE_FOLLOWING_MODE):
                    if self.state['platoon_pos'] > 1:
                        if abs(polar_r - self.state['polar_r']) < 35 and abs(polar_angle - self.state['polar_angle']) < pi/8:
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
                    self.linefollow_pub.publish(-1)
                self.busy = True
                #self.laneSwitch_pub.publish(1)
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
                self.busy = False
                self.state['lane'] = newLane
                if self.state['mode'] == LINE_FOLLOWING_MODE or self.state['mode'] == SIDE_FORMATION_MODE:
                    self.linefollow_pub.publish(50)
            elif self.state['lane'] > newLane:
                if self.state['mode'] == LINE_FOLLOWING_MODE or self.state['mode'] == SIDE_FORMATION_MODE:
                    self.linefollow_pub.publish(-1)
                self.busy = True
                #self.laneSwitch_pub.publish(0)
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
                self.busy = False
                self.state['lane'] = newLane
                if self.state['mode'] == LINE_FOLLOWING_MODE or self.state['mode'] == SIDE_FORMATION_MODE:
                    self.linefollow_pub.publish(50)
        elif(action_id == 'e'):
            print("setRole")
            self.state['role'] = msg
        elif(action_id == 'f'):
            print("setPosition")
            sender = int(msg)
            if self.busy is True:
                print("currently busy. cannot change leader")
                self.feedback_pub.publish("0,NOTOK")
            if sender == 0 and self.busy is False:
                self.busy = True
                commandMsg = "" + str(rospy.get_time()) + ",f,0,0,1,1"
                self.action_pub.publish(commandMsg)
            else:
                self.busy = True
                self.feedback_pub.publish("0,OK")
                if self.state['mode'] == LINE_FOLLOWING_MODE:
                    if(self.state['lane'] == 0):
                        self.linefollow_pub.publish(int(-1))
                        # stop
                        self.rightWheel_pub.publish(0)
                        self.leftWheel_pub.publish(0)
                        # switch left
                        self.rightWheel_pub.publish(80)
                        self.leftWheel_pub.publish(0)
                        rospy.sleep(.59)
                        self.rightWheel_pub.publish(80)
                        self.leftWheel_pub.publish(80)
                        rospy.sleep(1)
                        self.rightWheel_pub.publish(0)
                        self.leftWheel_pub.publish(80)
                        rospy.sleep(.59)

                        self.linefollow_pub.publish(60)
                        rospy.sleep(2)

                        self.linefollow_pub.publish(int(-1))
                        # stop
                        self.rightWheel_pub.publish(0)
                        self.leftWheel_pub.publish(0)
                        # switch left
                        self.rightWheel_pub.publish(0)
                        self.leftWheel_pub.publish(80)
                        rospy.sleep(.59)
                        self.rightWheel_pub.publish(80)
                        self.leftWheel_pub.publish(80)
                        rospy.sleep(1)
                        self.rightWheel_pub.publish(80)
                        self.leftWheel_pub.publish(0)
                        rospy.sleep(.59)
                    else:
                        self.linefollow_pub.publish(int(-1))
                        # stop
                        self.rightWheel_pub.publish(0)
                        self.leftWheel_pub.publish(0)
                        # switch left
                        self.rightWheel_pub.publish(0)
                        self.leftWheel_pub.publish(80)
                        rospy.sleep(.59)
                        self.rightWheel_pub.publish(80)
                        self.leftWheel_pub.publish(80)
                        rospy.sleep(1)
                        self.rightWheel_pub.publish(80)
                        self.leftWheel_pub.publish(0)
                        rospy.sleep(.59)

                        self.linefollow_pub.publish(60)
                        rospy.sleep(2)

                        self.linefollow_pub.publish(int(-1))
                        # stop
                        self.rightWheel_pub.publish(0)
                        self.leftWheel_pub.publish(0)
                        # switch left
                        self.rightWheel_pub.publish(80)
                        self.leftWheel_pub.publish(0)
                        rospy.sleep(.59)
                        self.rightWheel_pub.publish(80)
                        self.leftWheel_pub.publish(80)
                        rospy.sleep(1)
                        self.rightWheel_pub.publish(0)
                        self.leftWheel_pub.publish(80)
                        rospy.sleep(.59)
                self.platoons['leader'] = 1
                self.state['platoon_pos'] = 1
                self.feedback_pub.publish("0,DONE")
                self.busy = False
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
            if self.busy:
                print("currently busy. cannot set mode")
            else:
                payload_params = msg
                newMode = int(msg)
                self.state['mode'] = newMode
                if newMode == LINE_FOLLOWING_MODE or newMode == SIDE_FORMATION_MODE:
                    self.linefollow_pub.publish(50)
                    self.state['speed'] = (50,50)
                elif newMode == LISTENING_MODE  or newMode == SHRIMP_FOLLOWING_MODE:
                    self.linefollow_pub.publish(int(-1))
                    self.rightWheel_pub.publish(0)
                    self.leftWheel_pub.publish(0)
                else:
                    self.linefollow_pub.publish(int(-1))
                    self.rightWheel_pub.publish(0)
                    self.leftWheel_pub.publish(0)
        elif(action_id == 'i'):
            print("turnAndTravel")
            self.busy = True
            self.rightWheel_pub.publish(-80)
            self.leftWheel_pub.publish(80)
            rospy.sleep(1)
            self.rightWheel_pub.publish(0)
            self.leftWheel_pub.publish(0)
            self.busy = False
        elif(action_id == 'j'):
            print("intersection")
            if self.state['mode'] == LINE_FOLLOWING_MODE:
                self.busy = True
                #self.linefollow_pub.publish(int(-1))
                #if self.state['lane'] > 0:
                    # stop
                    #self.rightWheel_pub.publish(0)
                    #self.leftWheel_pub.publish(0)
                    # switch right
                    #self.rightWheel_pub.publish(0)
                    #self.leftWheel_pub.publish(80)
                    #rospy.sleep(.59)
                    #self.rightWheel_pub.publish(80)
                    #self.leftWheel_pub.publish(80)
                    #rospy.sleep(1)
                    #self.rightWheel_pub.publish(80)
                    #self.leftWheel_pub.publish(0)
                    #rospy.sleep(.59)
                    #self.state['lane'] = 0
                #while (float(self.state['y']) < float(yrange_min2)) or (float(self.state['y']) > float(yrange_max1)):
                    #self.linefollow_pub.publish(50)
                self.linefollow_pub.publish(int(-1))
                self.rightWheel_pub.publish(-80)
                self.leftWheel_pub.publish(80)
                rospy.sleep(1)
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(0)
                self.rightWheel_pub.publish(80)
                self.leftWheel_pub.publish(80)
                rospy.sleep(.55)
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(0)
                rospy.sleep(0.5)
                self.linefollow_pub.publish(50)
                rospy.sleep(10)
                # check for collisions (optional)
                self.linefollow_pub.publish(-1)
                self.rightWheel_pub.publish(80)
                self.leftWheel_pub.publish(80)
                rospy.sleep(.55)
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(0)
                self.rightWheel_pub.publish(-80)
                self.leftWheel_pub.publish(80)
                rospy.sleep(1)
                self.rightWheel_pub.publish(0)
                self.leftWheel_pub.publish(0)
                self.busy = False
            else:
                rospy.loginfo("intersection only allowed in linefollowing mode")
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
        params = data.data.split(',')
        rospy.loginfo(rospy.get_caller_id() + 'feedback %s', data.data)
        if int(params[0]) == 0:
            if params[1] == "NOTOK":
                print("could not change other robot! leader remains")
            elif params[1] == "OK" and self.state['mode'] == LINE_FOLLOWING_MODE:
                self.linefollow_pub.publish(-1)
            elif params[1] == "DONE":
                self.platoons['leader'] = 0
                self.state['platoon_pos'] = 2
                self.busy = False

    def shrimp_cb(self, data):
        print(data)
        params = data.data.split(',')
        timestamp = params[0]
        x = float(params[1])
        y = float(params[2])
        if self.state['mode'] == SHRIMP_FOLLOWING_MODE:
            dx = x - float(self.state['x'])
            dy = y - float(self.state['y'])
            #if abs(dx) < 5 and abs(dy) < 5: #Changed or to and
             #   return
            dist = sqrt(pow(dx,2)+pow(dy,2))
            direction = atan2(dy,dx)
            print("")
            print("         distance: %f" %float(dist))
            print("        direction: %f" %float(direction))
            robot_angle = math.degrees(float(self.state['theta']))
            difference = (robot_angle - math.degrees(float(direction)))
            print("Direction: "+str(math.degrees(float(direction)))+' '+"Theta: "+str(math.degrees(self.state['theta'])))
            print("difference: "+str(difference))
            
            
            print("This is dist: "+str(dist))
            if abs(difference) > 10.0 and dist > 30 :
                if difference < 0:
                    #turn right
                    self.leftWheel_pub.publish(-38)
                    self.rightWheel_pub.publish(38)
                    rospy.sleep(.5)
                    self.leftWheel_pub.publish(0)
                    self.rightWheel_pub.publish(0)
                elif(difference > 0):
                    #turn left
                    self.leftWheel_pub.publish(38)
                    self.rightWheel_pub.publish(-38)
                    rospy.sleep(.5)
                    self.leftWheel_pub.publish(0)
                    self.rightWheel_pub.publish(0)
            elif dist > 30:
                #move straight
                self.leftWheel_pub.publish(60)
                self.rightWheel_pub.publish(60)
            else:
                print("Close enough to correct position")
                self.leftWheel_pub.publish(0)
                self.rightWheel_pub.publish(0)
                self.busy = False


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
    robot = 'bigboy'
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
    ctrl = Controller(state=bigboy_state, platoons=platoons, robot=robot)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not ctrl.busy:
            if (ctrl.state['mode'] == LINE_FOLLOWING_MODE or ctrl.state['mode'] == SIDE_FORMATION_MODE) and ctrl.busy == False:
                ctrl.linefollow_pub.publish(int(ctrl.state['speed'][0]))
            elif ctrl.state['mode'] == SHRIMP_FOLLOWING_MODE:
                pass
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
