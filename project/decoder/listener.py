#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
# use serial temporary, before controller is implemented
import serial

from std_msgs.msg import String

arduinoData = serial.Serial('/dev/ttyUSB0', 9600)
mode = 0 # single vehicle mode
#mode = 1 # head convoy mode
#mode = 2 # head convoy mode


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
def gps_cb(data):
	states = data.data.split(';')
	state = states[0].replace('[', '')
	params = state.split(' ')
	x = params[0]
	y = params[1]
	z = params[2]
	ID = params[3]
	theta = params[4]
	
	print("")
	print("              x: %s" %x)
	print("              y: %s" %y)
	print("              z: %s" %z)
	print("             ID: %s" %ID)
	print("          theta: %s" %theta)
	print("")



# Example outputs
#[INFO] [1455208221.162028]: /listener_1451_1455208187311action 1571383446.15,g,-1,-1,-2,255;255
def action_cb(data):
	params = data.data.split(',')	
	timestamp = params[0]
	action_id = params[1]
	source_robot_id = int(params[2])
	target_platoon_id = int(params[3])
	target_robot_id = int(params[4])
	msg = params[5]

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
	elif(action_id == 'b'):
		print("setID")
	elif(action_id == 'c'):
		print("setType")	
	elif(action_id == 'd'):
		print("setLane")
	elif(action_id == 'e'):
		print("setRole")
	elif(action_id == 'f'):
		print("setPosition")
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
		arduinoData.write(action_id+chr(leftWheelSpeed)+chr(rightWheelSpeed)+endMarker)
	elif(action_id == 'h'):
		print("setMode")
                payload_params = msg
                newMode = int(msg[0])
                endMarker = '\n'
                arduinoData.write(chr(action_id)+chr(newMode)+endMarker)
                mode = newMode
	elif(action_id == 'i'):
		print("free")
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

def heartbeat_cb(data):
    rospy.loginfo(rospy.get_caller_id() + 'heartbeat %s', data.data)

def feedback_cb(data):
    rospy.loginfo(rospy.get_caller_id() + 'feedback %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('josefoutput', String, gps_cb)
    rospy.Subscriber('action', String, action_cb)
    rospy.Subscriber('heartbeat', String, heartbeat_cb)
    rospy.Subscriber('feedback', String, feedback_cb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
