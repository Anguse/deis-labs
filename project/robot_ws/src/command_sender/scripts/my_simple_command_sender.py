#!/usr/bin/env python
import rospy
from subprocess import call
import sys
from std_msgs.msg import String
import os

class commandSender:
	def __init__(self):

		self.pubAction = rospy.Publisher('action', String, queue_size=10)
		self.r=rospy.Rate(10)
		self.r.sleep()
	

def main():
	rospy.init_node('tinyboy_commandsender', anonymous=True)
	target_robot_id = 1
	target_robot_platoon_id = 0
	source_robot_id = 0
	speed = 80
	myCommandSender = commandSender()

	while True:

		myCommandSender.r.sleep()
		key= raw_input()
		print key
		
		if(key== ord("q")): 
			print "Pressed q"
			print "Quit"
			break
		elif(key== 'f'):
			print "Pressed f: Go forward"
			commandMsg = ""+ str(rospy.get_time()) + ",g,"+str(source_robot_id)+","+str(target_robot_platoon_id)+","+str(target_robot_id)+",%d;%d"%(speed,speed) #set speed, from GPS, to robots with no platoon, all robots, speed should be 255 left wheel, 255 right wheel
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 'b'):
			print "Pressed b: Back"
			commandMsg = ""+ str(rospy.get_time()) + ",g,"+str(source_robot_id)+","+str(target_robot_platoon_id)+","+str(target_robot_id)+",%d;%d"%(-speed,-speed) #set speed, from GPS, to robots with no platoon, all robots, speed should be -255 left wheel, -255 right wheel
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 'l'):
			print "Pressed l: Turn left"
			commandMsg = ""+ str(rospy.get_time()) + ",g,"+str(source_robot_id)+","+str(target_robot_platoon_id)+","+str(target_robot_id)+",%d;%d"%(0,speed) #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 255 right wheel
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 'r'):
			print "Pressed r: Turn right"
			commandMsg = ""+ str(rospy.get_time()) + ",g,"+str(source_robot_id)+","+str(target_robot_platoon_id)+","+str(target_robot_id)+",%d;%d"%(speed,0) #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 255 right wheel
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 's'):
			print "Pressed s: Stop"
			commandMsg = ""+ str(rospy.get_time()) + ",g,"+str(source_robot_id)+","+str(target_robot_platoon_id)+","+str(target_robot_id)+",0;0" #set speed, from GPS, to robots with no platoon, all robots, speed should be 255 left wheel, 255 right wheel
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== '0'):
			print "Pressed 0: mode 0(LINE_FOLLOWING_MODE)"
			commandMsg = ""+ str(rospy.get_time()) + ",h,"+str(source_robot_id)+","+str(target_robot_platoon_id)+","+str(target_robot_id)+",0"
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== '1'):
			print "Pressed 1: mode 1(LISTENING_MODE)"
			commandMsg = ""+ str(rospy.get_time()) + ",h,"+str(source_robot_id)+","+str(target_robot_platoon_id)+","+str(target_robot_id)+",1"
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== '2'):
			print "Pressed 2: mode 2(SHRIMP_FOLLOWING_MODE)"
			commandMsg = ""+ str(rospy.get_time()) + ",h,"+str(source_robot_id)+","+str(target_robot_platoon_id)+","+str(target_robot_id)+",2"
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== '3'):
			print "Pressed 3: mode 3(SIDE_FORMATION_MODE)"
			commandMsg = ""+ str(rospy.get_time()) + ",h,"+str(source_robot_id)+","+str(target_robot_platoon_id)+","+str(target_robot_id)+",3"
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg

		elif(key== 'i'):
			# Turn and travel
			print "Pressed i: turn and travel"
			msg = ""
			commandMsg = ""+ str(rospy.get_time()) + ',' +key + ',' + str(source_robot_id) + ',' + str(target_robot_platoon_id) + ',' + str(target_robot_id) + ',' + str(msg)
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 'd0'):
			# Set lane 0
			print "Pressed j: set lane"
			msg = "0"
			commandMsg = ""+ str(rospy.get_time()) + ',d' + ',' + str(source_robot_id) + ',' + str(target_robot_platoon_id) + ',' + str(target_robot_id) + ',' + str(msg)
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 'd1'):
			# Set lane 1
			print "Pressed j: set lane"
			msg = "1"
			commandMsg = ""+ str(rospy.get_time()) + ',d' + ',' + str(source_robot_id) + ',' + str(target_robot_platoon_id) + ',' + str(target_robot_id) + ',' + str(msg)
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 'k'):
			# Intersection
			print "Pressed k: intersection"
			msg = ""
			commandMsg = ""+ str(rospy.get_time()) + ',' + key + ',' + str(source_robot_id) + ',' + str(target_robot_platoon_id) + ',' + str(target_robot_id) + ',' + msg
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key=='w'):
			print "Pressed w: change of leader"
			commandMsg = ""+ str(rospy.get_time()) + ",f,"+str(source_robot_id)+","+str(target_robot_platoon_id)+","+str(target_robot_id)+",0"
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg




if __name__== '__main__':

    	print '-------------------------------------'
    	print '-            TOLLGATE 2             -'
    	print '-   OCT 2019, HH, Martin            -'
    	print '-------------------------------------'

	try:
		main()
	except rospy.ROSInterruptException:
		pass
	finally:
		pass



