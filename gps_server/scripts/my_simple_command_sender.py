#!/usr/bin/env python
import rospy
import numpy as np
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
	rospy.init_node('commandSender', anonymous=True)

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
			commandMsg = ""+ str(rospy.get_time()) + ",g,-1,-1,-2,60;60" #set speed, from GPS, to robots with no platoon, all robots, speed should be 255 left wheel, 255 right wheel
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 'b'):
			print "Pressed b: Back"
			commandMsg = ""+ str(rospy.get_time()) + ",g,-1,-1,-2,-60;-60" #set speed, from GPS, to robots with no platoon, all robots, speed should be -255 left wheel, -255 right wheel
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 'l'):
			print "Pressed l: Turn left"
			commandMsg = ""+ str(rospy.get_time()) + ",g,-1,-1,-2,0;60" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 255 right wheel
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 'r'):
			print "Pressed r: Turn right"
			commandMsg = ""+ str(rospy.get_time()) + ",g,-1,-1,-2,60;0" #set speed, from GPS, to robots with no platoon, all robots, speed should be 0 left wheel, 255 right wheel
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== 's'):
			print "Pressed s: Stop"
			commandMsg = ""+ str(rospy.get_time()) + ",g,-1,-1,-2,0;0" #set speed, from GPS, to robots with no platoon, all robots, speed should be 255 left wheel, 255 right wheel
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== '0'):
			print "Pressed 0: mode 0"
			commandMsg = ""+ str(rospy.get_time()) + ",h,-1,-1,-2,0"
			myCommandSender.pubAction.publish(commandMsg)
			print "sent command: ", commandMsg
		elif(key== '1'):
			print "Pressed 1: mode 1"
			commandMsg = ""+ str(rospy.get_time()) + ",h,-1,-1,-2,1"
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



