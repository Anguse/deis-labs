#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('teambeku', String, queue_size=10)
    rospy.init_node('g3_talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
	
	hello_str = "60 33 {} 1 {} 1 1 1 62"
	cmd = raw_input("command: ")
	cmd = cmd.split(" ")
	hello_str = hello_str.format(cmd[0], cmd[0])
    	rospy.loginfo('(Connected: '+ str(pub.get_num_connections()) +' ) '+hello_str)    
	pub.publish(hello_str)
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
