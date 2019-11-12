#!/usr/bin/env python

## This script visualizes tha robots position in a map
## It reads values published by the logger

#import rospy
#from std_msgs.msg import String
import matplotlib.pyplot as plt

data = [
    [0,0,0,0,0,1,1,1,1,0],
    [0,0,0,0,0,1,0,0,1,0],
    [0,0,1,0,1,0,1,1,0,0],
    [0,0,1,0,0,1,1,0,1,0],
    [0,0,1,0,1,0,0,1,1,0],
    [1,0,0,1,0,1,0,0,1,0],
    [0,1,0,0,0,1,1,1,1,1],
    [0,1,0,0,0,0,1,1,1,1],
    [1,0,0,0,1,1,1,0,1,0],
    [1,1,1,1,0,0,0,1,1,0]
]


if __name__ == "__main__":
    plt.imshow(data)
    plt.show()