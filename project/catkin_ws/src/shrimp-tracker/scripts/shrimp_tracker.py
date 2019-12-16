
# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import rospy
import cv2
import imutils
import time

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())
# define the lower and upper boundaries of the "red"
# ball in the HSV color space, then initialize the
# list of tracked points

#Blue Shrimp lower(22,20,0) upper(74,195,190)
#Red Shrimp lower(0,29,0) upper(12,255,200)

redLower = (0, 21, 75)
redUpper = (20,150 ,220 )
blueLower = (22,20,0)
blueUpper = (74,195,190)

pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	vs = VideoStream(src=0).start()

# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

#To resize frame width, height
frameWidth = 560
frameHeight = 750
resizedFrame = (frameWidth,frameHeight)


rospy.init_node('tracker', anonymous=True)
pub = rospy.Publisher('bigboy_shrimp', String, queue_size = 10)
rate = rospy.Rate(10) # 10hz



# keep looping
while not rospy.is_shutdown():
#while True:
	# grab the current frame
	frame = vs.read()

	# handle the frame from VideoCapture or VideoStream
	frame = frame[1] if args.get("video", False) else frame

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if frame is None:
		break

	#Rotate and resize to match video to map
	frame = cv2.rotate(frame, rotateCode=cv2.ROTATE_90_COUNTERCLOCKWISE)
	frame = cv2.resize(frame, resizedFrame)

	# resize the frame, blur it, and convert it to the HSV
	# color space
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "red", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, redLower, redUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

'''	#same construct using for the color "blue" instead
	mask2 = cv2.inRange(hsv, blueLower, blueUpper)
	mask2 = cv2.erode(mask2, None, iterations=2)
	mask2 = cv2.dilate(mask2, None, iterations=2)
'''

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None
'''
	# Same thing for blue as for red

	cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts2 = imutils.grab_contours(cnts2)
	center2 = None'''




	# only proceed if at least one contour was found
	if len(cnts) > 0''' or len(cnts2) > 0''':
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		#These values will be sent to controller instead of saved to file
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		centroid_x = center[0]
		centroid_y = abs(center[1]-frameHeight)

	'''	#Second time for second contour
		c2 = max(cnts2, key=cv2.contourArea)
		((x2, y2), radius2) = cv2.minEnclosingCircle(c2)
		M2 = cv2.moments(c2)
		#These values will be sent to controller instead of saved to file
		center2 = (int(M2["m10"] / M2["m00"]), int(M2["m01"] / M2["m00"]))
		centroid_x2 = center2[0]
		centroid_y2 = abs(center2[1]-frameHeight)'''



		#Send timestamp,x,y to controller
		pos_str = str(rospy.get_time())+','+str(centroid_x)+','+str(centroid_y)
		pub.publish(pos_str)

		'''pos_str2 ='Blue: ' + str(rospy.get_time())+','+str(centroid_x2)+','+str(centroid_y2)
		pub.publish(pos_str2)'''


		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (255, 0, 0), -1)

		'''if radius2 > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x2), int(y2)), int(radius2),
				(0, 255, 255), 2)
			cv2.circle(frame, center2, 5, (255, 0, 0), -1)'''

	# update the points queue
	pts.appendleft(center)
	#pts.appendleft(center2)
		# loop over the set of tracked points
	for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
	#rate.sleep()
# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()

# otherwise, release the camera
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()
