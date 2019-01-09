# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.depth_callback)
    self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info",CameraInfo,self.camera_info_callback)

    self.image_pub = rospy.Publisher("/detect_result",Image)
    self.detect_3d_point_pub = rospy.Publisher("/detect_3d_point", Pose, queue_size=50)
    

  def callback(self,data):
  	try:
  		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
  	except CvBridgeError as e:
  		print(e)

  	redLower = (160, 20, 70)
  	redUpper = (190, 255, 255)
  	#pts = deque(maxlen=args["buffer"])
  	self.pts=deque([], maxlen=64)

  	#print "image height: ", data.height
  	#print "image width: ", data.width

    # resize the frame, blur it, and convert it to the HSV
	# color space
	cv_image = imutils.resize(cv_image, width=600)
	blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "red", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, redLower, redUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(cv_image, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(cv_image, center, 5, (0, 0, 255), -1)

	# update the points queue
	self.pts.appendleft(center)
	
	# loop over the set of tracked points
	for i in range(1, len(self.pts)):
		# if either of the tracked points are None, ignore
		# them
		if self.pts[i - 1] is None or self.pts[i] is None:
			continue

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv2.line(cv_image, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

	# show the frame to our screen
	cv2.imshow("Image window", cv_image)
	key = cv2.waitKey(1) & 0xFF

	try:
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		self.detect_3d_point_pub.publish(self.ball_pos)
	except CvBridgeError as e:
		print(e)

  def depth_callback(self, msg):
  	try:
  		depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
  	except CvBridgeError as e:
  		print(e)

  	#print "depth image height: ", msg.height
  	#print "depth image width: ", msg.width
  	
	# Convert the depth image to a Numpy array since most cv2 functions require Numpy arrays.
	depth_image_array = np.array(depth_image, dtype = np.dtype('f8'))
	# Normalize the depth image to fall between 0 (black) and 1 (white)
	# http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125
	depth_image_norm = cv2.normalize(depth_image_array, depth_image_array, 0, 1, cv2.NORM_MINMAX)
	# Resize to the desired size
	depth_image = cv2.resize(depth_image_norm, (640,480), interpolation = cv2.INTER_CUBIC)



  	#depth_image = imutils.resize(depth_image, width=480)
  	#print depth_image

  	if (self.pts):
  		self.depth=depth_image[self.pts[0]]*255
  		print "depth value=",self.depth
  	#print depth_image[self.pts[0]]

  def camera_info_callback(self, data):

  	#print data.K
  	if (self.pts):
  		x = (float("%s"%self.pts[0][0])-data.K[2])/data.K[0]
  		y = (float("%s"%self.pts[0][1])-data.K[5])/data.K[4]
  	#y=(self.pts[1]-data.K[5])/data.K[4]
  	
  	    # Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera */

#		static void rs2_deproject_pixel_to_point(float point[3], const struct rs2_intrinsics * intrin, const float pixel[2], float depth)
#	{
#	    assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
#	    assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
#	    //assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model
#
#	    float x = (self.pts[0] - intrin->ppx) / intrin->fx;
#	    float y = (self..pts[1] - intrin->ppy) / intrin->fy;
#	    if(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
#	    {
#	        float r2  = x*x + y*y;
#	        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
#	        float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
#	        float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
#	        x = ux;
#	        y = uy;
#	    }

		self.ball_pos = Pose()
		self.ball_pos.position.x = self.depth * x;
		self.ball_pos.position.y = self.depth * y;
		self.ball_pos.position.z = self.depth;



		#self.detect_3d_point_pub.publish(self.ball_pos)
#	}


def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
