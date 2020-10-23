#!/usr/bin/env python

#This program is to be used with the turtlebot3. It's purpose is to use OpenCV to analyse
#the colous of the line that we want to follow and perform image processing so that the centroid of the line
#can be detected, which will the be used to move the TB3 in another code. This code is not concerened with
#moving the turtlebot, just identifying the line.

import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

ctrl_c = False

class LineFollower:

        def __init__(self):

                self.bridge_object = CvBridge()
                #Subscribe to image topic
		self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, self.camera_callback)
              

        def clean_up(self):
                #this function runs upon shutdown.
                cv2.destroyAllWindows()
                rospy.loginfo("Unregistering Camera Subscriber")
                self.image_sub.unregister()
                rospy.loginfo("Wait 1 second")
                rospy.sleep(1)
      

        def camera_callback(self, data):
                
                try:
                        #convert the image to cv_image format
			cv_image = self.bridge_object.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
                except CvBridgeError as e:
                        print(e)

                #### 1.GET IMAGE INFO AND CROP ####
                #get info about shape of captured image
                height, width, channels = cv_image.shape
                #print (cv_image.shape)
                #Crop image to only see 100 rows
                descentre = 160
                rows_to_watch = 100
                crop_img = cv_image[(height)/2+descentre: (height)/2+(descentre+rows_to_watch)][1:width]
                #print (crop_img.shape)

                #### 2.CONVERT FROM BGR TO HSV ####
		#convert cropped image from RGB to HSV
                hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
                
                #This section is used to determine the hsv values that we shoule be looking for.
                #First, we inspect the 'cropped image' which will be in RGB format. We get these
                #3 values, than manually rearrange them into BGR format. We then enter the 3 BGR
                #values in the array below.
                line_colour = np.uint8([[[58,180,195]]])
                #The next line will take the array of BGR values and convert them to HSV values.
                hsv_line_colour = cv2.cvtColor(line_colour, cv2.COLOR_BGR2HSV)
                #We print the HSV values to screen.
                print (hsv_line_colour)
                #We then HSV values we read on the screen, and create a upper and lower boundary for
                #these values. These values will be used in our mask, and any colour that falls in this
                #range will be identified.
                lower_yellow = np.array([15, 129, 145])
                upper_yellow = np.array([35, 229, 245])

                #### 3.APPLY THE MASK ####
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                res = cv2.bitwise_and(crop_img,crop_img,mask= mask)

                #### 4.GET THE CENTROIDS, DRAW A CIRCLE ####
                m = cv2.moments(mask, False)
                try:
                        cx = m['m10']/m['m00']
                        cy = m['m01']/m['m00']
                except ZeroDivisionError:
                        cx = width/2
                        cy = height/2
                #Draw circle on image to indicate detected centroid
                cv2.circle(res, (int(cx), int(cy)), 10,(0,0,255), -1)

                #Open a GUI, where you can see the contents of each image
                cv2.imshow("Original Image", cv_image)
                cv2.imshow("Cropped Image", crop_img)
                cv2.imshow("HSV Image", hsv)
                cv2.imshow("Mask", res)
                cv2.waitKey(1)

def main():

        global ctrl_c
        rospy.init_node('line_following_node', anonymous=True)
        line_follower_object = LineFollower()

        rate = rospy.Rate(5)
        ctrl_c = False

        def shutdownhook():
                global ctrl_c
                rospy.loginfo("Initiating ShutDown")
                line_follower_object.clean_up()                
                rospy.loginfo("ShutdownTime!")
                ctrl_c = True
        
        rospy.on_shutdown(shutdownhook)

        while not ctrl_c:
                rate.sleep

if __name__ == '__main__':
        main()

