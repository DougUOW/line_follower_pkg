#!/usr/bin/env python

#Description.
from __future__ import division
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


def region_of_interest(img, vertices):
	mask = np.zeros_like(img)
	match_mask_color = 255
	cv2.fillPoly(mask, vertices, match_mask_color)
	masked_image = cv2.bitwise_and(img, mask)
	return masked_image

def draw_lines(img, lines):#, colour=[0,0,255], thickness=3):
	colour=[0,0,255]
	thickness=3
	img = np.copy(img)
	#Create a blank image that matches size of original
	blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

	for line in lines:
		for x1, y1, x2, y2 in line:
			print("line in function ", line)
			cv2.line(blank_image, (x1,y1), (x2,y2), colour, thickness)
	#Merge the image with the lines onto the original image
	img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
	return img

def analyse_slope(lines, image):

		#This hole next chunk of code is used to determine the slope of the line. If the slope is too small,
		#  (<0.5),we assume it is not a lane and remove ot from our analysis
		#This should be in a function.
		left_line_x = []
		left_line_y = []
		right_line_x = []
		right_line_y = []

		for line in lines:
			for x1, y1, x2, y2 in line:
				slope = (y2 - y1) / (x2 - x1) # <-- Calculating the slope.
				if math.fabs(slope) > 0.5: # <-- Only consider extreme slope
					if slope <= 0: # <-- If the slope is negative, left group.
						left_line_x.extend([x1, x2])
						left_line_y.extend([y1, y2])
					else: # <-- Otherwise, right group.
						right_line_x.extend([x1, x2])
						right_line_y.extend([y1, y2])
		
		min_y = int(image.shape[0] * (3 / 5)) # <-- Just below the horizon
		max_y = image.shape[0] # <-- The bottom of the image
		
		poly_left = np.poly1d(np.polyfit(
    		left_line_y,
    		left_line_x,
    		deg=1
		))
		
		left_x_start = int(poly_left(max_y))
		left_x_end = int(poly_left(min_y))
		
		poly_right = np.poly1d(np.polyfit(
    		right_line_y,
    		right_line_x,
    		deg=1
		))
		
		right_x_start = int(poly_right(max_y))
		right_x_end = int(poly_right(min_y))	

		return([[[left_x_start, max_y, left_x_end, min_y],[right_x_start, max_y, right_x_end, min_y],]])

class LoadImage:

    def __init__(self):

	   	image_path = '/home/doug/catkin_ws/src/line_follower_pkg/images/solidWhiteCurve.jpg'
		image = cv2.imread(image_path,cv2.COLOR_BGR2RGB)
		gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
		
		height = gray_image.shape[0]
		width = gray_image.shape[1]
		region_of_interest_vertices = [(0, height), (width/2, height/2), (width, height)]
		
		canny_image = cv2.Canny(gray_image, 200, 300)
		cropped_image = region_of_interest(canny_image, np.array([region_of_interest_vertices], np.int32),)

		lines = cv2.HoughLinesP(cropped_image,
                        rho=6,
                        theta=np.pi/60,
                        threshold=160,
                        lines=np.array([]),
                        minLineLength=40,
                        maxLineGap=25)

		lines_sloped = analyse_slope(lines, image)

		image_with_lines = draw_lines(image, lines_sloped)

		#Display images on screen	
		cv2.imshow('Original Image', image)	
		cv2.imshow('Original Gray Image', gray_image)
		cv2.imshow('Canny Edge Detection', canny_image)
		cv2.imshow('Cropped Image', cropped_image)
		cv2.imshow('image_with_lines', image_with_lines)
		cv2.waitKey(0)

def main():

	load_image_object = LoadImage()
	rospy.init_node('load_image_node', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
        main()



