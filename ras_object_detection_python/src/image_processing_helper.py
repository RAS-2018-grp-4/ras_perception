#!/usr/bin/env python
'''
Author:     Ajinkya and David
Date:       2018/11/25
Description: This script contains helper functions for classical image analysis.
Examples are:
    - Color thresholding
    - Morphological opening
    - Gaussian Blurring
NOTE: The HSV parameters need to be tuned for a new environment.
use "set_live_hsv_params.py" for this.

COLOR LABELS: they are defined according to CNN's training
------------
0:  Yellow
1:  Green
2:  Orange
3:  Red
4:  Blue
5:  Purple
6:  Nothing
and...
7:  Black (for batteries)

'''
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import random
import cv2

def threshold_hsv(image, color_label):
    # get hsv image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    if color_label == 0:    #YELLOW
        mask = cv2.inRange(hsv, np.array([0,1102,138]), np.array([179,255,255]))
    elif color_label == 1:  #GREEN
        mask = cv2.inRange(hsv, np.array([32,46,0]), np.array([76,255,255]))
    elif color_label == 2:  #ORANGE
        # mask1 = cv2.inRange(hsv, np.array([5,150,150]), np.array([15,255,255]))
        # mask2 = cv2.inRange(hsv, np.array([160,220,150]), np.array([169,255,255]))
        # mask = mask1 + mask2 
        mask = cv2.inRange(hsv, np.array([8,0,0]), np.array([20,255,255]))
    elif color_label == 3:  #RED
        mask1 = cv2.inRange(hsv, np.array([0,0,0]), np.array([15,255,255]))
        mask2 = cv2.inRange(hsv, np.array([170,0,0]), np.array([179,255,255]))
        mask = mask1 + mask2 
        #mask = cv2.inRange(hsv, np.array([0,150,0]), np.array([4,255,200]))
    elif color_label == 4:  #BLUE
        mask = cv2.inRange(hsv, np.array([70,0,0]), np.array([170,255,255]))
    elif color_label == 5:  #PURPLE
        mask = cv2.inRange(hsv, np.array([26,0,63]), np.array([179,163,154]))
    elif color_label == 7:  #BLACK
        mask = cv2.inRange(hsv, np.array([0,0,0]), np.array([179,255,50]))

    return mask

def morphOpen(image):
    # define structuring element
    # take 5-10% of least dimension of image as kernel size.
    # Test for kernel size and shape
    kernel_size = min(5, int(min(image.shape[0],image.shape[1])*0.05))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
    #kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    return opening

