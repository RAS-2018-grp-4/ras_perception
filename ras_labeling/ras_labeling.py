import sys
import argparse

import cv2
print(cv2.__version__)

def extractImages():
    count = 0
    vidcap = cv2.VideoCapture('vid1.mp4')
    success,image = vidcap.read()
    success = True
    while success:
      vidcap.set(cv2.CAP_PROP_POS_MSEC,(count*1000))    # added this line 
      success,image = vidcap.read()
      print ('Read a new frame: ', success)
      cv2.imwrite( "./TEMP_Folder/" + str(count+1000) + ".jpg", image)     # save frame as JPEG file
      count = count + 1

if __name__=="__main__":
    #print("aba")

    extractImages()