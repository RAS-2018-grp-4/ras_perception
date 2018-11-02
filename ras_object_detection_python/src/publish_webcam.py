import numpy as np 
import cv2
import rospy 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('publish_webcam', anonymous=True)

cap = cv2.VideoCapture(0)

capturedImage = rospy.Publisher("/webcam_image", Image, queue_size=10)

r = rospy.Rate(20) # Hz 

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',frame)

    capturedImage.publish(CvBridge().cv2_to_imgmsg(frame))
    
    r.sleep()
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()