#!/usr/bin/env python
import cv2
import numpy as np 
import keras.models 
import glob
from datetime import datetime
from keras.models import model_from_json
from image_processing_helper import *

# imports for ROS integration
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import copy
from object_detection_test.msg import objects_found
from geometry_msgs.msg import PointStamped

'''
@author:Ajinkya Khoche

model_shape: A CNN which detects shape
-----------
Class Labels:   SHAPE
                -----
0:  Ball
1:  Cube
2:  Cylinder
3:  Hollow Cube
4:  Cross
5:  Triangle
6:  Star
7:  Nothing
8:  Obstacle
########################################

model_color: A CNN which detects color
-----------
Class Labels:   COLOR
                ------
0:  Yellow
1:  Green
2:  Orange
3:  Red
4:  Blue
5:  Purple
6:  Nothing
'''
#json_file = open('../../DL_training/KERAS_model/saved_models/cropped_shape_1.json', 'r')
json_file = open('/home/ras14/catkin_ws/src/ras_perception/DL_training/KERAS_model/saved_models/cropped_shape_1.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
model_shape = model_from_json(loaded_model_json)
# model_shape.load_weights('../../DL_training/KERAS_model/saved_models/cropped_shape_1.h5')
model_shape.load_weights('/home/ras14/catkin_ws/src/ras_perception/DL_training/KERAS_model/saved_models/cropped_shape_1.h5')
#model_shape = keras.models.load_model('./saved_models/keras_RAS_model_shape_1.h5')
# model_color = keras.models.load_model('../../DL_training/KERAS_model//saved_models/keras_cropped_color_1.h5')
model_color = keras.models.load_model('/home/ras14/catkin_ws/src/ras_perception/DL_training/KERAS_model/saved_models/keras_cropped_color_1.h5')

shape_class = ['Ball', 'Cube', 'Cylinder', 'Hollow Cube', 'Cross', 'Triangle', 'Star', 'Nothing' ]
color_class = ['Yellow', 'Green', 'Orange', 'Red', 'Blue', 'Purple', 'Nothing']

VIDEO_INFERENCE = 0
IMG_INFERNECE = 0

N_SHAPES = 7
N_COLORS = 6

DEBUG = 0

std = 58.363087
mean = 85.69786

expansion_param = 0.25

#Intrinsic camera parameters
cy = 352.49
fy = 672.55
cx = 635.18
fx = 672.55


def get_depth(bBox):
    x = bBox[0]
    y = bBox[1]
    w = bBox[2]
    h = bBox[3]
    # Taking 10 pixel square
    square_size = 5
    depth_square = lastFrame.depth[int(-square_size+round(y+h/2)):int(square_size+round(y+h/2)),int(-square_size+round(x+w/2)):int(square_size+round(x+w/2))]
    depth_square = copy.copy(depth_square)
    bad_I = np.argwhere(np.isnan(depth_square))
    depth_square[bad_I[:,0],bad_I[:,1]] = 0
    bad_I = np.argwhere(np.isinf(depth_square))
    depth_square[bad_I[:,0],bad_I[:,1]] = 0

    valid_index = np.nonzero(depth_square)
    if valid_index[0].shape[0] == 0 or valid_index[1].shape[0] == 0:
        return
    
    # #[Ajinkya]: z = z0 * cos(beta) , where z0 depth in 3D
    # # beta = arcsin(h/z). this is because camera is at a height h. depth z0 calculated is in 3D
    # # but we project everything in 2D. 
    # z0 = np.mean(depth_square[valid_index[0], valid_index[1]]) 
    # beta = np.arcsin(cam_mount_height / z0)
    # z = z0 * beta

    z = np.mean(depth_square[valid_index[0], valid_index[1]]) 
    z = -z
    #print(z)
        
    # use intrinsic camera parameters to convert from pixel coordinate to 
    # world coordinate (http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html)
    try:
        y_w = (round(y+h/2) - cy) / fy * z
        x_w = (round(x+w/2) - cx) / fx * z
    except:
        pass
        #continue
    return x_w, y_w, z



def detect_object(image, color_label):
    # variables to be returned
    x_w = None
    y_w = None
    z_w = None
    pred_shape_label = None
    pred_color_label = None

    # Threshold HSV range for chosen color label
    mask = threshold_hsv(image, color_label)

    mask_morph = morphOpen(mask)

    if DEBUG: 
        cv2.imshow('mask_blur_morph', mask_morph)
        cv2.waitKey(10)

    # find contours
    _, contours, _ = cv2.findContours(mask_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    largest_contours = sorted(contours, key=cv2.contourArea)[-10:]
    for k in range(largest_contours.__len__()):
        xx, yy, w, h = cv2.boundingRect(largest_contours[k])
        #print(box[2]*box[3])

        if float(h)/w > 0.8 and float(h)/w<1.2:
            if w*h > 2000:
                # we need to give slightly bigger image to detector to get a clear detection
                roi_x = max(0, int(xx - expansion_param*w))
                roi_y = max(0, int(yy - expansion_param*h))
                # br_x = min(image.shape[1], int(xx + w + 0.25*w))
                # br_y = min(image.shape[0], int(yy + h + 0.25*h))
                roi_w = int(w*(1+2*expansion_param))
                roi_h = int(h*(1+2*expansion_param))

                #roi_img = image[yy:yy+h, xx:xx+w]
                #roi_img = image[tl_y:tl_y+(br_y-tl_y), tl_x:tl_x+(br_x-tl_x)]
                roi_img = image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
                input_img = []
                input_img.append(cv2.resize(roi_img, (32,32)))
                input_img = np.array(input_img)

                # Normalize data
                input_img = input_img.astype('float32')
                input_img = (input_img-mean)/(std+1e-7)

                pred_color = model_color.predict(input_img)

                pred_color_label = np.argmax(pred_color)
                
                if pred_color_label != 6:   #NOTHING color
                    if(pred_color_label == color_label):
                        pred_shape = model_shape.predict(input_img)

                        pred_shape_label = np.argmax(pred_shape)
                        
                        if pred_shape_label != 7:    #NOTHING shape
                            # Get world coordinates of detected object from depth image 
                            bBox_temp = [xx, yy, w, h]
                            #if bBox_temp !=None:
                            try:
                                # get the depth of center of detected bbox
                                x_w, y_w, z_w = get_depth(bBox_temp)

                                #print(z_w)
                                #draw_result([xx,yy,w,h], image, pred_shape, pred_color)
                                draw_result([xx,yy,w,h], image, (0,255,0), pred_shape_label, pred_color_label, z_w)
                                #draw_result([tl_x,tl_y,tl_x+(br_x-tl_x),tl_y+(br_y-tl_y)], image)
                                draw_result([roi_x,roi_y, roi_w, roi_h], image, (255,0,0)) 

                                #print(color_class[np.argmax(pred_color)]+ ' ' +shape_class[pred_shape_label])
                            except:
                                pass
                                #print('wierd error!')
    return x_w, y_w, z_w, pred_shape_label, pred_color_label

def detect_battery():
    global lastFrame
    depth_img = lastFrame.depth
    rgb_img = lastFrame.image

    sobely = cv2.Sobel(depth_img,cv2.CV_64F,0,1,ksize=5)

    sobely_nonan = sobely.copy()
    bad_I = np.argwhere(np.isnan(sobely))
    sobely_nonan[bad_I[:,0],bad_I[:,1]] = 0

    sobely_thresh = np.where(sobely_nonan>0, 255, 0)
    sobely_thresh=np.uint8(sobely_thresh)
    # cv2.imshow('thresh',np.uint8(sobely_thresh))
    # cv2.waitKey(20)
    #plt.imshow(sobely) 
    

    #Morph open 
    sobely_thresh=morphOpen(sobely_thresh)
    # find contours
    _, contours, _ = cv2.findContours(sobely_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours != []:
        contour_sizes = [(cv2.contourArea(contour)>5000) for contour in contours]
        ind_temp = np.argwhere(contour_sizes)
        for j in range(ind_temp.shape[0]):
            largest_contours = contours[ind_temp[j,0]]
        
            #for k in range(largest_contours.__len__()):
            box = cv2.boundingRect(largest_contours)
            #if detect_color(rgb_img[yy:yy+h,xx:xx+w]==False):
            if  cv2.contourArea(largest_contours) < 6000:
                if float(box[2])/box[3] >1.5 or float(box[2])/box[3] < 0.7:     # flat or upright battery?
                    cv2.rectangle(rgb_img, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (0,0,255), 2)
            else:
                cv2.rectangle(rgb_img, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (0,0,255), 2)
            print(cv2.contourArea(largest_contours))
        cv2.imshow('detected battery or wall',rgb_img)
        cv2.waitKey(2)

        cv2.imshow('sobel',sobely_thresh)
        cv2.waitKey(2)
        # print('got here')  

    
#def draw_result(box, image, pred_shape, pred_color):
def draw_result(box, image, bbox_col, pred_shape_label= None, pred_color_label=None, z=None):
    cv2.rectangle(image, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), bbox_col, 2)
    
    if pred_color_label == None or pred_shape_label == None or z == None:
        pass
    else:
        '''Both shape and color on bounding box'''
        cv2.putText(image, color_class[pred_color_label]+ ' ' +shape_class[pred_shape_label], (box[0]-15,box[1]-20), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(image,  str(round(-z,3))+" m", (box[0]+15,box[1]+15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
    '''only color on bounding box'''
#     cv2.putText(image, color_class[np.argmax(pred_color)], (box[0],box[1]-15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2,
# cv2.LINE_AA)
    '''Only shape on bounding box'''
#     cv2.putText(image, shape_class[np.argmax(pred_shape)], (box[0],box[1]-15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2,
# cv2.LINE_AA)


class Frame:
    def __init__(self):
        self.image = []
        self.flagImage = False
        self.depth = []
        self.flagDepth = False
        
# create a global object of Frame
lastFrame = Frame()

#call back function to store image in class object 
def callback_storage_image(image_message):
    #print('trace 2')
    bridge = CvBridge()
    global lastFrame
    lastFrame.image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    #lastFrame.image = cv2.cvtColor(lastFrame.image, cv2.COLOR_BGR2RGB)
    lastFrame.flagImage = True


#call back function to store depth in class object
def callback_depth(image_message):
    #print('trace 3')
    bridge = CvBridge()
    global lastFrame
    lastFrame.depth = bridge.imgmsg_to_cv2(image_message, desired_encoding="32FC1")
    #print(lastFrame.depth)
    lastFrame.flagDepth = True


# MAIN FUNCTION
def main():
    rospy.init_node('object_detection_python_node', anonymous=True)
    
    # Publisher for processed image
    processedImage = rospy.Publisher("/processedImage", Image, queue_size=10)
    # Publisher for objects found
    obj_pub = rospy.Publisher("/object_position_cam_link", objects_found, queue_size=1)

    # Subscriber to RGB image
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, callback_storage_image)
    # Subscriber to depth image
    #rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image, callback_depth)
    rospy.Subscriber("/camera/depth/image", Image, callback_depth)
    
    r = rospy.Rate(5) # Hz
    while not rospy.is_shutdown():
        # Parameters for local map
        object_x = []
        object_y = []
        object_z = []
        object_shape = []
        object_color =[]

        global lastFrame
        processFrame = copy.deepcopy(lastFrame)
        #print('trace inside WHILE LOOP')
        if processFrame.flagImage == True and processFrame.flagDepth == True:
            #print('REACHED THIS POINT')
            a = datetime.now()
            frame = cv2.cvtColor(processFrame.image, cv2.COLOR_RGB2BGR)
            for i in range(N_COLORS):
                x_w, y_w, z_w, pred_shape_label, pred_color_label = detect_object(frame, i)

                if z_w !=None:
                    # fill in local map for that object
                    object_x.append(x_w)
                    object_y.append(y_w)
                    object_z.append(z_w)
                    object_shape.append(pred_shape_label)
                    object_color.append(pred_color_label)
            
            local_map = local_map = np.array((object_x, object_y, object_z, object_shape, object_color)).T

            b = datetime.now()
            c = b - a
            fps = 1.0/(c.total_seconds())

            # cv2.imshow('result', frame)
            # cv2.waitKey(20)
            obj_array = objects_found()
            obj_array.number_of_objects = local_map.shape[0]
            obj_array.array_colors = local_map[:,4].tolist()
            obj_array.array_shape = local_map[:,3].tolist()

            for j in range(local_map.shape[0]):
                point_temp = PointStamped()
                point_temp.header.frame_id = '/camera_link'
                #point_temp.header.stamp = rospy.Time.now()
                point_temp.point.x = local_map[j,0]
                point_temp.point.y = local_map[j,1]
                point_temp.point.z = local_map[j,2]
                obj_array.array_objects_found.append(point_temp)
                
            # Publish objects found
            if obj_array.number_of_objects!=0:
                obj_pub.publish(obj_array)

            # Publish processed image
            processedImage.publish(CvBridge().cv2_to_imgmsg(frame))

            
            ### DETECT BATTERY FUNCTION
            detect_battery()

            # print('##########')
            # print(local_map)
            # print('## FPS: ' + str(fps))
            # print('')
        
        r.sleep()

if __name__ == '__main__':
    main()


