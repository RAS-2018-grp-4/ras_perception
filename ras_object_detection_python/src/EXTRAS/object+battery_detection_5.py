#!/usr/bin/env python
'''
@author:Ajinkya Khoche

This script subscribes to RGB and depth images and calls:
    - detect_object to detect a valuable object
    - detect battery to detect obstacles.

for object detection it uses 2 CNNs, one each for shape and color.

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
and...
7:  Black (for batteries)
'''
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
from geometry_msgs.msg import PointStamped, Pose, PoseArray
import tf
from matplotlib import pyplot as plt


class Frame:
    def __init__(self):
        self.image = []
        self.flagImage = False
        self.depth = []
        self.flagDepth = False
        self.depth_registered = []
        self.flagDepth_Registered = False
        
lastFrame = Frame()
# class Detector():
#     def __init__(self):
#         # create a global object of Frame
#         self.lastFrame = Frame()

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

#Intrinsic camera parameters rgb image
cy_rgb = 245.811
fy_rgb = 472.631
cx_rgb = 316.227
fx_rgb = 472.631

#Intrinsic camera parameters depth image
cy_depth = 352.49
fy_depth = 672.55
cx_depth = 635.18
fx_depth = 672.55

# Size of square kernel of pixels for robust depth measure
square_size = 5

#def get_world_coord_rgb(bBox):
def get_world_coord_rgb(bBox,depth_square):
    xx = bBox[0]
    yy = bBox[1]
    w = bBox[2]
    h = bBox[3]

    depth_square = copy.copy(depth_square)
    bad_I = np.argwhere(np.isnan(depth_square))
    depth_square[bad_I[:,0],bad_I[:,1]] = 0
    bad_I = np.argwhere(np.isinf(depth_square))
    depth_square[bad_I[:,0],bad_I[:,1]] = 0

    valid_index = np.nonzero(depth_square)
    if valid_index[0].shape[0] == 0 or valid_index[1].shape[0] == 0:
        return
    
    z = np.mean(depth_square[valid_index[0], valid_index[1]]) 
    z = z
        
    # use intrinsic camera parameters to convert from pixel coordinate to 
    # world coordinate (http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html)
    try:
        y_w = (round(yy+h/2) - cy_rgb) / fy_rgb * z
        x_w = (round(xx+w/2) - cx_rgb) / fx_rgb * z
        # APPROXIMATION:
        x_w = -x_w
        y_w = -y_w
    except:
        pass
        #continue
    return x_w, y_w, z



def detect_object(image):
    # Parameters for local map
    object_x = []
    object_y = []
    object_z = []
    object_shape = []
    object_color =[]

    for color_label in range(N_COLORS):
        # Threshold HSV range for chosen color label
        mask = threshold_hsv(image, color_label)
        # Morphological opening
        mask_morph = morphOpen(mask)

        # find contours
        _, contours, _ = cv2.findContours(mask_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_sizes = [(cv2.contourArea(contour)>2000) for contour in contours]
        ind_temp = np.argwhere(contour_sizes)
        
        for k in range(ind_temp.shape[0]):
            largest_contours = contours[ind_temp[k,0]]
        
            xx, yy, w, h = cv2.boundingRect(largest_contours)

            if float(h)/w > 0.8 and float(h)/w<1.2:
                roi_x = max(0, int(xx - expansion_param*w))
                roi_y = max(0, int(yy - expansion_param*h))
                roi_w = int(w*(1+2*expansion_param))
                roi_h = int(h*(1+2*expansion_param))

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
                            # Define a square kernel for depth measurement 
                            depth_square = lastFrame.depth_registered[int(-square_size+round(yy+h/2)):int(square_size+round(yy+h/2)),int(-square_size+round(xx+w/2)):int(square_size+round(xx+w/2))]

                            #if bBox_temp !=None:
                            try:
                                # get the depth of center of detected bbox
                                #x_w, y_w, z_w = get_world_coord_rgb(bBox_temp)                                
                                x_w, y_w, z_w = get_world_coord_rgb(bBox_temp,depth_square)

                                draw_result([xx,yy,w,h], image, (0,255,0), pred_shape_label, pred_color_label, z_w)
                                draw_result([roi_x,roi_y, roi_w, roi_h], image, (255,0,0)) 

                                object_x.append(x_w)
                                object_y.append(y_w)
                                object_z.append(z_w)
                                object_shape.append(pred_shape_label)
                                object_color.append(pred_color_label)
                            except:
                                pass
                                #print('wierd error!')
    
    local_map = np.array((object_x, object_y, object_z, object_shape, object_color)).T

    obj_array = objects_found()
    obj_array.number_of_objects = local_map.shape[0]
    obj_array.array_colors = local_map[:,4].tolist()
    obj_array.array_shape = local_map[:,3].tolist()

    # wait for transform from rgb optical to map frame
    #listener_rgboptical2map.waitForTransform("camera_rgb_optical_frame", "/map", rospy.Time(0),rospy.Duration(4.0))

    for j in range(obj_array.number_of_objects):
        point_rgboptical = PointStamped()
        point_rgboptical.header.frame_id = "camera_rgb_optical_frame"

        #point_temp.header.stamp = rospy.Time.now()
        point_rgboptical.point.x = local_map[j,0]
        point_rgboptical.point.y = local_map[j,1]
        point_rgboptical.point.z = local_map[j,2]

        #point_map = listener_rgboptical2map.transformPoint("/map",point_rgboptical)
        obj_array.array_objects_found.append(point_rgboptical)

    return obj_array


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





def detect_battery(rgb_img, depth_img):
    battery_pos_array = PoseArray()
    # take image gradient along y 
    sobely = cv2.Sobel(depth_img,cv2.CV_64F,0,1,ksize=5)

    sobely_nonan = sobely.copy()
    bad_I = np.argwhere(np.isnan(sobely))
    sobely_nonan[bad_I[:,0],bad_I[:,1]] = 0

    sobely_thresh = np.where(sobely_nonan>0, 255, 0)
    sobely_thresh=np.uint8(sobely_thresh)
    
    #Morph open 
    sobely_thresh=morphOpen(sobely_thresh)
    # find contours
    _, contours, _ = cv2.findContours(sobely_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours != []:
        contour_sizes = [(cv2.contourArea(contour)>6000) for contour in contours]
        ind_temp = np.argwhere(contour_sizes)
        for j in range(ind_temp.shape[0]):
            largest_contours = contours[ind_temp[j,0]]
        
            #for k in range(largest_contours.__len__()):
            xx, yy, w, h = cv2.boundingRect(largest_contours)
            #if detect_color(rgb_img[yy:yy+h,xx:xx+w]==False):
            # if  cv2.contourArea(largest_contours) < 6000:
            #     if float(box[2])/box[3] >1.5 or float(box[2])/box[3] < 0.7:     # flat or upright battery?
            #         cv2.rectangle(rgb_img, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (0,0,255), 2)
            # else:
            cv2.rectangle(rgb_img, (xx, yy), (xx+w, yy+h), (0,0,255), 2)
        
            # take a pixel wide strip for each box and find its world coordinates
            #xx, yy, w, h = box
            d_temp = depth_img[int(yy+h/2),xx:xx+w]
            for k in range(d_temp.shape[0]):
                if(np.isnan(d_temp[k])):
                    pass
                else:
                    # get the depth at x = xx + k, y = yy + h/2, where:
                    # [xx yy w h] = box
                    #z_w = depth_img[xx:xx+k, yy:round(yy+h/2)]
                    z_w = d_temp[k]
                    z_w = z_w
                    # use intrinsic camera parameters to convert from pixel coordinate to 
                    # world coordinate (http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html)
                    try:
                        y_w = (round(yy+h/2) - cy_depth) / fy_depth * z_w
                        x_w = (round(xx+k) - cx_depth) / fx_depth * z_w

                        # APPROXIMATION:
                        x_w = -x_w
                        y_w = -y_w 

                        #point_depthoptical = PoseStamped()
                        point_depthoptical = Pose()
                        #point_depthoptical.header.frame_id = "camera_depth_optical_frame"
                        
                        #point_temp.header.stamp = rospy.Time.now()
                        #point_depthoptical.header.frame_id = depth_img.header.frame_id
                        #point_depthoptical.header.stamp = rospy.Time.now()
                        point_depthoptical.pose.position.x = x_w
                        point_depthoptical.pose.position.y = y_w
                        point_depthoptical.pose.position.z = z_w

                        point_depthoptical.pose.orientation.x = 0.0
                        point_depthoptical.pose.orientation.y = 0.0
                        point_depthoptical.pose.orientation.z = 0.0
                        point_depthoptical.pose.orientation.w = 1.0
                        battery_pos_array.poses.append(point_depthoptical)
                    except:
                        pass


        # Threshold HSV range for BLACK color
        mask = threshold_hsv(rgb_img, 7)
        # Morphological opening
        mask_morph = morphOpen(mask)

        # find contours
        _, contours, _ = cv2.findContours(mask_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_sizes = [(cv2.contourArea(contour)>20000) for contour in contours]
        ind_temp = np.argwhere(contour_sizes)
        
        for k in range(ind_temp.shape[0]):
            largest_contours = contours[ind_temp[k,0]]
            #print(cv2.contourArea(largest_contours))
            box = cv2.boundingRect(largest_contours)
            xx, yy, w, h = box
            cv2.rectangle(rgb_img, (xx, yy), (xx+w, yy+h), (0,255,0), 2)
            
            # Define a square kernel for depth measurement
            depth_square = lastFrame.depth[int(yy+h):int(2*square_size+(yy+h)),int(-square_size+round(xx+w/2)):int(square_size+round(xx+w/2))]
            cv2.rectangle(rgb_img, (int(xx+w/2), yy+h), (int(xx+w/2)+2*square_size, yy+h+2*square_size), (255,0,0), 3)
            try:
                x_w, y_w, z_w = get_world_coord_rgb(box, depth_square)
                #get_blackface_depth(box)

                # append to battery array
                point_depthoptical = Pose()
                ##### BELOW LINE IS AN APPROXIMATION! Actual frame is rgb optical x(
                #point_depthoptical.header.frame_id = 'camera_depth_optical_frame'
                # For this to be valid, we could just add 3 cm to x_w
                #x_w = x_w - 0.03

                #point_temp.header.stamp = rospy.Time.now()
                point_depthoptical.position.x = x_w
                point_depthoptical.position.y = y_w
                point_depthoptical.position.z = z_w

                point_depthoptical.orientation.x = 0.0
                point_depthoptical.orientation.y = 0.0
                point_depthoptical.orientation.z = 0.0
                point_depthoptical.orientation.w = 1.0
                battery_pos_array.poses.append(point_depthoptical)
            except:
                pass
    # print('got here')  
    cv2.imshow('detected battery or wall',rgb_img)
    cv2.waitKey(2)

    cv2.imshow('sobel',sobely_thresh)
    cv2.waitKey(2)

    return battery_pos_array
    

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

def callback_depth_registered(image_message):
    #print('trace 3')
    bridge = CvBridge()
    global lastFrame
    lastFrame.depth_registered = bridge.imgmsg_to_cv2(image_message, desired_encoding="32FC1")
    #print(lastFrame.depth)
    lastFrame.flagDepth_Registered = True

# MAIN FUNCTION
def main():
    rospy.init_node('object_detection_python_node', anonymous=True)
    
    # Publisher for processed image
    processedImage = rospy.Publisher("/processedImage", Image, queue_size=10)
    # Publisher for objects found
    obj_pub = rospy.Publisher("/object_position_cam_link", objects_found, queue_size=1)
    #obj_pub = rospy.Publisher("/object_position_map", objects_found, queue_size=1)
    # Publisher for detected batteries
    #pub_battery_detection = rospy.Publisher('/battery_detection', std_msgs.msg.Bool, queue_size=1)
    pub_battery_position = rospy.Publisher('/battery_position_cam_link', PoseArray, queue_size=1)


    # Subscriber to RGB image
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, callback_storage_image)
    # Subscriber to depth_registered image
    rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image, callback_depth_registered)
    # Subscriber to depth image
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

        if processFrame.flagImage == True and processFrame.flagDepth_Registered == True:
            a = datetime.now()
            frame = cv2.cvtColor(processFrame.image, cv2.COLOR_RGB2BGR)
            
            #create object_found object
            obj_array = objects_found()
            # see if you find an object in rgb frame?
            obj_array = detect_object(frame)

            # Publish objects found: 
            '''
            Q: Should this message be published when its empty?
            '''
            if obj_array.number_of_objects!=0:
                obj_pub.publish(obj_array)

            # Publish processed image
            processedImage.publish(CvBridge().cv2_to_imgmsg(frame))

        if processFrame.flagImage == True and processFrame.flagDepth == True:   
            frame = cv2.cvtColor(processFrame.image, cv2.COLOR_RGB2BGR)
            depth_frame = processFrame.depth
            
            ### DETECT BATTERY FUNCTION
            battery_pos_array = detect_battery(frame, depth_frame)
            battery_pos_array.header.frame_id = "camera_link"

            # Publish battery position
            '''
            Q: Should robot stop when it sees a battery?
            '''
            pub_battery_position.publish(battery_pos_array)

        r.sleep()

if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()


