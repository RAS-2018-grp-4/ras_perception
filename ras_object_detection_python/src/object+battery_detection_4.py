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
7:  Black (for batteries) ## IGNORE FOR NOW
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
from sensor_msgs.msg import Image, CompressedImage
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

# Load SHAPE CNN
json_file = open('/home/ras14/catkin_ws/src/ras_perception/DL_training/KERAS_model/saved_models/cropped_shape_3.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
model_shape = model_from_json(loaded_model_json)
model_shape.load_weights('/home/ras14/catkin_ws/src/ras_perception/DL_training/KERAS_model/saved_models/cropped_shape_3.h5')
# Load COLOR CNN
model_color = keras.models.load_model('/home/ras14/catkin_ws/src/ras_perception/DL_training/KERAS_model/saved_models/keras_cropped_color_2.h5')

# Define classes for color and shape
shape_class = ['Ball', 'Cube', 'Cylinder', 'Hollow Cube', 'Cross', 'Triangle', 'Star', 'Nothing' ]
#color_class = ['Yellow', 'Green', 'Orange', 'Red', 'Blue', 'Purple', 'Nothing']
color_class = ['Yellow', 'Green', 'Orange', 'Red', 'Blue', 'Purple']

VIDEO_INFERENCE = 0
IMG_INFERNECE = 0

N_SHAPES = 7
N_COLORS = 6

DEBUG = 0

# Mean and variance in training dataset (for data normalization)
std = 57.384      #58.363087
mean = 88.282     #85.69786

expansion_param = 0.25

#Intrinsic camera parameters depth image
cy_depth = 245.811
fy_depth = 472.631
cx_depth = 316.227
fx_depth = 472.631
Tx_depth = 0.0257
Ty_depth = 0.001266
Tz_depth = 0.00372

#Intrinsic camera parameters RGB image
cy_rgb = 223.877
fy_rgb = 614.344
cx_rgb = 314.850
fx_rgb = 616.344

# Size of square kernel of pixels for robust depth measure
square_size = 5

# ARRAY OF VALID OBJECTS
VALID_OBJECT = np.zeros((N_SHAPES, N_COLORS))

'''
                    Yellow    Green   Orange    Red     Blue    Purple
                         0       1       2       3       4       5
                 ----------------------------------------------------
    Ball        0|       1       0       0       1       0       0
    Cube        1|       1       1       0       0       1       0
    Cylinder    2|       0       1       0       1       0       0
    Hollow Cube 3|       0       1       0       1       0       0
    Cross       4|       0       0       1       0       0       1
    Triangle    5|       0       0       0       0       1       0
    Star        6|       0       0       1       0       0       1       
'''
VALID_OBJECT[0][0]=1    # Ball Yellow
VALID_OBJECT[0][3]=1    # Ball Red    
VALID_OBJECT[1][0]=1    # Cube Yellow
VALID_OBJECT[1][1]=1    # Cube Green
VALID_OBJECT[1][4]=1    # Cube Blue
VALID_OBJECT[2][1]=1    # Cylinder Green
VALID_OBJECT[2][3]=1    # Cylinder Red
VALID_OBJECT[3][1]=1    # Hollow cube green
VALID_OBJECT[3][3]=1    # Hollow cube red
VALID_OBJECT[4][2]=1    # Cross Orange
VALID_OBJECT[4][5]=1    # Cross Purple
VALID_OBJECT[5][4]=1    # Triangle Blue
VALID_OBJECT[6][2]=1    # Star Orange
VALID_OBJECT[6][5]=1    # Star Purple


'''Use intrinsic camera parameters to calculate world coordinate from pixel coordinate'''
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
    except:
        pass
        #continue
    return x_w, y_w, z


'''Function to detect objects from RGB image'''
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
        contour_sizes = [(cv2.contourArea(contour)>1200) for contour in contours]
        ind_temp = np.argwhere(contour_sizes)
        
        for k in range(ind_temp.shape[0]):
            largest_contours = contours[ind_temp[k,0]]
            
            xx, yy, w, h = cv2.boundingRect(largest_contours)

            if float(h)/w > 0.8 and float(h)/w<1.3:
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
                
                #if pred_color_label != 6:   #NOTHING color
                if(pred_color_label == color_label):
                    pred_shape = model_shape.predict(input_img)

                    pred_shape_label = np.argmax(pred_shape)
                    
                    if pred_shape_label != 7:    #NOTHING shape
                        '''CHECK IF COMBINED SHAPE + COLOR LABEL IS A VALID COMBINATION'''
                        if VALID_OBJECT[pred_shape_label][pred_color_label]==1:
                            #print(cv2.contourArea(largest_contours))
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
                                
                                '''DISTANCE THRESHOLD FOR OBJECT DETECTION'''
                                if z_w <= 0.45:
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

    for j in range(obj_array.number_of_objects):
        point_rgboptical = PointStamped()
        #point_rgboptical.header.frame_id = "camera_rgb_optical_frame"
        point_rgboptical.header.frame_id = "/camera_link"

        #point_temp.header.stamp = rospy.Time.now()
        point_rgboptical.point.x = local_map[j,0]
        point_rgboptical.point.y = local_map[j,1]
        point_rgboptical.point.z = local_map[j,2]

        #point_map = listener_rgboptical2map.transformPoint("/map",point_rgboptical)
        obj_array.array_objects_found.append(point_rgboptical)

    return obj_array


'''Helper function to draw rectangle and put text on rgb image'''
#def draw_result(box, image, pred_shape, pred_color):
def draw_result(box, image, bbox_col, pred_shape_label= None, pred_color_label=None, z=None):
    cv2.rectangle(image, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), bbox_col, 2)
    
    if pred_color_label == None or pred_shape_label == None or z == None:
        pass
    else:
        '''Both shape and color on bounding box'''
        cv2.putText(image, color_class[pred_color_label]+ ' ' +shape_class[pred_shape_label], (box[0]-15,box[1]-20), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(image,  str(round(z,3))+" m", (box[0]+15,box[1]+15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
    '''only color on bounding box'''
#     cv2.putText(image, color_class[np.argmax(pred_color)], (box[0],box[1]-15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2,
# cv2.LINE_AA)
    '''Only shape on bounding box'''
#     cv2.putText(image, shape_class[np.argmax(pred_shape)], (box[0],box[1]-15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2,
# cv2.LINE_AA)



'''Function to detect BATTERY from depth image and show it on RGB image
Depth is taken from non rectified image, but info is superimposed to 
rect depth image. this allows us to ignore stupid transforms from depth 
frame to map frame
'''
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
        contour_sizes = [(cv2.contourArea(contour)>8000) for contour in contours]
        ind_temp = np.argwhere(contour_sizes)
        for j in range(ind_temp.shape[0]):
            largest_contours = contours[ind_temp[j,0]]
        
            xx, yy, w, h = cv2.boundingRect(largest_contours)

            # print(float(h)/w)
            # print(cv2.contourArea(largest_contours))
                                
            
            # check color of bounding box, if it's not a color then its a battery!
            # roi_img = rgb_img[yy:yy+h, xx:xx+w]
            # input_img = []
            # input_img.append(cv2.resize(roi_img, (32,32)))
            # input_img = np.array(input_img)

            # # Normalize data
            # input_img = input_img.astype('float32')
            # input_img = (input_img-mean)/(std+1e-7)

            # pred_color = model_color.predict(input_img)
            # pred_color_label = np.argmax(pred_color)

            # if pred_color_label == 6:   #NOTHING color
            
            if float(h)/w >0.4:
                cv2.rectangle(rgb_img, (xx, yy), (xx+w, yy+h), (0,0,255), 2)
            
                try:
                    # Define a square kernel for depth measurement 
                    depth_square = lastFrame.depth_registered[int(-square_size+round(yy+h/2)):int(square_size+round(yy+h/2)),int(-square_size+round(xx+w/2)):int(square_size+round(xx+w/2))]
                    cv2.rectangle(rgb_img, (int(xx+w/2)-square_size, int(yy+h/2)-square_size), (int(xx+w/2)+square_size, int(yy+h/2)+square_size), (255,0,0), 3)
                
                    x_w, y_w, z_w = get_world_coord_rgb([xx, yy, w, h],depth_square)

                    '''DISTANCE THRESHOLD FOR BATTERY DETECTION'''
                    if z_w <= 0.4:
                        #point_depthoptical = PoseStamped()
                        point_depthoptical = Pose()
                        #point_depthoptical.header.frame_id = "camera_depth_optical_frame"
                        
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


        '''CODE FOR BLACK PART OF BATTERY'''
        # # Threshold HSV range for BLACK color
        # mask = threshold_hsv(rgb_img, 7)
        # # Morphological opening
        # mask_morph = morphOpen(mask)

        # # find contours
        # _, contours, _ = cv2.findContours(mask_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # contour_sizes = [(cv2.contourArea(contour)>20000) for contour in contours]
        # ind_temp = np.argwhere(contour_sizes)
        
        # for k in range(ind_temp.shape[0]):
        #     largest_contours = contours[ind_temp[k,0]]
        #     #print(cv2.contourArea(largest_contours))
        #     box = cv2.boundingRect(largest_contours)
        #     xx, yy, w, h = box
        #     cv2.rectangle(rgb_img, (xx, yy), (xx+w, yy+h), (0,255,0), 2)
            
        #     # Define a square kernel for depth measurement
        #     depth_square = lastFrame.depth[int(yy+h):int(2*square_size+(yy+h)),int(-square_size+round(xx+w/2)):int(square_size+round(xx+w/2))]
        #     cv2.rectangle(rgb_img, (int(xx+w/2), yy+h), (int(xx+w/2)+2*square_size, yy+h+2*square_size), (255,0,0), 3)
        #     try:
        #         x_w, y_w, z_w = get_world_coord_rgb(box, depth_square)
        #         #get_blackface_depth(box)

        #         # append to battery array
        #         point_depthoptical = Pose()
        #         ##### BELOW LINE IS AN APPROXIMATION! Actual frame is rgb optical x(
        #         #point_depthoptical.header.frame_id = 'camera_depth_optical_frame'
        #         # For this to be valid, we could just add 3 cm to x_w
        #         #x_w = x_w - 0.03

        #         #point_temp.header.stamp = rospy.Time.now()
        #         point_depthoptical.position.x = x_w
        #         point_depthoptical.position.y = y_w
        #         point_depthoptical.position.z = z_w

        #         point_depthoptical.orientation.x = 0.0
        #         point_depthoptical.orientation.y = 0.0
        #         point_depthoptical.orientation.z = 0.0
        #         point_depthoptical.orientation.w = 1.0
        #         battery_pos_array.poses.append(point_depthoptical)
        #     except:
        #         pass
    # print('got here') 

    # SHOW BATTERY IMAGES 
    # cv2.imshow('detected battery or wall',rgb_img)
    # cv2.waitKey(2)

    # cv2.imshow('sobel',sobely_thresh)
    # cv2.waitKey(2)

    return battery_pos_array
    

#call back function to store image in class object 
def callback_storage_image(image_message):
    bridge = CvBridge()
    global lastFrame
    lastFrame.image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
    # Image to numpy array
    #np_arr = np.fromstring(image_message.data, np.uint8)
    # Decode to cv2 image and store
    #lastFrame.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    lastFrame.flagImage = True
    


#call back function to store depth in class object
def callback_depth(image_message):
    bridge = CvBridge()
    global lastFrame
    lastFrame.depth = bridge.imgmsg_to_cv2(image_message, desired_encoding="32FC1")
    lastFrame.flagDepth = True

def callback_depth_registered(image_message):
    bridge = CvBridge()
    global lastFrame
    lastFrame.depth_registered = bridge.imgmsg_to_cv2(image_message, desired_encoding="32FC1")
    lastFrame.flagDepth_Registered = True

# MAIN FUNCTION
def main():
    rospy.init_node('object_detection_python_node', anonymous=True)
    
    # Publisher for processed image
    processedImage = rospy.Publisher("/processedImage", Image, queue_size=10)
    # Publisher for processed image
    batteryImage = rospy.Publisher("/batteryImage", Image, queue_size=10)
    # Publisher for objects found
    obj_pub = rospy.Publisher("/object_position_cam_link", objects_found, queue_size=1)
    # Publisher for detected batteries
    #pub_battery_detection = rospy.Publisher('/battery_detection', std_msgs.msg.Bool, queue_size=1)
    pub_battery_position = rospy.Publisher('/battery_position_cam_link', PoseArray, queue_size=1)


    # Subscriber to RGB image: ADD /compressed, CompressedImage Image type
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, callback_storage_image)
    # Subscriber to depth_registered image
    rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image, callback_depth_registered)
    # Subscriber to depth image
    rospy.Subscriber("/camera/depth/image", Image, callback_depth)
    
    r = rospy.Rate(10) # Hz
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
            battery_pos_array.header.frame_id = "/camera_link"

            # Publish battery position
            '''
            Q: Should robot stop when it sees a battery?
            '''
            pub_battery_position.publish(battery_pos_array)
            
            # Publish detected battery in an image
            batteryImage.publish(CvBridge().cv2_to_imgmsg(frame))

        r.sleep()

if __name__ == '__main__':
    main()
    cv2.destroyAllWindows()


