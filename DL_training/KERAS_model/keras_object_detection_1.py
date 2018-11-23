import cv2
import numpy as np 
import keras.models 
import glob
from datetime import datetime
from keras.models import model_from_json

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
json_file = open('saved_models/cropped_shape_1.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
model_shape = model_from_json(loaded_model_json)
model_shape.load_weights('saved_models/cropped_shape_1.h5')
#model_shape = keras.models.load_model('./saved_models/keras_RAS_model_shape_1.h5')
model_color = keras.models.load_model('./saved_models/keras_cropped_color_1.h5')

shape_class = ['Ball', 'Cube', 'Cylinder', 'Hollow Cube', 'Cross', 'Triangle', 'Star', 'Nothing' ]
color_class = ['Yellow', 'Green', 'Orange', 'Red', 'Blue', 'Purple', 'Nothing']

VIDEO_INFERENCE = 1
IMG_INFERNECE = 0

N_SHAPES = 7
N_COLORS = 6

DEBUG = 0

std = 58.363087
mean = 85.69786

expansion_param = 0.25

def morphOpen(image):
    # define structuring element
    # take 5% of least dimension of image as kernel size
    kernel_size = min(5, int(min(image.shape[0],image.shape[1])*0.05))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernel_size,kernel_size))
    #kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    return opening

def detect_object(image, color_label):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    if color_label == 0:    #YELLOW
        mask = cv2.inRange(hsv, np.array([17,128,90]), np.array([32,255,255]))
    elif color_label == 1:  #GREEN
        mask = cv2.inRange(hsv, np.array([32,50,0]), np.array([66,255,255]))
    elif color_label == 2:  #ORANGE
        # mask1 = cv2.inRange(hsv, np.array([5,150,150]), np.array([15,255,255]))
        # mask2 = cv2.inRange(hsv, np.array([160,220,150]), np.array([169,255,255]))
        # mask = mask1 + mask2 
        mask = cv2.inRange(hsv, np.array([5,150,150]), np.array([15,255,255]))
    elif color_label == 3:  #RED
        mask1 = cv2.inRange(hsv, np.array([0,130,120]), np.array([8,255,255]))
        mask2 = cv2.inRange(hsv, np.array([170,130,200]), np.array([179,255,255]))
        mask = mask1 + mask2 
        #mask = cv2.inRange(hsv, np.array([0,150,0]), np.array([4,255,200]))
    elif color_label == 4:  #BLUE
        mask = cv2.inRange(hsv, np.array([70,0,0]), np.array([120,255,255]))
    elif color_label == 5:  #PURPLE
        mask = cv2.inRange(hsv, np.array([28,0,50]), np.array([179,166,170]))

    # if DEBUG:
    #     cv2.imshow('mask', mask)
    #     cv2.waitKey(0)

    # blur image
    #mask_blur = cv2.GaussianBlur(mask,(2,2),0)
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

                #bBox_img = image[yy:yy+h, xx:xx+w]
                #bBox_img = image[tl_y:tl_y+(br_y-tl_y), tl_x:tl_x+(br_x-tl_x)]
                bBox_img = image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
                input_img = []
                input_img.append(cv2.resize(bBox_img, (32,32)))
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
                            #draw_result([xx,yy,w,h], image, pred_shape, pred_color)
                            draw_result([xx,yy,w,h], image, (0,255,0), pred_shape_label, pred_color_label)
                            #draw_result([tl_x,tl_y,tl_x+(br_x-tl_x),tl_y+(br_y-tl_y)], image)
                            draw_result([roi_x,roi_y, roi_w, roi_h], image, (255,0,0)) 

                            #print(color_class[np.argmax(pred_color)]+ ' ' +shape_class[pred_shape_label])
    
#def draw_result(box, image, pred_shape, pred_color):
def draw_result(box, image, bbox_col, pred_shape_label= None, pred_color_label=None):
    cv2.rectangle(image, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), bbox_col, 2)
    
    if pred_color_label == None or pred_shape_label == None:
        pass
    else:
        '''Both shape and color on bounding box'''
        cv2.putText(image, color_class[pred_color_label]+ ' ' +shape_class[pred_shape_label], (box[0],box[1]-15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
    '''only color on bounding box'''
#     cv2.putText(image, color_class[np.argmax(pred_color)], (box[0],box[1]-15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2,
# cv2.LINE_AA)
    '''Only shape on bounding box'''
#     cv2.putText(image, shape_class[np.argmax(pred_shape)], (box[0],box[1]-15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2,
# cv2.LINE_AA)


if VIDEO_INFERENCE:    
    #cap = cv2.VideoCapture('../ras_labeling/vid1.mp4')
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        a = datetime.now()

        ret, image = cap.read()
        
        resize_x = 1
        resize_y = 1
        image = cv2.resize(image, (0,0), fx=resize_x, fy=resize_y)

        for i in range(N_COLORS):
            detect_object(image, i)
        
        cv2.imshow('result', image)
        cv2.waitKey(1)
        #cv2.waitKey(200)
        
        b = datetime.now()
        c = b - a
        fps = 1.0/(c.total_seconds())
    
        print('## FPS: ' + str(fps))
        print('')

elif IMG_INFERNECE:
    try:
        while True:
            for file in glob.glob('../RAS_DATASET' + "/*.jpg"):
                image = cv2.imread(file)
                for i in range(N_COLORS):
                    detect_object(image, i)
                
                cv2.imshow('result', image)
                cv2.waitKey(0)            

    except KeyboardInterrupt:
        pass

    # image = cv2.imread('./1001.jpg')
    # image = cv2.resize(image, (0,0), fx=0.33, fy=0.33)

    # for i in range(N_COLORS):
    #     detect_object(image, i)
    
    # cv2.imshow('result', image)
    # cv2.waitKey(0)



