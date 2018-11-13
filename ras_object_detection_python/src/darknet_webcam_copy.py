from ctypes import *
import math
import random
import numpy as np 
import cv2
from datetime import datetime
import rospy 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#rospy.init_node('publish_webcam', anonymous=True)

###### GLOBAL VARIABLES
cap = cv2.VideoCapture(0)
last_frame = []

def sample(probs):
    s = sum(probs)
    probs = [a/s for a in probs]
    r = random.uniform(0, 1)
    for i in range(len(probs)):
        r = r - probs[i]
        if r <= 0:
            return i
    return len(probs)-1

def c_array(ctype, values):
    arr = (ctype*len(values))()
    arr[:] = values
    return arr

class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]

class DETECTION(Structure):
    _fields_ = [("bbox", BOX),
                ("classes", c_int),
                ("prob", POINTER(c_float)),
                ("mask", POINTER(c_float)),
                ("objectness", c_float),
                ("sort_class", c_int)]


class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]

class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]

    

#lib = CDLL("/home/pjreddie/documents/darknet/libdarknet.so", RTLD_GLOBAL)
lib = CDLL("/home/ajinkya/darknet_pjreddie/libdarknet.so", RTLD_GLOBAL)
lib.network_width.argtypes = [c_void_p]
lib.network_width.restype = c_int
lib.network_height.argtypes = [c_void_p]
lib.network_height.restype = c_int

predict = lib.network_predict
predict.argtypes = [c_void_p, POINTER(c_float)]
predict.restype = POINTER(c_float)

set_gpu = lib.cuda_set_device
set_gpu.argtypes = [c_int]

make_image = lib.make_image
make_image.argtypes = [c_int, c_int, c_int]
make_image.restype = IMAGE

get_network_boxes = lib.get_network_boxes
get_network_boxes.argtypes = [c_void_p, c_int, c_int, c_float, c_float, POINTER(c_int), c_int, POINTER(c_int)]
get_network_boxes.restype = POINTER(DETECTION)

make_network_boxes = lib.make_network_boxes
make_network_boxes.argtypes = [c_void_p]
make_network_boxes.restype = POINTER(DETECTION)

free_detections = lib.free_detections
free_detections.argtypes = [POINTER(DETECTION), c_int]

free_ptrs = lib.free_ptrs
free_ptrs.argtypes = [POINTER(c_void_p), c_int]

network_predict = lib.network_predict
network_predict.argtypes = [c_void_p, POINTER(c_float)]

reset_rnn = lib.reset_rnn
reset_rnn.argtypes = [c_void_p]

load_net = lib.load_network
load_net.argtypes = [c_char_p, c_char_p, c_int]
load_net.restype = c_void_p

do_nms_obj = lib.do_nms_obj
do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

do_nms_sort = lib.do_nms_sort
do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

free_image = lib.free_image
free_image.argtypes = [IMAGE]

letterbox_image = lib.letterbox_image
letterbox_image.argtypes = [IMAGE, c_int, c_int]
letterbox_image.restype = IMAGE

load_meta = lib.get_metadata
lib.get_metadata.argtypes = [c_char_p]
lib.get_metadata.restype = METADATA

load_image = lib.load_image_color
load_image.argtypes = [c_char_p, c_int, c_int]
load_image.restype = IMAGE

rgbgr_image = lib.rgbgr_image
rgbgr_image.argtypes = [IMAGE]

predict_image = lib.network_predict_image
predict_image.argtypes = [c_void_p, IMAGE]
predict_image.restype = POINTER(c_float)

def classify(net, meta, im):
    out = predict_image(net, im)
    res = []
    for i in range(meta.classes):
        res.append((meta.names[i], out[i]))
    res = sorted(res, key=lambda x: -x[1])
    return res

def detect(net, meta, image, thresh=.5, hier_thresh=.5, nms=.45):
    #check if image is an OpenCV frame
    if isinstance(image, np.ndarray):
        # GET C,H,W, and DATA values
        img = image.transpose(2, 0, 1)
        c, h, w = img.shape[0], img.shape[1], img.shape[2]
        nump_data = img.ravel() / 255.0
        nump_data = np.ascontiguousarray(nump_data, dtype=np.float32)
        #nump_data = np.ascontiguousarray(nump_data, dtype=np.int8)

        # make c_type pointer to numpy array
        ptr_data = nump_data.ctypes.data_as(POINTER(c_float))

        # make IMAGE data type
        im = IMAGE(w=w, h=h, c=c, data=ptr_data)
        #print('image is numpy array!!')
        #print(im)
    else:
        im = load_image(image, 0, 0)
        #print(im)

    #print('IMAGE SIZE IS: ' + str(im.c) + ', ' + str(im.h) + ', ' + str(im.w) )
    #print('IMAGE ptr_data is : ' + str(im.data))
    
    #im = load_image(image, 0, 0)
    #im = image
    num = c_int(0)
    pnum = pointer(num)
    predict_image(net, im)
    dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, None, 0, pnum)
    num = pnum[0]
    if (nms): do_nms_obj(dets, num, meta.classes, nms)

    res = []
    for j in range(num):
        for i in range(meta.classes):
            if dets[j].prob[i] > 0:
                b = dets[j].bbox
                res.append((meta.names[i], dets[j].prob[i], (b.x, b.y, b.w, b.h)))
    res = sorted(res, key=lambda x: -x[1])
    #free_image(im)
    free_detections(dets, num)
    return res



def draw_results(res, img):
    for element in res:
        box = element[2]
        xmin = int(box[0] - box[2] / 2. + 1)
        xmax = int(box[0] + box[2] / 2. + 1)
        ymin = int(box[1] - box[3] / 2. + 1)
        ymax = int(box[1] + box[3] / 2. + 1)
        rand_color = (random.randint(0,255),random.randint(0,255),random.randint(0,255))
        cv2.rectangle(img, (xmin, ymin), (xmax, ymax),color=rand_color , thickness=5, )
        cv2.putText(img, str(element[0])+" "+ '%.2f' % element[1],
                    (xmin, ymin),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, rand_color, thickness=5)


def callback_webcam_image(img_msg):
    global last_frame
    bridge = CvBridge()
    last_frame = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough") 

if __name__ == "__main__":
    global last_frame
    rospy.init_node('publish_webcam', anonymous=True)
    rospy.Subscriber("/webcam_image", Image, callback_webcam_image)
    #net = load_net("cfg/densenet201.cfg", "/home/pjreddie/trained/densenet201.weights", 0)
    #im = load_image("data/wolf.jpg", 0, 0)
    #meta = load_meta("cfg/imagenet1k.data")
    #r = classify(net, meta, im)
    ##print r[:10]
    
    #net = load_net("yolov2-tiny-RAS.cfg", "yolov2-tiny-RAS_440000.weights", 0)
    #meta = load_meta("ras_mydata.txt")
    net = load_net("yolov2-nano1-RAS.cfg", "yolov2-nano1-RAS_1700.weights", 0)
    meta = load_meta("ras_mydata.txt")

    while rospy.is_shutdown():
    #while cap.isOpened():
        
        a = datetime.now()

        ret, frame = cap.read()
        
        #frame = cv2.imread("data/dog.jpg")
        #print(frame[0][0][0])

        r = detect(net, meta, frame)
    
        #r = detect(net, meta, "data/dog.jpg")
        print r

        draw_results(r, frame)

        cv2.imshow('frame', frame)

        b = datetime.now()
        c = b - a
        fps = 1.0/(c.total_seconds())
        print('## FPS: ' + str(fps))
        if cv2.waitKey(1) & 0xFF == ord('q'):
           break    
