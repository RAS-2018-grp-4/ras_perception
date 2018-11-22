import cv2
import numpy as np
import keras.models 
import glob
from datetime import datetime
from keras.models import model_from_json

'''
@author:Ajinkya Khoche

# of classes Based on "Object dataset" in Modules
0:  Ball
1:  Cube
2:  Cylinder
3:  Hollow Cube
4:  Cross
5:  Triangle
6:  Star
'''

PATH_TEST = "../image_dataset_keras_shape/" 
VIDEO_INFERENCE = 1
IMG_INFERNECE = 0

std = 58.363087
mean = 85.69786

#model_shape = keras.models.load_model('saved_models/keras_RAS_model_shape_4.h5')
# Load trained CNN model
json_file = open('saved_models/model.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
model_shape = model_from_json(loaded_model_json)
model_shape.load_weights('saved_models/model.h5')
#model_shape = keras.models.load_model('saved_models/model.h5')

shape_class = ['Ball', 'Cube', 'Cylinder', 'Hollow Cube', 'Cross', 'Triangle', 'Star' ]

if VIDEO_INFERENCE:    
    cap = cv2.VideoCapture('../ras_labeling/vid2.mp4')
    #cap = cv2.VideoCapture(0)

    while cap.isOpened():
        a = datetime.now()

        ret, image = cap.read()

        if ret:
            input_img = []
            input_img.append(cv2.resize(image, (32,32)))
            input_img_clone = cv2.resize(image, (32,32))
            
            input_img = np.array(input_img)
            input_img = input_img.astype('float32')

            input_img = (input_img-mean)/(std+1e-7)
            
            prediction = model_shape.predict(input_img)
            print(shape_class[np.argmax(prediction)])

            cv2.imshow('image', image)
            cv2.waitKey(10)

            b = datetime.now()
            c = b - a
            fps = 1.0/(c.total_seconds())
            print('## FPS: ' + str(fps))
            print('')

elif IMG_INFERNECE:
    try:
        # while True:
        #     label = np.random.randint(0,7)

        #     if label == 0:
        #         dirname = 'Ball'       
        #     elif label == 1:
        #         dirname = 'Cube'
        #     elif label == 2:
        #         dirname = 'Cylinder'
        #     elif label == 3:
        #         dirname = 'Hollow Cube'
        #     elif label == 4:
        #         dirname = 'Cross'
        #     elif label == 5:
        #         dirname = 'Triangle'
        #     elif label == 6:
        #         dirname = 'Star'

        for file in glob.glob('../CROPPED_DATASET'+ "/*.jpg"):
            image = cv2.imread(file)

            input_img = []
            input_img.append(cv2.resize(image, (32,32)))

            input_img_clone = cv2.resize(image, (32,32))
            input_img = np.array(input_img)


            input_img = input_img.astype('float32')

            input_img = (input_img-mean)/(std+1e-7)
            #input_img = (input_img-mean)/(std+1e-7)
            
            prediction = model_shape.predict(input_img)
            #print('Actual: ' + str(dirname) + '     detected: ' + shape_class[np.argmax(prediction)])
            print('detected: ' + shape_class[np.argmax(prediction)])
            cv2.imshow('image', cv2.resize(input_img_clone, (100,100)))
            cv2.waitKey(3000)

            #cv2.imshow('image', image)
            #cv2.waitKey(0)


    except KeyboardInterrupt:
        pass
    