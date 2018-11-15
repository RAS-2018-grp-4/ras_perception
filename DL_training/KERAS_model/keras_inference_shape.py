import cv2
import numpy as np
import keras.models 
import glob
from datetime import datetime

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
VIDEO_INFERENCE = 0
IMG_INFERNECE = 1

inference_model = keras.models.load_model('saved_models/keras_RAS_model_shape_2.h5')

if VIDEO_INFERENCE:    
    #cap = cv2.VideoCapture('/home/driverless/ras_perception/DL_training/ras_labeling')
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        a = datetime.now()

        ret, image = cap.read()

        if ret:
            input_img = []
            input_img.append(cv2.resize(image, (32,32)))
            input_img = np.array(input_img)

            prediction = inference_model.predict(input_img)
            print(prediction)

            cv2.imshow('image', image)
            cv2.waitKey(10)

            b = datetime.now()
            c = b - a
            fps = 1.0/(c.total_seconds())
            print('## FPS: ' + str(fps))
            print('')

elif IMG_INFERNECE:
    try:
        while True:
            label = np.random.randint(0,7)

            if label == 0:
                dirname = 'Ball'       
            elif label == 1:
                dirname = 'Cube'
            elif label == 2:
                dirname = 'Cylinder'
            elif label == 3:
                dirname = 'Hollow Cube'
            elif label == 4:
                dirname = 'Cross'
            elif label == 5:
                dirname = 'Triangle'
            elif label == 6:
                dirname = 'Star'

            for file in glob.glob(PATH_TEST + dirname + "/*.jpg"):
                image = cv2.imread(file)

                input_img = []
                input_img.append(cv2.resize(image, (32,32)))
                input_img = np.array(input_img)

                prediction = inference_model.predict(input_img)
                print('Actual: ' + str(dirname) + '     detected: ' + str(prediction))
                cv2.waitKey(3000)

                #cv2.imshow('image', image)
                #cv2.waitKey(0)


    except KeyboardInterrupt:
        pass
    