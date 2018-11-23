import cv2 
import numpy as np 
import matplotlib.pyplot as plt
import os
import ast
from sklearn.model_selection import train_test_split
from sklearn import datasets, svm, metrics
from sklearn.neural_network import MLPRegressor
'''
COLOR CODE: 
0:  Yellow
1:  Green
2:  Orange
3:  Red
4:  Blue
5:  Purple
'''
PATH_DATA = "../image_dataset-master-cropped/"
color_class = ['Yellow', 'Green', 'Orange', 'Red', 'Blue', 'Purple']

def load_custom_data(color_label):
    x_list = []
    y_list = []

    # x_test_list = []
    # y_test_list = []

    ### LOAD Training DATA  ###
    print('##########    LOADING DATA     ##########')
    for dirname in os.listdir(PATH_DATA):
        legit = 0
        if dirname == 'Yellow Cube' or dirname == 'Yellow Ball':
            if color_label == 0:
                legit = 1
        if dirname == 'Green Cube' or dirname == 'Green Hollow Cube' or dirname == 'Green Cylinder':
            if color_label == 1:
                legit = 1
        if dirname == 'Orange Cross' or dirname == 'Orange Star':
            if color_label == 2:
                legit = 1
        if dirname == 'Red Cube' or dirname == 'Red Cylinder' or dirname == 'Red Hollow Cube':
            if color_label == 3:
                legit = 1
        if dirname == 'Blue Cube' or dirname == 'Blue Triangle':
            if color_label == 4:
                legit = 1
        if dirname == 'Purple Cross' or dirname == 'Purple Star':
            if color_label == 5:
                legit = 1

        if legit:
            print('##########    ' + dirname + '      ##########')

            with open(PATH_DATA + dirname + "/hsv_data_1.txt", 'r') as f:
                hsv_data = ast.literal_eval(f.read())

            f.close()

            with open(PATH_DATA + dirname + "/image_name_data_1.txt", 'r') as f:
                image_name_data = ast.literal_eval(f.read())

            f.close()

            for j in range(len(hsv_data)):
                try:
                    image_path = PATH_DATA + dirname + '/' + image_name_data[j]
                    if os.path.exists(image_path):
                        image = cv2.imread(PATH_DATA + dirname + '/' + image_name_data[j])
                        image_reduced = cv2.resize(image, (32,32))
                        x_list.append(image_reduced.reshape(-1))
                        y_list.append(hsv_data[j]) 
                except:
                    pass

            # for file in glob.glob(PATH_DATA + dirname + "/*.jpg"):

            #     image = cv2.imread(file)

            #     x_list.append(cv2.resize(image, (32,32)))
            #     y_list.append(label)
            
    x_arr = np.array(x_list)
    #y_arr = np.reshape(np.array(y_list), (-1,1))
    y_arr = np.array(y_list)

    x_train, x_test, y_train, y_test = train_test_split(x_arr, y_arr, test_size=0.3, random_state=0)
    return (x_train, y_train), (x_test, y_test) 

regressor = []

for i in range(6):
    print('Fitting model for color ' + color_class[i])
    (x_train, y_train), (x_test, y_test) = load_custom_data(i)

    print('Training data and target sizes: \n{}, {}'.format(x_train.shape,y_train.shape))
    print('Test data and target sizes: \n{}, {}'.format(x_test.shape,y_test.shape))

    #Normalize
    x_train = x_train.astype('float32')
    x_test = x_test.astype('float32')

    mean = np.mean(x_train,axis=(0,1))
    std = np.std(x_train,axis=(0,1))

    x_train = (x_train-mean)/(std+1e-7)
    x_test = (x_test-mean)/(std+1e-7)
    # mean = np.mean(x_train,axis=1)
    # mean_broadcasted = (np.array([mean,]*x_train.shape[1])).T

    # std = np.std(x_train,axis=1)
    # x_train = (x_train-mean_broadcasted)/(np.reshape(std,(-1,1))+1e-7)
    # x_test = (x_test-mean_broadcasted)/(np.reshape(std,(-1,1))+1e-7)


    regressor.append(MLPRegressor())
    regressor[i].fit(x_train,y_train)

    score = regressor[i].score(x_test,y_test)
    print('Score for color ' + color_class[i] + ' is: ' + str(score))

print('seems okay')
# # Create a classifier: a support vector classifier
# classifier = svm.SVC(gamma=0.001)
# #fit to the trainin data
# classifier.fit(x_train,y_train)

# # now to Now predict the value of the digit on the test data
# y_pred = classifier.predict(x_test)

# print("Classification report for classifier %s:\n%s\n"
#       % (classifier, metrics.classification_report(y_test, y_pred)))