'''
Created on 24.02.2018

@author: mitschen@gmail.com
github: www.github/mitschen/CarND-Capstone/ros/src/tl_detector/light_classification
'''

import random
############################################################################
#NOTE: not available in the Udacity environment of CarND-Capstone
############################################################################
#import sklearn
import numpy as np
import tensorflow as tf
import os
import scipy.ndimage  
import scipy.misc


default_graph_path = './tensor/linux_tensor0.999'

#Used for Training & Classification
#Reusing the LeNet architecture from the TrafficSignClassifier with
#some small adaptation. I've added an additional FC 
def Lenet(features, keep_prob):
    mu = 0.
    sigma = 0.1
    
    
    conv1W       = tf.Variable(tf.truncated_normal(shape=(5,5,3,32), mean = mu, stddev = sigma), name='conv1_W')
    conv1B       = tf.Variable(tf.zeros(32), name = 'conv1_B')
    conv1        = tf.nn.conv2d(features, conv1W, (1,1,1,1), padding='VALID') + conv1B
    conv1        = tf.nn.relu(conv1)
    conv1        = tf.nn.max_pool(conv1, ksize=(1,2,2,1), strides=(1,2,2,1), padding='VALID')
    
    conv2W       = tf.Variable(tf.truncated_normal(shape=(5,5,32,43), mean = mu, stddev = sigma), name='conv2_W')
    conv2B       = tf.Variable(tf.zeros(43), name = 'conv2_B')
    conv2        = tf.nn.conv2d(conv1, conv2W, (1,1,1,1), padding='VALID') + conv2B
    conv2        = tf.nn.relu(conv2)
    conv2        = tf.nn.max_pool(conv2, ksize=(1,2,2,1), strides=(1,2,2,1), padding='VALID')

    fc0 = tf.contrib.layers.flatten(conv2)
    noOut = int(fc0.get_shape()[1])
    #84
    fc1Cnt = 84
    fc1W = tf.Variable(tf.truncated_normal(shape=(noOut, fc1Cnt), mean = mu, stddev = sigma), name='fc1_W')
    fc1B = tf.Variable(tf.zeros(fc1Cnt), name='fc1_b')
    
    fc1 = tf.matmul(fc0, fc1W,) + fc1B
    fc1 = tf.nn.relu(fc1)
    fc1 = tf.nn.dropout(fc1, keep_prob)
    
    #42
    fc2Cnt = 42
    fc2W = tf.Variable(tf.truncated_normal(shape=(fc1Cnt, fc2Cnt), mean = mu, stddev = sigma), name='fc2_W')
    fc2B = tf.Variable(tf.zeros(fc2Cnt), name='fc2_b')
    
    fc2    = tf.matmul(fc1, fc2W) + fc2B
    fc2    = tf.nn.relu(fc2)
    fc2    = tf.nn.dropout(fc2, keep_prob)
    
    fc3Cnt = 21
    fc3W = tf.Variable(tf.truncated_normal(shape=(fc2Cnt, fc3Cnt), mean = mu, stddev = sigma), name='fc3_W')
    fc3B = tf.Variable(tf.zeros(fc3Cnt), name='fc3_b')

    fc3    = tf.matmul(fc2, fc3W) + fc3B
    fc3    = tf.nn.relu(fc3)
    fc3    = tf.nn.dropout(fc3, keep_prob)
    
    fc4Cnt = 3
    fc4W = tf.Variable(tf.truncated_normal(shape=(fc3Cnt, fc4Cnt), mean = mu, stddev = sigma), name='fc4_W')
    fc4B = tf.Variable(tf.zeros(fc4Cnt), name='fc4_b')
    
    logits = tf.matmul(fc3, fc4W) + fc4B
    return logits


###############################################################################
# What is an image element 
# [ numpy.array of the image, label ]
# the numpy array of the image is a 32x32x3 bit image in rgb
# the label is a number of 0-red, 1-yellow, 2-green
#
# Please note I refer to the image element several times below
###############################################################################


#Used for Training
#Expects a folderpath. The function will iterate through all subfolders and 
#searching for image files.
#As a result it returns a list of lists containing
#[ RED-Img, YELLOW-Img, GREEN-Img ]
#Img contains the image element as well as the label (0=red, ..., 2=green)
#All images getting scaled to 32x32px and the alpha is removed from the image
#Returns a list of dataarrays of image elements
def importCustomImages(filepath):
    resultingImages = [ ]
    data = [ [], [], [] ]
    redCnt = 0
    yelCnt = 0
    greCnt = 0
    for subdir, dir, files in os.walk(filepath):
        for file in files :
            if file.endswith((".png", ".jpg", ".jpeg")):
                fqp = os.path.join(subdir, file)
                if -1 != fqp.find("red"):
                    data[0].append((fqp, 0))
                    redCnt += 1
                elif -1 != fqp.find("yellow"):
                    data[1].append( (fqp, 1) )
                    yelCnt += 1
                elif -1 != fqp.find("green"):
                    data[2].append( (fqp, 2) )
                    greCnt +=1
    
    for i in range(3):
        if 0!=len(data[i]):
          newImages = []
          for file in data[i]:
            #resize to 32/32 px and remove alphachannel if availble as well
            newImages.append( (np.array(scipy.misc.imresize(scipy.misc.imread(file[0]), (32,32))[:,:,:3], dtype=np.float32)
                                        , file[1]) )
          resultingImages.append(newImages)
          colorVal = [ "red", "yellow", "green"]
          print("Total of {0} samples for {1}".format(len(newImages), colorVal[newImages[0][1]] ))
    return resultingImages

#Used for Training 
#Expects a dataarray of image elements - for each image it applies normalization
#pushing the mean to zero with a std-deviation of 1.
#PLEASE NOTE: seems like the TL-classifier doesn't really like this normalization
#             at all. The results in matching rate were very bad. I guess this
#             is because of the "relu" activation which doesn't like color
#             values below 128
#
#LONG STORY SHORT: DO NOT NORMALIZE
def normalizeZeroMeanData(dataarray):
    for data in dataarray:
        data[0] -= 128.
        data[0] /= 128.
    return dataarray

#Used for Training
#Expects list of dataarrays of image elements - makes sure that all labels are
#given with same numbers. Therefore this function randomly chooses an
#element e.g. yellow and appends it  
def dataNormalizeCnts(val):
    val.sort(key = lambda x : len(x), reverse=True)
    fillUpTo = len(val[0])
    for i in range (1, len(val)):
        while len(val[i]) < fillUpTo:
            val[i].append(random.choice(val[i]))
    return val

#Used for Training
#Expects a dataarray of image elements and applies some augmentation on these images.
#In details - each image is appended with
# flipped l-r-
# rotated +/-15 degree
# rotated +/-90 degree
# shifted 2,2 px to lower, right/ upper left
# This function will result in 14 times more pictures
def dataAugmentation(dataarray):
    resultingData = []
    #each image is now
    #turned for 15 degreed to the right and to the left
    #shifted by 2 px to the bottom right / top left
    #for each of this images, the x axis is turned
    for data in dataarray:
        img = data[0]
        label = data[1]
        resultingData.append(data)
        resultingData.append( (np.fliplr(resultingData[-1][0]), label) )
        resultingData.append( (scipy.ndimage.interpolation.rotate(img, 15., reshape=False, mode='nearest'), label))
        resultingData.append( (np.fliplr(resultingData[-1][0]), label) )
        resultingData.append( (scipy.ndimage.interpolation.rotate(img, 90., reshape=False, mode='nearest'), label))
        resultingData.append( (np.fliplr(resultingData[-1][0]), label) )
        resultingData.append( (scipy.ndimage.interpolation.shift(img,  (2., 2., 0.), mode='nearest'), label) )
        resultingData.append( (np.fliplr(resultingData[-1][0]), label) )
        resultingData.append( (scipy.ndimage.interpolation.rotate(img, -15., reshape=False, mode='nearest'), label))
        resultingData.append( (np.fliplr(resultingData[-1][0]), label) )
        resultingData.append( (scipy.ndimage.interpolation.rotate(img, -90., reshape=False, mode='nearest'), label))
        resultingData.append( (np.fliplr(resultingData[-1][0]), label) )
        resultingData.append( (scipy.ndimage.interpolation.shift(img,  (-2., -2., 0.), mode='nearest'), label) )
        resultingData.append( (np.fliplr(resultingData[-1][0]), label) )
    return resultingData

#For Verification only
#Uses the LeNet architecture in combination with a given tensor-graph (flepath) 
#and verifies the matching rate on a dataarray of image elements.
#The function will write down the Accuracy of matching to the console
def loadCNNAndVerify(filepath, val):
    batchsize = 128
    x = tf.placeholder(tf.float32, (None, 32, 32, 3))
    y = tf.placeholder(tf.int32, (None))
    keep_prob = tf.constant(1.)
    logits = Lenet(x, keep_prob)
    correct_prediction = tf.equal(tf.argmax(logits, 1), tf.argmax(tf.one_hot(y, 3), 1))
    accuracy_operation = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
    
    total_accuracy = 0.
    
    X = np.array(val)[...,0].tolist()
    Y = np.array(val)[...,1].tolist()
    
    num_examples = len(X)
    
    
    with tf.Session() as sess:
        tf.train.Saver().restore(sess, filepath)
        
        for offset in range(0, num_examples, batchsize):
            batch_x, batch_y = X[offset:offset+batchsize], Y[offset:offset+batchsize]
            accuracy = sess.run(accuracy_operation, feed_dict={x:batch_x, y:batch_y})
            total_accuracy += (accuracy * len(batch_x))
        
    print("Total accuracy of {0} samples is {1:.3f}".format(num_examples, total_accuracy / num_examples))
        
        
#For Training only
#This is the main-function which trains the CNN (LeNet)
#As input it expects a dataaray of image elements. From this array
#it will chose a subset of training/ validation and test-data
#If the training results in an accuracy of about 90%, the function writes
#the tensor-graph (& weights) to the ./tensor folder.
def trainCNN(data):
    rate = 0.0005
    epochs = 30
    batchsize = 128
    keep_probability = 0.6
    
    input_layer  = tf.placeholder(tf.float32, (None, 32, 32, 3))
    inputY = tf.placeholder(tf.int32, (None))
    labels = tf.one_hot(inputY, 3)
    keep_prob = tf.placeholder(tf.float32)
    
    logits = Lenet(input_layer, keep_prob)
    
    cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=labels))
    optimizer = tf.train.AdamOptimizer(learning_rate=rate).minimize(cost)
    
    # Accuracy
    correct_pred = tf.equal(tf.argmax(logits, 1), tf.argmax(labels, 1))
    accuracy = tf.reduce_mean(tf.cast(correct_pred, tf.float32))
    
    # Initializing the variables
    init = tf. global_variables_initializer()
    ############################################################################
    #NOTE: not available in the Udacity environment of CarND-Capstone
    ############################################################################
    #in_samples, in_labels = sklearn.utils.shuffle(np.array(data)[...,0],np.array(data)[...,1])
    #Instead do not shuffle
    in_samples, in_labels = np.array(data)[...,0],np.array(data)[...,1]
    
    noTraining = int(len(in_samples) * 0.52)
    noValidation = int(len(in_samples) * 0.12)
    noTest = int(len(in_samples) * 0.36)

    #Split the samples into training/verification and test
    #Please note: i've trained always from the scratch - so it didn't
    #             really hurt to chose the test-data always again
    training_x = in_samples[:noTraining].tolist()
    training_y = in_labels[:noTraining].tolist()
    validation_x = in_samples[noTraining:noTraining+noValidation].tolist()
    validation_y = in_labels[noTraining:noTraining+noValidation].tolist()
    test_x = in_samples[noTraining+noValidation:].tolist()
    test_y = in_labels[noTraining+noValidation:].tolist()
    
    print("Training {0} Validation {1} Test {2} - {3}".format(noTraining, noValidation, noTest, len(data)), str(type(training_x)))
    print(training_x[0].shape)
    
    # Launch the graph
    with tf.Session() as sess:
        sess.run(init)
 
        for epoch in range(epochs):
            stopLoop = False
            batchIdx = 0
            #each epoch, shuffle the training data again
            ############################################################################
            #NOTE: not available in the Udacity environment of CarND-Capstone
            ############################################################################
#             training_x, training_y = sklearn.utils.shuffle(training_x, training_y)
            while(not stopLoop):
                start = batchIdx * batchsize
                end = min((batchIdx+1) * batchsize, len(training_x))
                batch_x = training_x[start: end]
                batch_y = training_y[start:end]
                stopLoop = end==len(training_x)
                sess.run(optimizer, feed_dict={
                    input_layer: batch_x,
                    inputY: batch_y,
                    keep_prob: keep_probability})
     
                batchIdx += 1
            #do validation check after each epoch    
            valid_acc = sess.run(accuracy, feed_dict={
                input_layer: validation_x[:],
                inputY: validation_y[:],
                keep_prob : 1.})
            print('Epoch {:>2}, Validation Accuracy: {:.6f}'.format(epoch+1, valid_acc))
     
        # Calculate Test Accuracy
        test_acc = sess.run(accuracy, feed_dict={
            input_layer: test_x[:],
            inputY: test_y[:],
            keep_prob: 1.})
        print('Testing Accuracy: {}'.format(test_acc))
        if(test_acc > 0.9):
            tf.train.Saver().save(sess, './tensor/linux_tensor{:.3f}'.format(test_acc))
    
    
class TrafficLightClassifier(object):
    def __init__(self, filepath = default_graph_path):
        self.path = os.path.abspath(filepath)
#         print(filepath)
#         print(tf.__version__)
        self.session = None 
        self.x = tf.placeholder(tf.float32, (None, 32, 32, 3))
        self.rate = tf.constant(1.)
        self.classifier = tf.argmax(Lenet(self.x, self.rate), 1)
        self.session = tf.InteractiveSession()
        tf.train.Saver().restore(self.session, self.path)
                
    def classifyImageFromPath(self, path):
        return self.classifyImage(scipy.misc.imread(path))
    
    def classifyImage(self, img):
        ##resize
        image = np.array(scipy.misc.imresize(img, (32,32))[:,:,:3], dtype=np.float32)
        ##normalize
        #As mentioned above (see normalizeZeroMeanData) don't do that
        #for traffic lights
#         image -= 128.
#         image = image.astype(np.float32) / 128.

        #our tensor expects an array
        image = [image]

        #start interactive session if not yet existing
        if self.session is None:
            self.session = tf.InteractiveSession()
            tf.train.Saver().restore(self.session, self.path)
        return self.session.run(self.classifier, feed_dict={self.x:image})[0]
     
# used for Training, Verification and testing    
if __name__ == '__main__':
    train = False   #train the CNN
    verify = True   #verify the CNN on a given set
    test = True     #test the TrafficLightClassifier for a given graph
    #this folder contains a bunch of testdata i've found and downloaded
    #here https://github.com/udacity/iSDC-P5-traffic-light-classifier-starter-code
    #Furthermore i've used a bunch of TrafficLight images from the simulator
    #to train the classifier
    #img_sourcefolder = "C:/Users/micha/Desktop/Udacity/Last/training"
    img_sourcefolder = "/home/student/Pictures/training"
    #img_sourcefolder = "/home/student/Udacity/CarND-Capstone/ros/src/tl_detector/misclassified"
    #path to an already trained tensor graph
    tensor_sourcepath=default_graph_path#'../tensor/linux_tensor0.999'
    if train:
        #read tl-images and apply labels
        val = importCustomImages(img_sourcefolder)
        #adjust the number of element counts so that each
        #traffic light occurs the same time
        val = dataNormalizeCnts(val)
        #concatenate all images from a list of lists to a list of images
        val = np.concatenate(val)
        #Don't do normalization!!!!
        #val = normalizeZeroMeanData(val)
        #Apply augmentation to the dataset
        val = dataAugmentation(val)
        if 0!=len(val):
          #start training
          trainCNN(val)
        else:
          print("No data for training")
        exit(0)

    if verify:
        val = importCustomImages(img_sourcefolder)
        val = np.concatenate(val)
        if(0 != len(val)):
          loadCNNAndVerify(tensor_sourcepath, val)
        else:
          print("No data for validation")
        exit(0)
        
    if test:
        color = [ 'RED', 'YELLOW', 'GREEN']
        test = TrafficLightClassifier(tensor_sourcepath)
        
        #pathes used in windows
        sim_green_path   = 'C:/Users/micha/Desktop/Udacity/Last/training/sim_samples/green'
        sim_red_path     = 'C:/Users/micha/Desktop/Udacity/Last/training/sim_samples/red'
        sim_yellow_path  = 'C:/Users/micha/Desktop/Udacity/Last/training/sim_samples/yellow'
        real_green_path  = 'C:/Users/micha/Desktop/Udacity/Last/training/green'
        real_red_path    = 'C:/Users/micha/Desktop/Udacity/Last/training/red'
        real_yellow_path = 'C:/Users/micha/Desktop/Udacity/Last/training/yellow'
        cnt = 0    
        for subdir, dir, files in os.walk(real_yellow_path):
            for file in files :
                if file.endswith((".png", ".jpg", ".jpeg")):
                    print(color[test.classifyImageFromPath(os.path.join(subdir, file))])
                    cnt += 1
        print( "Total "+str(cnt))
    
