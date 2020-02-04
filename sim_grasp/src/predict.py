# -*- coding: utf-8 -*-

""" 
Predict grasp rectangle for an image named 'test.png'
"""

from __future__ import print_function, absolute_import

import tflearn
from tflearn.layers.core import input_data, dropout, fully_connected
from tflearn.layers.conv import conv_2d, max_pool_2d
from tflearn.layers.normalization import local_response_normalization, batch_normalization
from tflearn.layers.estimator import regression
from tflearn.metrics import *
import pickle
import torchfile
import numpy as np
import tensorflow as tf
from grasp_metric import *
import argparse
#from PIL import Image, ImageDraw
import PIL

green = (0, 255, 0)
red = (255, 0, 0)
blue = (0, 0, 255)

def draw_box(ps, draw, color1, color2): #p is point
    #ps = [p1, p2, p3, p4]
    for i in xrange(4): #index starts from 0
        x1, y1 = ps[i]
        x2, y2 = ps[(i+1) % 4]
        if i % 2 == 1: #i = 1 or i = 3 => p2 or p4, so p1 to p2 is white => width, p2 to p3 is green => height, ...
            draw.line((x1, y1, x2, y2), fill=color1, width=2)
        else: # i = 0 or i =2 => p1 or p3
            draw.line((x1, y1, x2, y2), fill=color2, width=2)

def convert(grasp):
    x = 0 #x position of center of rectangle
    y = 1 #y position of center of rectangle
    h = 2 #position of height
    w = 3 #position of width
    sinpos = 4
    cospos = 5
    #compute three corner points of the intersection rectangle
    graspangle = np.arctan2(grasp[sinpos],grasp[cospos])/2.
    
    #transform rectangle vertices to image coordinate
    rot1 = np.asarray([[np.cos(graspangle),-np.sin(graspangle)],[np.sin(graspangle),np.cos(graspangle)]])
    grasprect = np.asarray([
        (+grasp[w]/2.,+grasp[h]/2.),
        (+grasp[w]/2.,-grasp[h]/2.),
        (-grasp[w]/2.,-grasp[h]/2.),
        (-grasp[w]/2.,+grasp[h]/2.)])
    grasprect = np.dot(rot1, grasprect.transpose()).transpose()
    grasprect += [grasp[x],grasp[y]]
    rect_center = np.asarray(grasp[x], grasp[y]) # center of rectangle in image coordinate
    return grasprect, graspangle, rect_center

def rect_metric(prediction, target, inputs):
    with tf.name_scope('rect'):
       res = tf.reduce_mean(tf.cast(grasp_error(prediction, target),tf.float32))
    return res

def create_model(weights_path):
	# Building Network
	network = input_data(shape=[None, 227, 227, 3])

	network = conv_2d(network, 64, 5, strides=2, activation='relu')
	network = max_pool_2d(network, 2, strides=2) #no padding
	#network = batch_normalization(network)
	network = local_response_normalization(network) #local_response_normalization

	network = conv_2d(network, 128, 3, strides=2, activation='relu')
	network = max_pool_2d(network, 2, strides=2)
	#network = batch_normalization(network)
	network = local_response_normalization(network)
	
	network = conv_2d(network, 128, 3, activation='relu')
	
	network = conv_2d(network, 128, 3, activation='relu')
	
	network = conv_2d(network, 256, 3, strides=2, activation='relu')
	network = max_pool_2d(network, 2, strides = 2)
	#network = batch_normalization(network)
	network = local_response_normalization(network)
	
	network = fully_connected(network, 1024, activation='tanh')
	#network = dropout(network, 0.5)
	
	network = fully_connected(network,1024, activation='tanh')
	#network = dropout(network, 0.5)
	
	network = fully_connected(network, 6, activation='linear', name='fc') #, 6 , restore=False
	adam = tflearn.optimizers.Adam(learning_rate=0.001,  epsilon=0.1)
	network = regression(network, optimizer=adam, loss='mean_square',metric=rect_metric) #restore=False
	
	#
	model = tflearn.DNN(network) #grasp_net
	model.load(weights_path)
	
	return model
	#/home/chandan_main/Documents/tn_cong/Grap_Redmon/More_neurons/backup/1024/grasp_redmon-95120'
	
def predict_grasp (model, img):
	#img = Image.open('test.png')
	#if(len(img) != 227):
	#	print('Wrong image size! Should be 227x227')
	#	return 0
	img_arr = np.asarray(img).astype(float)
	
	img_norm = img_arr/255.0
	prediction = model.predict([img_norm])
	grasp_rect = convert(prediction[0])
	'''
	draw = ImageDraw.Draw(img)
	draw_box(grasp_rect, draw, blue, red)
	img.show()
	'''
	return (grasp_rect)
