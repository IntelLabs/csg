from __future__ import division
import os
import time
import math
import numpy as np
import tensorflow as tf
import tensorflow.contrib.slim as slim
from .net import Net

#import matplotlib as mpl
#import matplotlib.cm as cm
import cv2

from tensorflow.python.ops import control_flow_ops
from numba import cuda 

def disp_to_depth(disp, min_depth, max_depth):
    min_disp = 1. / max_depth
    max_disp = 1. / min_depth
    scaled_disp = tf.to_float(min_disp) + tf.to_float(max_disp - min_disp) * disp
    depth = tf.to_float(1.) / scaled_disp
    return depth

class DepthEstimator(object): 
    def __init__(self):
        config = dict()
        config['model'] = dict()
        config['dataset'] = dict()
        config['model']['batch_norm_decay'] =0.95
        config['model']['batch_norm_epsilon'] = 1e-5
        config['model']['batch_size'] = 1
        config['model']['pose_scale'] = 1e-2
        config['model']['num_source']=3
        config['model']['num_scales']=4
        config['dataset']['image_width'] = 640
        config['dataset']['image_height']=192
        config['dataset']['min_depth'] = 0.1
        config['dataset']['max_depth'] = 100.
        
        self.config = config
        self.preprocess = True
        self.min_depth = np.float(config['dataset']['min_depth'])
        self.max_depth = np.float(config['dataset']['max_depth'])
        
    def preprocess_image(self, image):
        image = (image - 0.45) / 0.225
        return image        
    
    def build_model(self, ckpt): 
        self.num_scales = self.config['model']['num_scales']
        self.num_source = self.config['model']['num_source']
        
        with tf.name_scope('data_loading'):
            self.tgt_image_uint8 = tf.compat.v1.placeholder(tf.uint8, [1,
                    self.config['dataset']['image_height'], self.config['dataset']['image_width'], 3])
            self.tgt_image = tf.image.convert_image_dtype(self.tgt_image_uint8, dtype=tf.float32)
            tgt_image_net = self.preprocess_image(self.tgt_image)
            
        with tf.variable_scope('monodepth2_model', reuse=tf.AUTO_REUSE) as scope:
            net_builder = Net(False, **self.config)
            res18_tc, skips_tc = net_builder.build_resnet18(tgt_image_net)
            pred_disp = net_builder.build_disp_net(res18_tc, skips_tc)
            pred_disp_rawscale = [tf.image.resize_bilinear(pred_disp[i], 
                                        [self.config['dataset']['image_height'], self.config['dataset']['image_width']]) for i in
                range(self.num_scales)]
            pred_depth_rawscale = disp_to_depth(pred_disp_rawscale, self.min_depth, self.max_depth)

            self.pred_depth = pred_depth_rawscale[0]
            self.pred_disp = pred_disp_rawscale[0]

        var_list = [var for var in tf.compat.v1.global_variables() if "moving" in var.name]
        var_list += tf.compat.v1.trainable_variables()
        saver = tf.compat.v1.train.Saver(var_list, max_to_keep=10)
        config = tf.compat.v1.ConfigProto()
        config.gpu_options.allow_growth = True
        self.sess = tf.compat.v1.Session(config=config)
        latest_ckpt = tf.train.latest_checkpoint('{}'.format(ckpt))
        saver.restore(self.sess,latest_ckpt)
    

    def estimate_image(self, bgr): 

        image = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        image = cv2.resize(image,(self.config['dataset']['image_width'],self.config['dataset']['image_height']),cv2.INTER_AREA)
        image =np.expand_dims(image,axis=0)
        tgt_image_np_batch = image
        fetches = {'depth': self.pred_depth,'disp': self.pred_disp}
        results = self.sess.run(fetches, feed_dict={self.tgt_image_uint8: tgt_image_np_batch})
        src_depth =np.squeeze(results['depth'])
        depth_resized = cv2.resize(src_depth,(bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_CUBIC)
        
        #import matplotlib.pyplot as plt
        #plt.imshow(depth_resized)
        
        return depth_resized

    def release(self): 

        self.sess.close()
        device = cuda.get_current_device()
        device.reset()
        return

