import numpy as np
import tensorflow as tf
#import tflearn
from tensorflow.contrib.layers.python.layers import batch_norm
import random
import pickle 
import scipy.ndimage as nd 
import scipy 
import math
import svgwrite
from svgwrite.image import Image as svgimage
from PIL import Image


import tf_common_layer as common




# 240x240 --> 60x60 

class SegmentationModel():
	def __init__(self, sess, image_size=224):
		self.sess = sess 

		self.image_size = image_size
		self.BuildCNN()

		self.sess.run(tf.global_variables_initializer())
		self.saver = tf.train.Saver(max_to_keep=30)


		self.summary_loss = []
		self.test_loss =  tf.placeholder(tf.float32)
		self.train_loss =  tf.placeholder(tf.float32)

		self.summary_loss.append(tf.summary.scalar('loss/test', self.test_loss))
		self.summary_loss.append(tf.summary.scalar('loss/train', self.train_loss))

		self.merged_summary = tf.summary.merge_all()


		pass 


	def BuildCNN(self):    

		self.input = tf.placeholder(tf.float32, shape = [None, self.image_size, self.image_size, 3])
		self.target = tf.placeholder(tf.int32, shape = [None, self.image_size/4, self.image_size/4, 2])
		self.lr = tf.placeholder(tf.float32, shape=[])
		self.is_training = tf.placeholder(tf.bool)
		self.dropout = tf.placeholder(tf.float32)


		ch = 8

		conv1, _, _ = common.create_conv_layer('cnn_l1', self.input, 3, ch, kx = 5, ky = 5, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True)
		conv2, _, _ = common.create_conv_layer('cnn_l2', conv1, ch, ch*2, kx = 5, ky = 5, stride_x = 2, stride_y = 2, is_training = self.is_training, batchnorm = True)
		# 120*120 (ch*2)

		#conv3, _, _ = common.create_conv_layer('cnn_l3', conv2, ch*2, ch*2, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True)
		conv4, _, _ = common.create_conv_layer('cnn_l4', conv2, ch*2, ch*4, kx = 3, ky = 3, stride_x = 2, stride_y = 2, is_training = self.is_training, batchnorm = True)
		# 60*60*(ch*4)

		#conv5, _, _ = common.create_conv_layer('cnn_l5', conv4, ch*4, ch*4, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True)
		conv6, _, _ = common.create_conv_layer('cnn_l6', conv4, ch*4, ch*8, kx = 3, ky = 3, stride_x = 2, stride_y = 2, is_training = self.is_training, batchnorm = True)
		# 30*30*(ch*8)

		conv7, _, _ = common.create_conv_layer('cnn_l7', conv6, ch*8, ch*8, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True)
		conv8, _, _ = common.create_conv_layer('cnn_l8', conv7, ch*8, ch*16, kx = 3, ky = 3, stride_x = 2, stride_y = 2, is_training = self.is_training, batchnorm = True)
		# 15*15*(ch*16)


		#conv14_concat = tf.concat([conv14, conv8],axis=3)
		#conv15, _, _ = common.create_conv_layer('cnn_l15', conv14_concat, ch*32, ch*16, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True, deconv=False)
		conv16, _, _ = common.create_conv_layer('cnn_l16', conv8, ch*16, ch*16, kx = 3, ky = 3, stride_x = 2, stride_y = 2, is_training = self.is_training, batchnorm = True, deconv=True)
		conv17, _, _ = common.create_conv_layer('cnn_l17', conv16, ch*16, ch*8, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True, deconv=False)
		# 30*30*(ch*8)

		conv17_concat = tf.concat([conv17, conv6],axis=3)
		conv18, _, _ = common.create_conv_layer('cnn_l18', conv17_concat, ch*16, ch*8, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True, deconv=False)
		conv19, _, _ = common.create_conv_layer('cnn_l19', conv18, ch*8, ch*8, kx = 3, ky = 3, stride_x = 2, stride_y = 2, is_training = self.is_training, batchnorm = True, deconv=True)
		conv20, _, _ = common.create_conv_layer('cnn_l20', conv19, ch*8, ch*4, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True, deconv=False)
		# 60*60*(ch*4)

		# conv20_concat = tf.concat([conv20, conv4],axis=3)
		# conv21, _, _ = common.create_conv_layer('cnn_l21', conv20_concat, ch*8, ch*4, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True, deconv=False)
		# conv22, _, _ = common.create_conv_layer('cnn_l22', conv21, ch*4, ch*4, kx = 3, ky = 3, stride_x = 2, stride_y = 2, is_training = self.is_training, batchnorm = True, deconv=True)
		# conv23, _, _ = common.create_conv_layer('cnn_l23', conv22, ch*4, ch*2, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True, deconv=False)
		# # 192*192*(ch*2)

		# conv23_concat = tf.concat([conv23, conv2],axis=3)
		# conv24, _, _ = common.create_conv_layer('cnn_l24', conv23_concat, ch*4, ch*2, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True, deconv=False)
		# conv25, _, _ = common.create_conv_layer('cnn_l25', conv24, ch*2, ch*2, kx = 3, ky = 3, stride_x = 2, stride_y = 2, is_training = self.is_training, batchnorm = True, deconv=True)
		# conv26, _, _ = common.create_conv_layer('cnn_l26', conv25, ch*2, ch, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True, deconv=False)
		# # 384*384*ch

		conv20 = tf.nn.dropout(conv20, self.dropout)

		conv27, _, _ = common.create_conv_layer('cnn_l27', conv20, ch*4, 2, kx = 3, ky = 3, stride_x = 1, stride_y = 1, is_training = self.is_training, batchnorm = True, deconv=False, activation='linear')
		

		self.loss = tf.reduce_mean(tf.losses.softmax_cross_entropy(self.target, conv27))
		self.train_op = tf.train.AdamOptimizer(learning_rate=self.lr).minimize(self.loss)


		self.output = tf.nn.softmax(conv27)


	def Train(self, inputdata, target, lr):
		feed_dict = {
			self.input : inputdata,
			self.target : target,
			self.lr : lr,
			self.is_training : True,
			self.dropout : 0.7
		}

		return self.sess.run([self.loss, self.train_op], feed_dict=feed_dict)


	def Evaluate(self, inputdata, target):
		feed_dict = {
			self.input : inputdata,
			self.target : target,
			self.is_training : False,
			self.dropout : 1.0
		}

		return self.sess.run([self.loss, self.output], feed_dict=feed_dict)

	def saveModel(self, path):
		self.saver.save(self.sess, path)

	def restoreModel(self, path):
		self.saver.restore(self.sess, path)


	def addLog(self, test_loss, train_loss):
		feed_dict = {
			self.test_loss : test_loss,
			self.train_loss : train_loss
		}
		return self.sess.run(self.merged_summary, feed_dict=feed_dict)















