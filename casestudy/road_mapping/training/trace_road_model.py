from model import SegmentationModel
from dataloader import SegmentationDataLoader

from time import time 
import numpy as np 

import tensorflow as tf 

from PIL import Image 
import scipy 
import scipy.ndimage

import sys 
import math 
import cv2 


image_size = 224


def load_model(name="model20190919_2/model25000" ):

	sess = tf.Session(config=tf.ConfigProto())

	model = SegmentationModel(sess, image_size=image_size)
	model.restoreModel(name)

	return model



def apply_model(img_in, model=None):

	if model is not None:
		img_in = (scipy.misc.imresize(img_in, (240,320)).astype(np.float)/255.0) - 0.5 

		target = np.zeros((1,56,56,2))
		img_in = (img_in[120-112:120+112, 160-112:160+112,:]).reshape((1,224,224,3))

		t0 = time() 
		_, output = model.Evaluate(img_in, target)
		print(time() - t0)

		output = (output[0,:,:,0] * 255).reshape((56,56)).astype(np.uint8)

	else:
		with tf.Session(config=tf.ConfigProto()) as sess:
			model = SegmentationModel(sess, image_size=image_size)
			model.restoreModel("model20190919/model5000")

			img_in = (scipy.misc.imresize(img_in, (240,320)).astype(np.float)/255.0) - 0.5 

			target = np.zeros((1,56,56,2))
			img_in = (img_in[120-112:120+112, 160-112:160+112,:]).reshape((1,224,224,3))

			t0 = time() 
			_, output = model.Evaluate(img_in, target)
			print(time() - t0)

			output = (output[0,:,:,0] * 255).reshape((56,56)).astype(np.uint8)


	return output





def toWayPoint(result_in):

	angle_res = 32

	best_direction = -1
	best_direction_d = 100


	sample_x = 28
	sample_y = 28

	for i in range(angle_res):
		angle = i / float(angle_res) * 360.0  
		dx = math.cos(math.radians(angle))
		dy = math.sin(math.radians(angle))

		cur_x = sample_x
		cur_y = sample_y

		d = 0 

		while True:
			x = int(cur_x + d * dx)
			y = int(cur_y + d * dy)

			if x < 0 or x >= 56 or y < 0 or y >= 56 :
				break 

			if result_in[x,y] > 127 :

				# d1 = d 
				# d2 = d
				# d3 = d 


				# while True:
				# 	x = int(cur_x + d2 * dx)
				# 	y = int(cur_y + d2 * dy)

				# 	if x < 0 or x >= 56 or y < 0 or y >= 56 :
				# 		break 

				# 	if result_in[x,y] < 127 :
				# 		break
					
				# 	d2 += 1


				# while True:
				# 	x = int(cur_x + d3 * dx)
				# 	y = int(cur_y + d3 * dy)

				# 	if x < 0 or x >= 56 or y < 0 or y >= 56 :
				# 		break 

				# 	if result_in[x,y] < 127 :
				# 		break
					
				# 	d3 -= 1

				
				
				# d =  d + float(d2 - d3)/2.0

				break
			else:
				d = d + 1 


		if abs(d) < abs(best_direction_d):
			best_direction_d = d 
			best_direction = i  


	# if best_direction_d < 25 and best_direction_d > 0 :
	# 	best_direction_d += 1.5 



	angle = best_direction / float(angle_res) * 360.0  
	dx = math.cos(math.radians(angle))
	dy = math.sin(math.radians(angle))

	new_x = int(sample_x + dx * best_direction_d)
	new_y = int(sample_y + dy * best_direction_d)


	if new_x < 2:
		new_x = 2 
	if new_y < 2:
		new_y = 2 
	if new_x > 53:
		new_x = 53
	if new_y > 53:
		new_y = 53

	# mean-shift 

	for i in range(5):
		best_dxy = (0,0)
		best_dxy_score = 0

		for dxy in [(1,1),(-1,-1),(1,-1),(-1,1)]:
			x = new_x + dxy[0]
			y = new_y + dxy[1]

			s = np.sum(result_in[x-1:x+2, y-1:y+2])

			if s > best_dxy_score:
				best_dxy_score = s 
				best_dxy = (dxy[0], dxy[1])


		new_x += best_dxy[0]
		new_y += best_dxy[1]

		if new_x < 1 or new_x > 54 or new_y < 1 or new_y > 54:
			break 



	#print(best_direction_d, best_direction * 5.0)


	# search direction 

	best_direction = -1
	best_direction_d = 0

	for i in range(angle_res):
		angle = i / float(angle_res) * 360.0  
		dx = math.cos(math.radians(angle))
		dy = math.sin(math.radians(angle))

		if dx > 0 :
			continue

		cur_x = new_x
		cur_y = new_y

		d = 0 

		while True:
			x = int(cur_x + d * dx)
			y = int(cur_y + d * dy)

			if x < 0 or x >= 56 or y < 0 or y >= 56 :
				break 

			if result_in[x,y] <= 127 :
				break
			else:
				d = d + 1 

			#if d > 20:
			#	break 

		#print(angle, "dist", d)

		if d > best_direction_d:
			best_direction_d = d 
			best_direction = i  

	#print(best_direction_d, best_direction)

	if best_direction_d > 20:
		best_direction_d = 20 

	angle = best_direction / float(angle_res) * 360.0   
	dx = math.cos(math.radians(angle))
	dy = math.sin(math.radians(angle))

	new_x_next = int(new_x + dx * best_direction_d)
	new_y_next = int(new_y + dy * best_direction_d)





	return (new_x, new_y), (new_x_next, new_y_next), (28,28), angle 


def draw_arrow(img, p1, p2, color, width):
	dx = p2[0] - p1[0]
	dy = p2[1] - p1[1]

	l = np.sqrt(dx*dx + dy*dy)

	dx = dx/l * width * 3
	dy = dy/l * width * 3

	p3 = (p2[0] - dx, p2[1] - dy)

	p4 = (int(p3[0] - dy), int(p3[1] + dx))
	p5 = (int(p3[0] + dy), int(p3[1] - dx))


	cv2.line(img, p1, p2, color, width)
	cv2.line(img, p4, p2, color, width)
	cv2.line(img, p5, p2, color, width)



if __name__ == "__main__":

	counter = 0 
	model = load_model()

	for file in sys.argv[1:]:

		img = scipy.ndimage.imread(file)

		output = apply_model(img, model=model)

		v1,v2,v0, angle = toWayPoint(output)

		#print(v1,v2,v0)


		output = np.pad(output, 2, constant_values = 127)

		Image.fromarray(output).save("output/output%d_seg.jpg" % counter)

		img = scipy.ndimage.imread(file)


		def convert(v):
			return ((v[1] - 28) * 8 + 320, (v[0] - 28) * 8 + 240)
		 

		cv2.line(img, convert(v0), convert(v1), (255,0,0), 3)
		cv2.line(img, convert(v1), convert(v2), (255,0,0), 3)
		draw_arrow(img, convert(v0), convert(v2), (0,255,0), 5)

		cv2.line(img, (96, 16), (96+448, 16), (0,0,255), 2)
		cv2.line(img, (96, 16), (96, 16+448), (0,0,255), 2)
		cv2.line(img, (96+448, 16), (96+448, 16+448), (0,0,255), 2)
		cv2.line(img, (96, 16+448), (96+448, 16+448), (0,0,255), 2)



		Image.fromarray(img).save("output/output%d_img.jpg" % counter)


		counter += 1

		print(file, counter)





