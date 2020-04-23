import matplotlib.pyplot as plt
import numpy as np 
import sys 
import scipy.interpolate
import scipy.misc
import scipy.ndimage

import math

from PIL import Image 

from subprocess import Popen 

import pickle 
import cv2 



profile = pickle.load(open("fisheye_profile.p", "rb"))

K = np.array(profile[0].tolist())
D = np.array(profile[1].tolist())

print(K)
print(D)

proc = []

for input_image in sys.argv[2:]:

	print(input_image)

	output_name = sys.argv[1]+"/"+input_image.split("/")[-1].replace(".jpg","_fixed.jpg")	


	img = cv2.imread(input_image)
	dim = np.shape(img)
	DIM_org = (dim[0], dim[1])

	img = scipy.misc.imresize(img, (1944, 2592))



	dim = np.shape(img)

	DIM = (dim[1], dim[0])

	# cv2.imshow("origin",img)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3)*2.0, K, DIM, cv2.CV_16SC2)

	undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

	#undistorted_img = cv2.fisheye.undistortImage(img, K, D)


	# cv2.imshow("undistorted",undistorted_img)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()


	undistorted_img = scipy.misc.imresize(undistorted_img, DIM_org)


	cv2.imwrite(output_name, undistorted_img)
	#Image.fromarray(undistorted_img).save(output_name)




	# proc.append(Popen("python fisheye.py %s %s" % (input_image, output_name), shell=True))


	# if len(proc) > 12 :
	# 	proc[0].wait()

	# 	proc = proc[1:]
	


# for p in proc :
# 	p.wait()
