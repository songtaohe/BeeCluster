import sys
sys.path.append('../../python') # path to beecluster
sys.path.append('training') # path to the training folder 
import beecluster
import trace_road_model

from time import sleep, time 
import base64
import math 
import json 
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import scipy 
import scipy.ndimage
import cv2 
from datetime import datetime
from subprocess import Popen
from PIL import Image 

counter = 0
foldername = datetime.today().strftime('%Y-%m-%d-%H-%M-%S')
Popen("mkdir %s "%foldername, shell=True).wait()
Popen("mkdir -p tmp", shell=True).wait()
Popen("rm tmp/*", shell=True).wait()

CNNModel = trace_road_model.load_model("training/model20190919_2/model25000")

def process_image_string(imgstr):
	global counter 
	global foldername
	

	with open(foldername+"/raw_image%d.jpg" % counter, 'wb') as fout:
		fout.write(base64.b64decode(imgstr))

	# fix the fisheye effect 
	Popen("python fisheye_fix.py %s %s" % (foldername, foldername+"/raw_image%d.jpg" % counter), shell=True).wait() 

	img = scipy.ndimage.imread(foldername+"/raw_image%d_fixed.jpg" % counter)

	Popen("cp " + foldername+"/raw_image%d_fixed.jpg" % counter + " tmp/output%d_img.jpg" % counter, shell=True).wait()

	return img 

	
def compute_next_waypoint(img, states):
	global CNNModel
	global counter 
	global foldername

	segmentation = trace_road_model.apply_model(img, CNNModel)
	
	p1, p2, p0, angle = trace_road_model.toWayPoint(segmentation)

	
	photo_yaw = float(states[4])

	vec = [p2[0] - p0[0], p2[1] - p0[1]]
	l = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1])
	vec[0] /= l 
	vec[1] /= l 

	global_dx = vec[1] * math.cos(math.radians(photo_yaw + 90)) - vec[0] * math.sin(math.radians(photo_yaw + 90))
	global_dy = vec[0] * math.cos(math.radians(photo_yaw + 90)) + vec[1] * math.sin(math.radians(photo_yaw + 90))  

	global_dyaw = 180.0 - angle 

	# log 
	json.dump([p1,p2,p0,angle,global_dx,global_dy], open("tmp/log%d.json" % counter, "w"))
	Image.fromarray(segmentation).save("tmp/output%d_seg.jpg" % counter)


	return (global_dx, global_dy), global_dyaw

def _parse_result(ret, loc, yaw, stepsize):
	global counter 
	global foldername

	# parse the result
	imgstr = ret.split("|")[-1].split("\"")[0]
	states = ret.split("|")[0:-1]

	# decode image, fix projection
	img = process_image_string(imgstr)

	# run CNN model, compute next way point
	dLoc, dyaw = compute_next_waypoint(img, states)

	# compute the next waypoint
	dLoc = (dLoc[0] * stepsize, dLoc[1] * stepsize)
	new_loc = (loc[0] + dLoc[0], loc[1] + dLoc[1], loc[2])

	# save json 
	data = [states, loc, yaw, dLoc, dyaw]
	json.dump(data, open(foldername+"/log%d.json" % counter, "w"))
	counter += 1

	new_yaw = yaw + dyaw 

	if new_yaw > 180:
		new_yaw -= 360

	if new_yaw < -180:
		new_yaw += 360 

	return new_loc, yaw + dyaw 


def trace_road(bc, loc, yaw):

	# fly to 'loc' and change yaw
	bc.act("set_yaw_" + str(yaw))
	bc.act("flyto", loc)
	
	ret = bc.act("take_photo_fast").val

	return ret 


if __name__ == "__main__":
	bc = beecluster.Session(appID = "trace_road")

	loc = initial_position = (-30,0,10)
	yaw = initial_yaw = -60 
	stepsize = 6.0

	# trace 25 steps  
	for i in range(1,25):
		ret =  bc.newTask(trace_road, bc, loc, yaw, stepsize)

		loc, yaw = _parse_result(ret, loc, yaw, stepsize)
		
		print("iteration: %d road location: (%.2f, %.2f) heading: %.2f" % (i, loc[0], loc[1], yaw))


	bc.close()







