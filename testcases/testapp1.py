####
#
# Test App 1 (Gaussian process sampling)
# 
# Test dynamic task cancellation in a Gaussian process example.
# 
# You can run this with 'beecluster' or 'baseline' 
# to see the performance difference.
#
####

import sys
sys.path.append('../python') # path to beecluster
import beecluster

from time import time, sleep 
import numpy as np 
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel, WhiteKernel
import json 
# wifi gp 

locations = json.load(open("testapp1_data.json"))
loc2taskname = {}
gptime = 0
sample_loc = []
sample_value = []

locs = np.zeros((len(locations),2))
done = np.zeros((len(locations)))


for i in range(len(locations)):
	locs[i,0] = locations[i][0]
	locs[i,1] = locations[i][1]


def GaussianProcessUpdate(std_threshold = 1.05):
	global sample_loc
	global sample_value
	global done 
	 
	kernel = ConstantKernel() + RBF(length_scale=1.0, length_scale_bounds=(1e1, 1e2)) + WhiteKernel(noise_level=0.01, noise_level_bounds=(1e-1, 0.5))

	gp = GaussianProcessRegressor(kernel = kernel, n_restarts_optimizer = 30)
	gp.fit(np.array(sample_loc), np.array(sample_value))

	y_mean, y_std = gp.predict(locs, return_std = True)

	done_mask = np.where((y_std<std_threshold)&(done<0.5))

	print("removed task",len(done_mask[0]))

	if len(sample_value) > 6:
		print("removed task",len(done_mask[0]))
		done[done_mask] = 1.0

	#Log(locs.tolist(), y_mean.tolist(), y_std.tolist(), done.tolist(), sample_loc, sample_value)
	
	if len(sample_value) > 6:
		return done_mask
	else:
		return None 


def SenseAndUpdate(bc, loc):
	global sample_loc
	global sample_value
	global loc2taskname
	global gptime

	bc.act("flyto", loc)

	h1 = bc.act("wifi:sense", None)
	h2 = bc.act("wifi:sense", None)
	h3 = bc.act("wifi:sense", None)
	
	value = (float(h1.val) + float(h2.val) + float(h3.val))/3.0
	print(loc, value)

	bc.Lock()

	sample_loc.append((loc[0], loc[1]))
	sample_value.append(value)

	t0 = time()
	removed_tasks = GaussianProcessUpdate()
	gptime += time() - t0

	bc.UnLock()

	if removed_tasks is not None:
		cancel_list = [loc2taskname[(locs[removed_tasks[0][i]][0], locs[removed_tasks[0][i]][1], 5)] for i in range(len(removed_tasks[0]))]
		bc.cancelTask(cancel_list)

	return 


if __name__ == "__main__":
	beecluster.ResetRemoteServer()

	bc = beecluster.Session(appID = "testapp1")

	# generate all the tasks (grid_size * grid_size)
	for i in range(len(locs)):
		loc = (locs[i][0], locs[i][1], 5)
		h = bc.newTask(SenseAndUpdate, bc, loc, NonInterruptible = True, SameDrone = True)
		loc2taskname[loc] = h.getTaskName()

	bc.waitUnfinishedTasks()

	bc.close(SaveDTG=False)

	print("duration", bc.getDuration())

