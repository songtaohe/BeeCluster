####
#
# Test Case 3
# 
# Test continous operation (task hand-off)
# Requires at least two drones in the simulator.
#
####

import sys
sys.path.append('../python') # path to beecluster
import beecluster
from time import time, sleep 
import math 
import numpy as np 


beecluster.ResetRemoteServer()
sleep(0.1)

# test non-same drone but non-interruptible 
def circle(bc, r):
	interval = []

	for i in range(32*4):
		alpha = 2*3.1415926/32.0*i
		bc.act("flyto", (r * math.cos(alpha), r * math.sin(alpha),30))
		h = bc.act("gp:sense", None)
		h.val 
		if i > 3:
			interval.append(time()-t0)

		t0 = time()
		print("circle point #%d"%i)

	bc.wait()

	print("max interval: ", np.amax(interval))
	print("average interval: ",np.mean(interval))
	print("interval std: ", np.std(interval))

	return (np.amax(interval), np.mean(interval), np.std(interval))



if __name__ == "__main__":
	bc = beecluster.Session(appID = "TEST-3")

	h = bc.newTask(circle, bc, 80, TaskName="circle2", NonInterruptible = True, SameDrone = False)

	result = h.val 
	
	bc.close()


	assert result[0] < result[1]*3.0, "######## TEST-3  FAILED ######## Hand-off too slow!"


	print("######## TEST-3  PASSED ########")












