####
#
# Test App 0 (Tracing a road)
# 
# Test dynamic task creation and the speculative execution feature.
# 
# You can run this with one drone and two drones 
# to see the performance difference.
#
####

import sys
sys.path.append('../python') # path to beecluster
import beecluster

from time import time, sleep 


# tracing roads 

def sense(bc, loc):
	bc.act("flyto", loc)
	bc.act("gp:sense", None)
	bc.wait()
	#sleep(1.0)

	return 

bc = beecluster.Session(appID = "testapp0")

t0 = time()
loc = (0,-80,10)

for i in range(25):
	print("iteration ",i)
	h = bc.newTask(sense, bc, loc)
	h.val 
	if i == 0:
		t0 = time()
	loc = (loc[0], loc[1] + 6, loc[2])

print("duration real", time() - t0)
print("duration sim", bc.getDuration())

bc.close()

