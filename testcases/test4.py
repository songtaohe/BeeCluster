####
#
# Test Case 4
# 
# Test the overhead of the runtime system - disptach 100 tasks. 
# 
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
def visit(bc, loc):
	bc.act("flyto", loc)
	bc.wait()


if __name__ == "__main__":
	bc = beecluster.Session(appID = "TEST-4")

	handles = []

	t0 = time()
	for x in range(-50,51,10):
		for y in range(-50,51,10):
			h = bc.newTask(visit, bc, (x,y,30), TaskName="visit")
			handles.append(h)

	t1 = time()

	results = [h.val for h in handles]
	t2 = time()

	bc.close(SaveDTG=False)

	print("dispatch 100 tasks", t1-t0)
	print("complete 100 tasks", t2-t1)

	print("######## TEST-4  PASSED ########")












