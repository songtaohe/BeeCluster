####
#
# Test Case 1
# 
# Test if the act api is async
# Requires at least one drone in the simulator
#
####

import sys
sys.path.append('../python') # path to beecluster
#from beecluster import bc
import beecluster
from time import time, sleep 


# action queue test
def func1(sess):
	#bc.enterCodeBlock() 

	t0 = time() 
	bc.act("flyto", (10,10,30))
	bc.act("flyto", (80,10,30))
	bc.act("flyto", (80,80,30))
	bc.act("flyto", (10,80,30))
	h = bc.act("flyto", (10,10,30))
	
	t1 = time()
	print("Dispatch 5 Actions Time:", t1 - t0)
	ret = h.val
	#ret = "aaa"
	t2 = time() 
	print("All actions completed",t2 - t1)

	#bc.exitCodeBlock() 

	assert t2-t1>t1-t0, "######## TEST-1  FAILED ######## Act primitive seems not to be non-blocking"

	return ret 



if __name__ == "__main__":
	bc = beecluster.Session(appID = "TEST-1")

	print("pass-1")
	h = bc.newTask(func1, bc)
	print(h.val)

	print("pass-2")
	h = bc.newTask(func1, bc)
	print(h.val)

	print("pass-3")
	h = bc.newTask(func1, bc)
	print(h.val)

	print("######## TEST-1  PASSED ########")

	bc.close(SaveDTG = True)