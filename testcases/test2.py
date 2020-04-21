####
#
# Test Case 2
# 
# Test virtual drone to physical drone binding relationship.
# Requires at least two drones in the simulator.
#
####

import sys
sys.path.append('../python') # path to beecluster
import beecluster 
from time import time, sleep 
import math 

# 
def visitOnePoint(bc, loc):
	bc.act("flyto", loc)

	bc.wait() 

	return

# test same drone but interruptible
def box(bc, n):
	bc.act("flyto", (10,10,30))
	
	for i in range(n):
		bc.act("flyto", (80,10,30))
		bc.act("flyto", (80,80,30))
		bc.act("flyto", (10,80,30))
		bc.act("flyto", (10,10,30))
		bc.wait() 

		print("box #%d",i)

	return h.val 


# test non-same drone but non-interruptible 
def circle(bc, r):
	for i in range(32):
		alpha = 2*3.1415926/32.0*i
		h = bc.act("flyto", (r * math.cos(alpha), r * math.sin(alpha),30))
		h.val 
		print("circle point #%d"%i)

	bc.wait()

	return


if __name__ == "__main__":
	### require at least 2 drones in this test
	bc = beecluster.Session(appID = "TEST-2")

	h = bc.newTask(visitOnePoint, bc, (-20,-20,30))
	h.val 

	print("finish task 1")

	h1 = bc.newTask(box, bc, 5, TaskName="box", NonInterruptible = False, SameDrone = True)
	h2 = bc.newTask(circle,bc, 50, TaskName="circle1", NonInterruptible = True, SameDrone = False)
	h3 = bc.newTask(circle,bc, 80, TaskName="circle2", NonInterruptible = True, SameDrone = False)


	h1.wait()
	h2.wait()
	h3.wait()

	

	bc.close(SaveDTG=False)
	print("######## TEST-2  PASSED ########")












