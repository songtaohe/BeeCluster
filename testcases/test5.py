####
#
# Test Case 5
# 
# Test failure handling 
# - timeout 
# - drone change-of-state
#
# It's better to use a short battery time (e.g., 100 seconds) in 
# the simulator, otherwise this task need to wait for a long time 
# until running out of the battery.
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

# test error handling and override flags api 

def infinite_loop(bc):
	bc.setFlags(True, True)

	bc.act("flyto", (50,50,10))
	
	while True:
		bc.act("wifi:sense").val
		sleep(1.0)

if __name__ == "__main__":
	with beecluster.Session(appID = "TEST-5") as bc:

		def error_handler(e):
			print(e)


		# test timeout 
		bc.newTask(infinite_loop, bc, TaskName="loop", error_handler = error_handler, timeout = 5.0).val 
		

		# test binding violation 
		bc.newTask(infinite_loop, bc, TaskName="loop", error_handler = error_handler, timeout = None).val 
		

		


	print("######## TEST-5  PASSED ########")












