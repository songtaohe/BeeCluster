####
#
# Test Case 0
# 
# Test the setup of basic BeeCluster environment 
#
####

import sys
sys.path.append('../python') # path to beecluster
#from beecluster import bc
import beecluster
from time import time, sleep 
import threading
import atexit

# Session Creation Style 1
bc = beecluster.Session(appID = "TEST-0")
# do nothing 
bc.close()

# Session Creation Style 2
with beecluster.Session(appID = "TEST-0") as bc:
	# do nothing
	pass 



# Checking correctness

def check():
	#print(threading.active_count())
	c = 0
	for t in threading.enumerate():
		print(t.getName(), t.isDaemon(), t.is_alive())
		c += 1

	assert c <= 1, "######## TEST-0  FAILED ######## There are %d unreleased contexts (e.g., the daemon thread)." % (c-1) # the main thread

	print("######## TEST-0  PASSED ########")


atexit.register(check)