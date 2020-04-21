####
#
# Test App 2 (Gradient descent)
# 
# Test dynamic task creation and the speculative execution feature.
# 
#
####

import sys
sys.path.append('../python') # path to beecluster
import beecluster

from time import time, sleep 
import numpy as np 

def measure_wifi_signal(bc, loc):
	bc.act("flyto", loc)

	values = []
	for i in range(5):
		v1 = bc.act("wifi:sense").val
		values.append(v1)

	value = np.mean(values)

	return value

if __name__ == "__main__":
	beecluster.ResetRemoteServer()

	t0 = time()

	bc = beecluster.Session(appID = "testapp2")
	
	loc = (60, 0, 5)
	old_dx, old_dy = -1.0, 0.0 
	ratio = 0.75 # keep 0.75
	times = []
	t_start = time()
	for i in range(12):

		loc1 = (loc[0]-3, loc[1] - 3, loc[2])
		loc2 = (loc[0]-3, loc[1] + 3, loc[2])
		loc3 = (loc[0]+ 3, loc[1] - 3, loc[2])
		loc4 = (loc[0]+ 3, loc[1] + 3, loc[2])

		h1 = bc.newTask(measure_wifi_signal, bc, loc1, NonInterruptible = True, SameDrone = True)
		h2 = bc.newTask(measure_wifi_signal, bc, loc2, NonInterruptible = True, SameDrone = True)
		h3 = bc.newTask(measure_wifi_signal, bc, loc3, NonInterruptible = True, SameDrone = True)
		h4 = bc.newTask(measure_wifi_signal, bc, loc4, NonInterruptible = True, SameDrone = True)

		v1 = h1.val
		v2 = h2.val
		v3 = h3.val
		v4 = h4.val

		times.append(time() - t_start)

		# compute the gradient
		dx = v3 + v4 - v1 - v2 + 0.000000001 
		dy = v2 + v4 - v1 - v3 + 0.000000001

		l = np.sqrt(dx*dx + dy*dy)

		dx /= l 
		dy /= l 

		# gradient momentum
		dx = dx * (1-ratio) + old_dx * ratio
		dy = dy * (1-ratio) + old_dy * ratio

		l = np.sqrt(dx*dx + dy*dy)

		dx /= l 
		dy /= l 

		loc = (loc[0] + dx * 6, loc[1] + dy * 6, 5)
		
		if i > 0:
			print(i,dx,dy, loc, (v1+v2+v3+v4)/4.0, times[-1] - times[-2])
		else:
			print(i,dx,dy, loc, (v1+v2+v3+v4)/4.0)



	bc.close(SaveDTG=False)
	print("duration", bc.getDuration())

	print("first_5_iterations:", (times[6] - times[1])/5)
	print("last_5_iterations:", (times[11] - times[6])/5)


