# This file implements the BeeCluster python runtime library.
# BeeCluster Python Client

import threading 
from time import time,sleep
import socket 
import requests
from inspect import getframeinfo, stack
import json
import atexit



class TaskCancelled(Exception):
	def __str__(self):
		return "TaskCancelled Exception"
	pass 

class BindingViolation(Exception):
	def __str__(self):
		return "BindingViolation Exception"
	pass 

class TimeOut(Exception):
	def __str__(self):
		return "TimeOut Exception"
	pass 

class BeeClusterContext(object):
	def __init__(self, bc):
		self.bc = bc 

		self.drone_handler = None 
		self.affinity = 0 


		self.group_id = None 
		self.thread_id = None 
		self.thread_num = None

		self.barrier = None 

		self.SIMD = False 


		self.codeblock = []
		self.codeblock_name = "rootblock"

		# two type's of action here
		# newtask
		# wait
		self.last_action_type = "wait"
		self.waitstack = []


		self.scope = "root"


		self.codeblock_id = 0 
		self.codeblock_metainfo = {"firstloc":None, "actions":[], "duration":0.0}


		# simd 
		self.sync_counter = 0 


		# cancelled? or have error? or timeout?
		self.isCancelled = False 
		self.hasBindingViolation = False
		self.isTimeOut = False



		# drone status (abstract drone)
		self.loc = [0,0,0]
		self.samedrone_handler = None 


		# Action Queue
		self.action_queue = []
		self.action_results = []
		self.action_new_events = [threading.Event()]
		self.action_done_events = [] # threading.Event()
		self.action_queue_execution_ptr = 0
		#self.action_queue_lock = threading.Lock()

		self.daemon_stop = False
		self.daemon_thread = threading.Thread(target=self.daemon)
		self.daemon_thread.start() # TODO need to clean this up 
		#print("create", self.daemon_thread.getName(), self.daemon_thread.isAlive())
		
		# todo need to add a deamon ?
		# the deamon executing the action one by one 
	def addAction(self, action_name, *argv):
		if self.isCancelled:
			raise TaskCancelled
		if self.hasBindingViolation:
			raise BindingViolation
		if self.isTimeOut:
			raise TimeOut

		self.action_queue.append((action_name, *argv))
		self.action_results.append("None")
		self.action_new_events.append(threading.Event()) # ptr + 1 for next action
		self.action_done_events.append(threading.Event())
	
		self.action_new_events[-2].set()

		handle = Action(self, len(self.action_queue)-1)
		return handle

	def wait(self):
		if self.isCancelled:
			raise TaskCancelled

		if self.hasBindingViolation:
			raise BindingViolation
		
		if self.isTimeOut:
			raise TimeOut

			
		n = len(self.action_queue)-1
		self.action_done_events[n].wait()

		if self.isCancelled:
			raise TaskCancelled
		
		if self.hasBindingViolation:
			raise BindingViolation
		
		if self.isTimeOut:
			raise TimeOut



	def executeAction(self, action_name, *argv):
		context = self 
		if context.isCancelled :
			raise TaskCancelled

		if self.hasBindingViolation:
			raise BindingViolation
		
		if self.isTimeOut:
			raise TimeOut

		ret = None 
		retry = False 
		#if action_name == "maneuver/flyto":
		if action_name == "flyto":
			if context.drone_handler == None:
				drone_id = None 
				#nonInterruptibleFlag = context.codeblock[-1]["NonInterruptible"]
				nonInterruptibleFlag = context.codeblock[-1]["NonInterruptible"] and (context.codeblock[-1]["SameDrone"]==False)

				if context.codeblock[-1]["SameDrone"] == True and context.samedrone_handler is not None:
					drone_id = context.samedrone_handler.drone_id

				context.drone_handler = self.bc._acquire_drone(argv[0], current_context = self, droneID = drone_id, NonInterruptible = nonInterruptibleFlag)
				
				if context.samedrone_handler is None:
					context.samedrone_handler = DroneHandler(context.drone_handler.drone_id)

				if context.codeblock_metainfo["firstloc"] is None :
					context.codeblock_metainfo["firstloc"] = argv[0]

			else:
				context.drone_handler.act("flyto", argv[0])

			context.loc = [argv[0][0],argv[0][1],argv[0][2]]
			ret = {}
			ret["status"] = 0
			ret["data"] = None 
			ret = json.dumps(ret)


		#elif action_name.startswith("system/"): # system action, e.g., get time 
		elif action_name == "GetDuration":
			ret = DroneHandler(0).act(action_name, argv)
			return ret, retry  

		else:
			if context.drone_handler == None:
				drone_id = None 
				nonInterruptibleFlag = context.codeblock[-1]["NonInterruptible"] and (context.codeblock[-1]["SameDrone"]==False)

				if context.codeblock[-1]["SameDrone"] == True and context.samedrone_handler is not None:
					drone_id = context.samedrone_handler.drone_id

				context.drone_handler = self.bc._acquire_drone(context.loc, current_context = self,droneID = drone_id, NonInterruptible = nonInterruptibleFlag)
				
				if context.samedrone_handler is None:
					context.samedrone_handler = DroneHandler(context.drone_handler.drone_id)

			ret = context.drone_handler.act(action_name, argv)
			#print("???", ret)
			try:
				#print("action result raw", ret.decode('utf-8'))
				actionresult = json.loads(ret)
				#print("[Debug] action result raw", actionresult)

				if actionresult["status"] >= 0 :
					#ret = actionresult["data"]
					pass
				else:
					#print("[Debug] action result raw", actionresult, "retry in 0.1 seconds (unverified feature)")
					retry = True 
			except:
				pass

		#if context.affinity == 0  and context.drone_handler != None:
		if context.codeblock[-1]["SameDrone"] == True and context.codeblock[-1]["NonInterruptible"] == True:
			pass 
		else:
			self.bc._release_drone(context.drone_handler)
			context.drone_handler = None 

		#print(ret)
		if ret is not None :
			try:
				ret = ret.decode('utf-8')
			except:
				pass 

		context.codeblock_metainfo["actions"].append([action_name, *argv, ret])

		msg = {}

		msg["cmd"] = "logAction"
		msg["scopeName"] = context.scope 
		msg["nodename"] = context.codeblock_name 
		msg["actionname"] = action_name
		msg["actionret"] = ret 

		_rpc(json.dumps(msg))


		return json.loads(ret)["data"], retry  


	def daemon(self):
		t0 = time()
		ptr = 0
		try:
			while True:
				ptr = self.action_queue_execution_ptr 
				self.action_new_events[ptr].wait()
				self.action_new_events[ptr].clear()
				if self.daemon_stop == True:
					break 

				#print(ptr,"clear")
				action_name = self.action_queue[ptr][0]
				if len(self.action_queue[ptr]) > 1:
					action_parameter = self.action_queue[ptr][1]
				else:
					action_parameter = None 
				
				#print("execute action ", ptr, self.codeblock_name, self.isCancelled)

				while True:
					self.action_results[ptr], retry = self.executeAction(action_name, action_parameter)
					
					if self.config["timeout"] is not None and time()-t0 > self.config["timeout"]:
						raise TimeOut

					if retry :
						if self.codeblock[-1]["NonInterruptible"] and self.codeblock[-1]["SameDrone"]:
							raise BindingViolation

						sleep(0.1)
					else:
						break

					

				self.action_done_events[ptr].set()
				self.action_queue_execution_ptr += 1

		except TaskCancelled:
			self.isCancelled = True
			self.daemon_stop = True
			while ptr < len(self.action_done_events):
				self.action_done_events[ptr].set()
				ptr += 1

		except BindingViolation:
			self.hasBindingViolation = True 
			self.daemon_stop = True 
			while ptr < len(self.action_done_events):
				self.action_done_events[ptr].set()
				ptr += 1

		except TimeOut:
			self.isTimeOut = True 
			self.daemon_stop = True 
			while ptr < len(self.action_done_events):
				self.action_done_events[ptr].set()
				ptr += 1


		#print("daemon stopped")
		self.daemon_stop = True

		return 

	def cleanup(self):	
		# TODO clean up 
		# stop the daemon

		# this context has already been cleaned up 
		if self.daemon_stop == True:
			return 

		self.daemon_stop = True 
		self.action_new_events[-1].set() 

		#print("waiting for daemon to stop")
		self.daemon_thread.join()

		#print("cleanup", self.daemon_thread.getName(), self.daemon_thread.isAlive())
		
		for e in self.action_new_events:
			e.clear()

		for e in self.action_done_events:
			e.clear()

		#print("clean up context")

		pass


def _rpc(msg, port = 8008):
	msg = str.encode(msg)
	r = requests.post("http://127.0.0.1:%d" % port, data = msg)

	return r.text 

def _post(msg, url):
	msg = str.encode(msg)
	r = requests.post(url, data = msg)
	return r.text 



class DroneHandler(object):
	def __init__(self, drone_id):
		self.drone_id = drone_id 

		pass 

	def act(self, action_name, *argv):
		ret = None 

		# talk to remote drone 
		if action_name == "flyto":
			#print(argv)

			ret = _rpc("{\n \"cmd\": \"flyto\",\n \"x\":%f,\n \"y\":%f,\n \"z\":%f, \"id\": %d \n }" % (argv[0][0], argv[0][1], argv[0][2], self.drone_id), port=8009)
			
		else:
			ret = _rpc("{\n \"cmd\": \"%s\", \"id\": %d }" % (action_name, self.drone_id), port=8009)

		return ret 


# Action Handler 
class Action(object):
	def __init__(self, context, action_ptr):
		self.context = context
		self.action_ptr = action_ptr

	def __getattribute__(self, name):
		if name == "val":
			if self.context.isCancelled:
				raise TaskCancelled
			if self.context.hasBindingViolation:
				raise BindingViolation
			if self.context.isTimeOut:
				raise TimeOut

			self.context.action_done_events[self.action_ptr].wait()

			if self.context.isCancelled:
				raise TaskCancelled
			if self.context.hasBindingViolation:
				raise BindingViolation
			if self.context.isTimeOut:
				raise TimeOut
			
			return self.context.action_results[self.action_ptr]
		else:
			return object.__getattribute__(self,name)


# Task 
class Task(object):
	def __init__(self, bc, config, func, callback_handler, error_handler, timeout,  *argv):

		self.config = config
		self.taskname = "%s_%d" % (self.config["func_id"],self.config["task_id"])
		self.bc = bc
		self.func = func 
		self.argv = argv

		self.callback_handler = callback_handler
		self.error_handler = error_handler
		self.config["timeout"] = timeout 

		bc.BCLock() 
		bc.active_task_counter += 1
		self.task_uid = bc.active_task_counter
		bc.BCUnLock()

		self.thread = threading.Thread(target=self.run)
		self.thread.start()

		self.state = 0 

		self.subroutine_context_thandle = None 



	def run(self):
		self.bc.newThread()
		thandle = self.bc.thisThread()


		self.childContextHandle = thandle

		self.bc.contexts[thandle] = BeeClusterContext(self.bc)
		self.bc.contexts[thandle].config = self.config 

		self.bc.contexts[thandle].scope = "%s_%d" % (self.config["func_id"],self.config["task_id"])

		self.bc.contexts[thandle].group_id = self.config["group_id"]
		self.bc.contexts[thandle].thread_id = self.config["thread_id"]
		self.bc.contexts[thandle].thread_num = self.config["thread_num"]
		self.bc.contexts[thandle].barrier = self.config["barrier"]
		
		self.bc.name2context[self.taskname] = self.childContextHandle

		self.subroutine_context_thandle = thandle 

		self.ret = None # add error code here ?

		try:
			self.bc.enterCodeBlock(blockname = self.config["func_id"], SameDrone = self.config["SameDrone"], NonInterruptible = self.config["NonInterruptible"]) 
		
			#print(self.argv)
			if len(self.argv) == 0:
			#if (self.argv[0]) is None:
				self.ret = self.func()
			else:
				self.ret = self.func(*self.argv)

			self.bc.exitCodeBlock()
			
		except TaskCancelled:
			self.ret = None 

		except Exception as e:
			if self.error_handler is not None:
				self.error_handler(e)

		if self.bc.contexts[thandle].drone_handler is not None :
			self.bc._release_drone(self.bc.contexts[thandle].drone_handler)

		if self.callback_handler is not None:
			self.callback_handler(self.ret)

		self.bc.BCLock() 
		self.bc.active_task_counter -= 1
		self.bc.BCUnLock()

		self.bc.contexts[thandle].cleanup()



	def getTaskName(self):
		return self.taskname

	def wait(self):
		self.val 
		return

	def __getattribute__(self, name):
		if name == "val":
			if self.state == 0:
				self.thread.join()
				self.state = 1 
				

				# only do this for the first time the value is retrieved. 
				context = self.bc.contexts[self.bc.thisThread()]
				names = ["%s_%d" % (self.config["func_id"],self.config["task_id"])]

				

				#print("???", context.waitstack)

				# this name could be the last wait in its routine 
				if len(self.bc.contexts[self.childContextHandle].waitstack) > 0:
					names = list(self.bc.contexts[self.childContextHandle].waitstack)

				#print(names)


				if context.last_action_type == "wait":
					for item in names :
						#print(names, item)
						if item not in context.waitstack:
							context.waitstack.append(item)

				else:
					#context.waitstack = []
					context.waitstack = names

				context.last_action_type = "wait"

				#print("???2", context.waitstack)

			return self.ret

		return object.__getattribute__(self,name)

# use session to create beecluster object 
def Session(appID = "ABC123", BeeClusterControlEndpoint = "http://127.0.0.1:8008", BeeClusterDroneEndpoint = "http://127.0.0.1:8009"):
	return BeeCluster(appID = appID, BeeClusterControlEndpoint = BeeClusterControlEndpoint, BeeClusterDroneEndpoint = BeeClusterDroneEndpoint)

def ResetRemoteServer(seed = None):	
	if seed is None:
		requests.get(url = "http://localhost:8007/reset")
	else:
		requests.get(url = "http://localhost:8007/reset?seed=%d" % seed)

class BeeCluster(object):
	def __init__(self, appID = "ABC123", BeeClusterControlEndpoint = "http://127.0.0.1:8008", BeeClusterDroneEndpoint = "http://127.0.0.1:8009"):
		self.contexts = {}
		self._reset()
		self.connect(appID = appID, BeeClusterControlEndpoint = BeeClusterControlEndpoint, BeeClusterDroneEndpoint = BeeClusterDroneEndpoint)


	def _reset(self):
		#self.cleanup()
		self.isConnected = False
		self.contexts = {}
		self.lifelongid_dict = {}
		self.lifelongid = 0
		
		self.global_lock = threading.Lock()
		self.internal_global_lock = threading.Lock()


		self.newThread()

		if self.thisThread() in self.contexts:
			self.contexts[self.thisThread()].cleanup()

		self.contexts[self.thisThread()] = BeeClusterContext(self)

		self.name2context = {}

		# root context 
		self.contexts[self.thisThread()].config = {"group_id": None,
			"thread_id": None,
			"thread_num": None, 
			"barrier": None,
			"func_id": "root",
			"task_id": 0}

		self.group_num = 0 

		self.func_list = {}
		self.func_id = 0 
		self.task_id = 0 

		self.DCG = []
		self.DCGSyncBlock = {}
		self.DCGSyncBlock_id = 0 

		self.active_task_counter = 0 

	def __enter__(self):
		return self 

	def __exit__(self, type, value, traceback):
		self.close()


	def connect(self, appID = "ABC123", BeeClusterControlEndpoint = "http://127.0.0.1:8008", BeeClusterDroneEndpoint = "http://127.0.0.1:8009"):
		self.BeeClusterControlEndpoint = BeeClusterControlEndpoint
		self.BeeClusterDroneEndpoint = BeeClusterDroneEndpoint

		self.appID = appID 
		self.sessionID = 0 # query from control endpoint 

		msg = {}
		msg["cmd"] = "newSession"
		msg["appid"] = self.appID

		self.sessionID = int(_post(json.dumps(msg), url=self.BeeClusterControlEndpoint))
		self.isConnected = True 

	def close(self, SaveDTG=False):
		self.cleanup()

		msg = {}
		if SaveDTG == False:
			msg["cmd"] = "stopSession"
		else:
			msg["cmd"] = "stopSessionAndSaveDTG"

		msg["appid"] = self.appID 
		msg["sessionid"] = self.sessionID 

		_post(json.dumps(msg), url=self.BeeClusterControlEndpoint)

	def cleanup(self):
		allContexts = self.contexts.values()
		for context in allContexts:
			try:
				context.cleanup()
			except:
				pass 
		


	def Lock(self):
		self.global_lock.acquire() 

	def UnLock(self):
		self.global_lock.release()

	def BCLock(self):
		self.internal_global_lock.acquire() 

	def BCUnLock(self):
		self.internal_global_lock.release()

	def waitUnfinishedTasks():
		while True:
			sleep(0.1)
			cc = bc.active_task_counter
			if cc == 0:

				break


	def newThread(self):
		self.BCLock()
		self.lifelongid_dict[self.thisThreadRaw()] = self.lifelongid
		self.lifelongid += 1
		self.BCUnLock()


	def thisThread(self, uid = 0):
		try:
			tid = threading.get_ident()
		except:
			tid = threading._get_ident()

		uid = self.lifelongid_dict[tid] 

		return str(tid)+"_"+str(uid) 

	def thisThreadRaw(self):
		try:
			tid = threading.get_ident()
		except:
			tid = threading._get_ident()

		
		return tid

		#return threading._get_ident()


	def addDependency(self, taskuid, dependingSet, scope = "root", metainfo = None):

		self.BCLock()

		if len(dependingSet) == 0 :
			dependingSet =[scope]

		if len(dependingSet) > 1:

			s = sorted(list(dependingSet))
			name = ""
			for item in s:
				name += item 

			if name in self.DCGSyncBlock:
				syncBlockName = self.DCGSyncBlock[name]
				dependingSet = [syncBlockName]
			else:
				syncBlockName = "syncBlock_%d" % self.DCGSyncBlock_id
				self.DCGSyncBlock_id += 1 

				self.DCGSyncBlock[name] = syncBlockName

				#print("new dependency2", [syncBlockName, list(dependingSet), scope, metainfo])

				self.DCG.append([syncBlockName, list(dependingSet), scope, metainfo])
				
				msg = {}
				msg["cmd"] = "addDependency"
				msg["scopeName"] = scope 
				msg["nodename"] = syncBlockName
				msg["dependencies"] = list(dependingSet)
				msg["metainfo"] = "\"" + json.dumps(metainfo) + "\""

				_rpc(json.dumps(msg))


				dependingSet = [syncBlockName]


		self.DCG.append([taskuid, list(dependingSet), scope, metainfo])

		msg = {}
		msg["cmd"] = "addDependency"
		msg["scopeName"] = scope 
		msg["nodename"] = taskuid 
		msg["dependencies"] = list(dependingSet)
		msg["metainfo"] = "\"" + json.dumps(metainfo) + "\""
		msg["appid"] = self.appID 
		msg["sessionid"] = self.sessionID 

		_rpc(json.dumps(msg))

		#print("new dependency", [taskuid, dependingSet, scope, metainfo])

		self.BCUnLock()


	def newTask(self, func, *argv, TaskName = None, SameDrone=False, NonInterruptible = False, callback_handler = None, error_handler = None, timeout = None):
		config = {"group_id": None,
			"thread_id": None,
			"thread_num": None, 
			"barrier": None,
			"TaskName": TaskName,
			"SameDrone": SameDrone,
			"NonInterruptible":NonInterruptible,
			"callback_handler" : callback_handler,
			"error_handler" : error_handler}

		if TaskName is not None:
			config["func_id"] = TaskName
		else:	
			config["func_id"] = func.__name__
			
		self.BCLock() 
		task_id = self.task_id
		self.task_id += 1

		self.BCUnLock()

		config["task_id"] = task_id

		context = self.contexts[self.thisThread()]
		context.last_action_type = "newtask"


		self.addDependency("%s_%d" % (config["func_id"],config["task_id"]), list(context.waitstack), context.scope)

		task = Task(self, config, func, callback_handler, error_handler, timeout,  *argv)

		return task

	# This function is used in the old version. It needs to be updated.  
	def newTaskSIMD(self, num, func, *argv):

		barrier = threading.Barrier(num)
		tasks = []

		self.BCLock() 
		
		task_id = self.task_id
		self.task_id += num

		self.BCUnLock()

		context = self.contexts[self.thisThread()]
		context.last_action_type = "newtask"

		for i in range(num):
			config = {"group_id": self.group_num,
			"thread_id": i,
			"thread_num": num, 
			"barrier": barrier,
			"func_id": func.__name__,
			"task_id": task_id + i}

			

			self.addDependency("%s_%d" % (config["func_id"],config["task_id"]), list(context.waitstack), context.scope)


			tasks.append(Task(self, config, func, None,  *argv))


		self.group_num += 1


		return tasks 


	def get_thread_id(self):
		return self.contexts[self.thisThread()].thread_id

	def get_thread_num(self):
		return self.contexts[self.thisThread()].thread_num


	 
	def sync(self):

		context = self.contexts[self.thisThread()]
		context.barrier.wait()

		sync_name = "syncBlockSIMD_%d_%d"%(context.group_id, context.sync_counter)
		context.sync_counter += 1

		self.addDependency(sync_name, list(context.waitstack), context.scope)
		context.waitstack = [sync_name]

		

		if self.get_thread_id() == 0 :
			self.contexts[self.thisThread()].barrier.reset()


	def cancelTask(self, names):
		if type(names) == list:
			pass 
		else:
			names = [names]

		context = self.contexts[self.thisThread()]

		for name in names:
			self.addDependency("cancel_%s" % name, list(context.waitstack), context.scope)


		for name in names:
			handle = self.name2context[name]
			self.contexts[handle].isCancelled = True
			msg = {}
			msg["cmd"] = "cancelRequest"
			msg["scopename"] = name 
			msg["appid"] = self.appID 
			msg["sessionid"] = self.sessionID 
			
			ack = _post(json.dumps(msg), url=self.BeeClusterControlEndpoint)

			#ack = _rpc("{\n \"cmd\": \"cancelRequest\",\n \"scopename\":\"%s\" \n }" % (name))

		pass


	# act(action_name and argv)
	def	act(self, action_name, *argv):
		context = self.contexts[self.thisThread()]
		if context.isCancelled :
			if context.drone_handler != None :
				self._release_drone(context.drone_handler)
				context.drone_handler = None 

			raise TaskCancelled


		action_handle = context.addAction(action_name, *argv)
		
		return action_handle

	def wait(self):
		context = self.contexts[self.thisThread()]
		context.wait()


	def setFlags(self, SameDrone=None, NonInterruptible = None):
		context = self.contexts[self.thisThread()]
		if NonInterruptible is not None:
			context.codeblock[-1]["NonInterruptible"] = NonInterruptible
		if SameDrone is not None:
			context.codeblock[-1]["SameDrone"] = SameDrone

	def setAffinity(self, flag):

		context = self.contexts[self.thisThread()]

		if flag == True:
			context.affinity += 1
		else:
			context.affinity -= 1


		if context.affinity == 0 :
			# release the drone 
			self._release_drone(context.drone_handler)
			context.drone_handler = None 

	def enterCodeBlock(self, blockname = None, SameDrone = True, NonInterruptible = False, Offload = False, SIMD = False):

		context = self.contexts[self.thisThread()]
		if context.isCancelled :
			if context.drone_handler != None :
				self._release_drone(context.drone_handler)
				context.drone_handler = None 

			raise TaskCancelled

		if SameDrone == True:
			context.affinity += 1
		flags = {"SameDrone":SameDrone, "NonInterruptible":NonInterruptible, "Offload":Offload, "SIMD":SIMD}
		context.codeblock.append(flags)

		context.codeblock_metainfo = {"firstloc":None, "actions":[], "duration":0.0}


		# add a dependency here
		# later on, when there is a request or action, update the dependency entry


		if blockname is None:
			caller = getframeinfo(stack()[1][0])
			blockname = "%s:%d" % (caller.filename, caller.lineno)

		config = context.config


		metainfo_dict = dict(context.codeblock_metainfo)
		metainfo_dict.update(dict(flags))

		self.addDependency("codeBlock_%s_%d_%s_%d" % (config["func_id"],config["task_id"], blockname, context.codeblock_id), list(context.waitstack), context.scope, metainfo = metainfo_dict)

		context.codeblock_name = "codeBlock_%s_%d_%s_%d" % (config["func_id"],config["task_id"], blockname, context.codeblock_id)


		context.waitstack = []
		context.waitstack = ["codeBlock_%s_%d_%s_%d" % (config["func_id"],config["task_id"], blockname, context.codeblock_id)]

		context.last_action_type == "wait"


		context.codeblock_id += 1



	def exitCodeBlock(self):

		context = self.contexts[self.thisThread()]
		if context.isCancelled :
			if context.drone_handler is not None :
				self._release_drone(context.drone_handler)
				context.drone_handler = None 

			raise TaskCancelled


		flags = context.codeblock.pop()


		if flags["SameDrone"] == True:
			context.affinity -= 1

			if context.affinity == 0 :
				# release the drone 
				if context.drone_handler is not None:
					self._release_drone(context.drone_handler)
					context.drone_handler = None 


		# move the following code to the entercodeblock part 

		# caller = getframeinfo(stack()[1][0])

		# blockname = "%s:%d" % (caller.filename, caller.lineno)

		# config = context.config


		# metainfo_dict = dict(context.codeblock_metainfo)
		# metainfo_dict.update(dict(flags))

		# self.addDependency("codeBlock_%s_%d_%s_%d" % (config["func_id"],config["task_id"], blockname, context.codeblock_id), list(context.waitstack), context.scope, metainfo = metainfo_dict)


		# context.waitstack = []
		# context.waitstack = ["codeBlock_%s_%d_%s_%d" % (config["func_id"],config["task_id"], blockname, context.codeblock_id)]

		# context.last_action_type == "wait"


		# context.codeblock_id += 1


	def _acquire_drone(self, loc, sensor = None, current_context = None, NonInterruptible = False, droneID = None ):
		if current_context is None:
			context = self.contexts[self.thisThread()]
		else:
			context = current_context

		msg = {}
		msg["cmd"] = "newRequest"
		msg["scopeName"] = context.scope 
		msg["nodename"] = context.codeblock_name
		msg["x"] = loc[0]
		msg["y"] = loc[1]
		msg["z"] = loc[2]
		msg["appid"] = self.appID 
		msg["sessionid"] = self.sessionID 


		# use this if we want to acquire a specific drone 
		if droneID is not None:
			msg["droneid"] = droneID
		else:
			msg["droneid"] = -1

		# use this to let the backend prepare a hand-off drone if needed 
		msg["noninterruptibleflag"] = NonInterruptible

		#print(msg)

		reqid = _rpc(json.dumps(msg))


		#reqid = _rpc("{\n \"cmd\": \"newRequest\",\n \"x\":%f,\n \"y\":%f,\n \"z\":%f,\n \"ScopeName\": \"%s\" \n }" % (loc[0],loc[1], loc[2], context.scope))

		#print(reqid)

		reqid = int(reqid)

		drone_handler = _rpc("{\n \"cmd\": \"getDrone\", \"id\": %d }" % reqid)

		# todo add support for drone cancel 

		drone_handler = int(drone_handler)

		if drone_handler == -1 :
			print("cancel task at", loc, reqid)
			raise TaskCancelled

		
		#print("acquire drone", loc, reqid, drone_handler)
		return DroneHandler(drone_handler)


	def _release_drone(self, drone_handler):
		ret = _rpc("{ \"cmd\": \"releaseDrone\", \"id\": %d }" % drone_handler.drone_id)
		#print("release drone", drone_handler.drone_id, "should be ack -->",ret)
		
		pass 


	def reset(self, seed = None):
		if seed is None:
			requests.get(url = "http://localhost:8007/reset")
		else:
			requests.get(url = "http://localhost:8007/reset?seed=%d" % seed)

		self._reset()

	def getDuration(self):
		ret = DroneHandler(0).act("GetDuration", None)
		return float(ret)


	def control(self, endpoint = "abc"):
		requests.get(url = "http://localhost:8007/" + endpoint)

		
	def DCG2GML(self, filename = "tmp.gml"):
		with open(filename,"w") as fout:
			fout.write("graph\n[\n")

			def addNode(nodeid, label= "none", color = "#FF0000", order=0):

				sg = "graphics [ type \"roundrectangle\" fill \"%s\" outline \"#000000\"]" % color 

				fout.write("  node\n  [\n   id %s\n   label \"%s\"\n   %s\n   order %d\n  ]\n" % (nodeid, label, sg, order))


			def addEdge(node1id, node2id):
				fout.write("  edge\n  [\n   source %s\n   target \"%s\"\n  ]\n" % (node1id, node2id))


			addNode("root","root")

			order =  0

			for item in self.DCG:
				color = "#FF4444"

				if item[0].startswith("syncBlock"):
					color = "#FF4444"

				elif item[0].startswith("codeBlock"):
					color = "#FF8800"

				else:
					color = "#4444FF"


				addNode(item[0], item[0], color, order)
				order += 1

			for item in self.DCG:

				for dependent in item[1]:
					addEdge( dependent, item[0])


			fout.write("]\n")


# bc = BeeCluster() 


# def clean():
# 	bc.cleanup()

# atexit.register(clean)





