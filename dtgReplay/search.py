#
# This file implements the hyper-parameter search with hyperopt library. 
# 
#
from subprocess import Popen 
import json 
from time import time, sleep
import numpy as np 
import math 
import sys 
from statistics import mean
import argparse

parser = argparse.ArgumentParser()

parser.add_argument('-cfg', action='store', dest='config_file', type=str,
                    help='config_file', required =True)

parser.add_argument('-algo', action='store', dest='algo', type=str,
                    help='searching algorithm', required =False, default="RS")

args = parser.parse_args()

delta = 1e-10

class Parameters():
	def __init__(self, config_file):
		self.config = json.load(open(config_file))
		self.dim = len(self.config["parameters"])

	def init_p(self):
		p = [mean(self.config["parameters"][i]["range"]) for i in range(self.dim)]
		return tuple(p) 

	def next_p(self, p, s = 0.1):
		new_p = []
		for i in range(len(p)):
			l, u = self.config["parameters"][i]["range"][0], self.config["parameters"][i]["range"][1]
			v_now = (p[i] - l)/(u-l+delta)
			v_new = np.random.normal(v_now, s)
			v_new = min(1.0, max(0.0, v_new))
			new_p.append(v_new * (u-l)+l)

		return tuple(new_p)

	def sample(self, p):
		Popen("rm ../duration.txt", shell=True).wait()

		template = json.load(open("../os/configs/framework/framework_config_replay_template.json"))

		for ind in range(self.dim):
			name = self.config["parameters"][ind]["name"]
			template["SchedulerWeights"][name] = p[ind]

		json.dump(template, open("../os/configs/framework/framework_config_replay.json","w"), indent=2)

		cmd = "cd ..; timeout 20 go run os/*.go %s %s" % (self.config["dtg"], self.config["appid"])
		Popen(cmd, shell=True).wait() 

		try:
			with open("../duration.txt","r") as fin:
				v = float(fin.readline())
		except:
			v = 100000.0 # timeout

		return v

class RandomSearchWithSimulatedAnnealing():
	def __init__(self, config_file):
		self.config = json.load(open(config_file))
		self.P = Parameters(config_file)
		
		self.samples = []

	def search(self):
		N = self.config["iteration"]
		K = 1
		P = self.P
		best_p = None
		best_v = None 

		current_p = P.init_p()
		current_v = P.sample(current_p)

		t0 = time()

		for i in range(N):
			t = K/(i+1)

			next_p = P.next_p(current_p, s = t+0.1)
			next_v = P.sample(next_p)

			self.samples.append([next_p, next_v])

			if next_v < current_v:
				current_v = next_v
				current_p = next_p
			else:
				prob = math.exp(-(next_v - current_v)/(current_v+delta)/t)

				if prob > np.random.uniform(0,1):
					current_v = next_v
					current_p = next_p

			if best_p is None or best_v > current_v:
				best_p = current_p
				best_v = current_v


			print("[Search] ts", time()-t0,"it",i," current: P = ", ["%.2f"%x for x in current_p], "time =", current_v, " best: p =", ["%.2f"%x for x in best_p], "time = ", best_v )

		json.dump(self.samples, open("backup.json","w"), indent=2)



def SearchWithHyperoptObjective(p):
	config = json.load(open(".searchcfg.json"))
	dim = len(config["parameters"])

	Popen("rm ../duration.txt", shell=True).wait()

	template = json.load(open("../os/configs/framework/framework_config_replay_template.json"))

	for ind in range(dim):
		name = config["parameters"][ind]["name"]
		template["SchedulerWeights"][name] = p[ind]

	json.dump(template, open("../os/configs/framework/framework_config_replay.json","w"), indent=2)

	cmd = "cd ..; timeout 20 go run os/*.go %s %s" % (config["dtg"], config["appid"])
	Popen(cmd, shell=True).wait() 

	try:
		with open("../duration.txt","r") as fin:
			v = float(fin.readline())
	except:
		v = 1000.0 # timeout

	print(p)

	return v 

class SearchWithHyperopt():
	def __init__(self, config_file):
		self.config = json.load(open(config_file))
		self.P = Parameters(config_file)
		self.config_file = config_file

	def search(self):
		from hyperopt import hp 
		from hyperopt import fmin, tpe, space_eval

		Popen("cp %s %s" % (self.config_file, ".searchcfg.json"), shell=True).wait()

		space = []

		for i in range(len(self.config["parameters"])):
			l, u = self.config["parameters"][i]["range"][0], self.config["parameters"][i]["range"][1]
			
			space.append(hp.uniform(self.config["parameters"][i]["name"], l, u))

		best = fmin(SearchWithHyperoptObjective, space, algo=tpe.suggest, max_evals=self.config["iteration"])

		print(best)
		print(space_eval(space, best))


if __name__ == "__main__":
	if args.algo == "randomsearch":
		S = RandomSearchWithSimulatedAnnealing(args.config_file)
	if args.algo == "hyperopt":
		S = SearchWithHyperopt(args.config_file)

	S.search()

	
