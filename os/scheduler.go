package main 

/*
 * This file implements the scheduler of BeeCluster.
 * 
 */

import (
	"fmt"
	"sort"
	"image"
	"image/color"
	"image/png"
	"github.com/llgcode/draw2d/draw2dimg"
	"os"
	"math/rand"
	"math"
	//"time"
)


const (
	EXIST_REQUEST = 0
	PREDICTION_REQUEST = 1
)

// define some common datatype 

// type ITask interface{
// 	GetLocation() [3]float64
// 	GetType() int 
// }

// type IDrone interface{

// }


type ScheduleProblemDroneInfo struct {
	ClusterID	int 

	//Time2PredictedTasks []float64 
	//Time2Tasks 			[]float64
}

type ScheduleProblemTaskInfo struct {
	ClusterID	int 
	Utility     float64

	//Time2PredictedTasks []float64 
	//Time2Tasks 			[]float64
}


type SchedulerHeuristicWeights struct {
	Weights map[string]float64 `json"weights"`
}


// the assignment index would be 
// from 0 to len(Tasks)-1 are for real tasks 
// from len(Tasks) to len(Tasks) + len(PredictedTasks) are predictions 
type ScheduleProblem struct {
	PredictedTasks 	[]*VirtualNode
	Tasks           []*Request 
	ShadowTasks 	[]Request 
	Drones          []*Drone 

	TaskInfo 		[]*ScheduleProblemTaskInfo
	DroneInfo 		[]*ScheduleProblemDroneInfo

	TaskPerCluster  		[][]int 
	PredictedTaskPerCluster [][]int 
	DroneClosestPrediction 	[]int 

	PerTaskPrediction []*Prediction

	PredictedTaskCluster [][3]float64 
	PredictedTaskClusterAssignment []int 


	fra 			*Framework 
	assignment_size int 
	// other information ... 

	unassignedTask 	[]int
	unassignedDrone []int  

	weights 	map[string]float64

	drone2taskAffinity [][]int 
}

// func MakeScheduleProblem() *ScheduleProblem {
// 	sp := new(ScheduleProblem)


// 	return sp 
// }

func (sp *ScheduleProblem) setup() {
	// setup 
	sp.DroneClosestPrediction = make([]int, len(sp.Drones))

	if len(sp.PredictedTaskCluster) > 1 {
		sp.TaskPerCluster = make([][]int, len(sp.PredictedTaskCluster))
		sp.PredictedTaskPerCluster = make([][]int, len(sp.PredictedTaskCluster))

		for tid, info := range sp.TaskInfo {
			sp.TaskPerCluster[info.ClusterID] = append(sp.TaskPerCluster[info.ClusterID], tid)
		}



		for tid, clusterID := range sp.PredictedTaskClusterAssignment {
			sp.PredictedTaskPerCluster[clusterID] = append(sp.PredictedTaskPerCluster[clusterID], tid)
		}



		for i:=0; i<len(sp.Drones); i++ {

			predictedTask := make(map[int][3]float64)
			currentLoc := sp.Drones[i].Loc

			for _, tid := range sp.PredictedTaskPerCluster[sp.DroneInfo[i].ClusterID] {
				predictedTask[tid] = sp.PredictedTasks[tid].Location
			}

			nextPredictedTask := findClosestKey(currentLoc, predictedTask, distance)
				
			sp.DroneClosestPrediction[i] = nextPredictedTask
		}

	}


	for i:=0; i<len(sp.Drones); i++ {
		task_ids := []int{}

		for j:=0; j<len(sp.Tasks); j += 1{
			if sp.DroneTaskAffinity(i,j) {
				task_ids = append(task_ids, j)
			}
		}

		for j:=len(sp.Tasks); j<len(sp.Tasks) + len(sp.PredictedTasks); j ++ {
			if sp.DroneTaskAffinity(i,j) {
				task_ids = append(task_ids, j)
			}	
		}

		sp.drone2taskAffinity = append(sp.drone2taskAffinity, task_ids)
	}

	// for j:=0; j<len(sp.Tasks); j += 1{
	// 	fmt.Println(j, sp.Tasks[j].acquiredDroneID)
	// }
}

func (sp *ScheduleProblem) Solve() []int {

	sp.setup()

	// solve

	assignment := sp.FindFirstAssignment()
	//assignment = sp.FilterAssignment(assignment) 

	// this is not baseline!
	//return assignment

	if len(sp.Tasks) + len(sp.PredictedTasks) < len(sp.Drones) {
		if vis_log {
			sp.DebugVisualization(assignment, "os/web/assignment.png")
		}
		return assignment
	}


	//fmt.Println(assignment)
	//fmt.Println("greedy searching")

	var best_cost float64 
	best_assignment := make([]int, sp.assignment_size)

	copy(best_assignment, assignment)
	current_assignment := assignment

	sp.BeforeRandomAssignment(current_assignment)
	best_cost = sp.ComputeCost(current_assignment, false)


	PrintLog(LogInfo, fmt.Sprintln("Schedule problem: nTask=", len(sp.Tasks)," nDrone=", len(sp.Drones), "nPred=",len(sp.PredictedTasks)))
	PrintLog(LogInfo, fmt.Sprintln("Unassigned Tasks:   ",sp.unassignedTask))
	PrintLog(LogInfo, fmt.Sprintln("Tasks Per Cluster:  ",sp.TaskPerCluster))
	PrintLog(LogInfo, fmt.Sprintln("Drone Closest Pred: ",sp.DroneClosestPrediction))
	//fmt.Println(sp.DroneInfo[0], sp.DroneInfo[1])

	// Simulated Annealing

	var k_max float64 = 100
	for k:=0; k< 500; k ++ {
		T := k_max/float64(k+1) 
		sp.BeforeRandomAssignment(current_assignment)

		for i:=0; i<10;i++ {
			ass := sp.RandomAssignmentBasic(current_assignment)
			ass = sp.FilterAssignment(ass)
			var same bool = true
			for j:=0;j<len(ass);j++{
				if ass[j] != current_assignment[j] {
					same = false
					break
				}
			}

			if same {
				continue
			} else {
				_cost := sp.ComputeCost(ass, false)
				var p float64 = 1.0
				if _cost > best_cost {
					p = math.Exp(-(_cost - best_cost)/T)
				}

				if p > rand.Float64() {
					best_cost = _cost
					copy(current_assignment, ass)

				}
				break
			}
		}


		if k % 100 == 0 {
			PrintLog(LogDebug,fmt.Sprintln("[Solver-SA] it", k, "T=", T, "cost=", best_cost, "assignment=", current_assignment))
		}
	}

	copy(best_assignment, current_assignment)

	// Greedy Search
	for it:= 0 ; it<10; it ++ {
		if quiet_log == false {
			if it == 0 || it == 9 {
				PrintLog(LogDebug, fmt.Sprintln("[Solver-Greedy] iteration", it, best_cost, best_assignment))
			}
		}

		//fmt.Println("iteration", it, best_cost, best_assignment)

		sp.BeforeRandomAssignment(current_assignment)

		for i:=0; i< 100; i++ {
			ass := sp.RandomAssignmentBasic(current_assignment)
			ass = sp.FilterAssignment(ass)
			_cost := sp.ComputeCost(ass, false)

			if _cost < best_cost {
				best_cost = _cost
				copy(best_assignment, ass)
			}
		}

		copy(current_assignment, best_assignment)
	}

	PrintLog(LogDebug, fmt.Sprintln("[Solver] Total Cost: ", sp.ComputeCost(assignment, true)))

	if vis_log {
		sp.DebugVisualization(assignment, "os/web/assignment.png")
	}
	return assignment
}



func (sp *ScheduleProblem) ComputeCost(assignment []int, printall bool) float64 {
	// The cost consist of several parts
	// (1) workload imbalance cost
	// (2) mutual task cost 
	// (3) route efficiency (avoid zig-zag)
	// (4) fairness among different tasks (if appliable)
	// (5) time^2 cost
	// (6) gatherring cost (drone should be far away to each other)
	// (7) scattering cost (drone should be close to each other)



	var w float64 = 1.0 

	// Part 0: Task Reward (avoid drone doing no tasks)
	var TaskReward float64 = sp.TaskReward(assignment)

	// Part 1: Workload imbalance cost 
	var WorkloadImbalanceCost float64 = 0.0 
	
	w = 1.0 // default 
	if nw, ok := sp.weights["WorkloadImbalanceCost"]; ok {
		//fmt.Println("markmark1",nw)
		w = nw 
	}

	WorkloadImbalanceCost = sp.WorkloadImbalanceCost(assignment) * w * 0.0 // TODO * weights 


	// Part 2: Mutual Task Cost (Correlation)
	var MutualTaskCost float64 = 0.0 
	w = 1.0 // default 
	if nw, ok := sp.weights["MutualUtilityCost"]; ok {
		//fmt.Println("markmark2",nw)
		w = nw 
	}

	MutualTaskCost = sp.MutualTaskCost(assignment) * w // TODO * weights 


	// Part 3(a): Route Efficiency cost (avoid zigzag)
	w = 4.0 // default 
	if nw, ok := sp.weights["EfficientRouteCost(a)"]; ok {
		//fmt.Println("markmark3",nw)
		w = nw 
	}
	var RouteEfficiencyCost float64 = sp.EfficientRouteCost(assignment) * w // was 0.0
	//RouteEfficiencyCost = - RouteEfficiencyCost * RouteEfficiencyCost * 1.0
	

	// Part 3(b): Route Efficiency cost (avoid zigzag)
	w = 20.0 // default 
	if nw, ok := sp.weights["EfficientRouteCost(b)"]; ok {
		//fmt.Println("markmark4",nw)
		w = nw 
	}

	RouteEfficiencyCost += sp.EfficientRouteFurthestTaskHeuristic(assignment) * w


	// Part 4: Fairness among different tasks 
	var FairnessCost float64 = 0.0


	// Part 5: time^2 cost 
	w = 1.0 // default 
	if nw, ok := sp.weights["Time2Cost"]; ok {
		//fmt.Println("markmark5",nw)
		w = nw 
	}

	var Time2Cost float64 = 0.0 
	Time2Cost = sp.Time2Cost(assignment) * w // was 0.1


	// Part 6: dependency violation cost
	var DepVioCost float64 = 0.0 


	// must proceed!

	// var ProceedCost float64 = 100000.0 

	// for _, v := range assignment{
	// 	if v < len(sp.Tasks) {
	// 		ProceedCost = 0
	// 	}
	// }

	// if printall {
	// 	fmt.Println("WorkloadImbalanceCost: ", WorkloadImbalanceCost)
	// 	fmt.Println("MutualTaskCost: ", MutualTaskCost)
	// 	fmt.Println("RouteEfficiencyCost: ", RouteEfficiencyCost)
	// 	fmt.Println("Time2Cost: ", Time2Cost)
	// 	fmt.Println("ProceedCost", ProceedCost)
	// }


	// TODO add weights to them 
	return TaskReward + WorkloadImbalanceCost + MutualTaskCost + RouteEfficiencyCost + FairnessCost + Time2Cost + DepVioCost //+  ProceedCost
}
func (sp *ScheduleProblem) TaskReward(assignment []int) float64 {
	var r float64 = 0
	task2drone := make(map[int]int)

	for did,tid := range(assignment) {
		if (tid >= 0 && tid < len(sp.Tasks) && sp.Tasks[tid].IsShadowRequest == false) {
			r -= 1000.0
			//PrintLog(LogDebug, fmt.Sprintln("Task Reward ", r, did, tid))

			task2drone[tid] = did 
		}

		if tid >= len(sp.Tasks) {
			r -= 1000.0 / float64(sp.PredictedTasks[tid-len(sp.Tasks)].Depth + 1)
			//PrintLog(LogDebug, fmt.Sprintln("Task Reward ", r, did, tid))

		}

	}

	//PrintLog(LogDebug, fmt.Sprintln("Task Reward", r, "Assignment", assignment))




	for did,tid := range(assignment) {
		if tid >= 0 && tid < len(sp.Tasks) && sp.Tasks[tid].IsShadowRequest == true {
			//var current_did int
			if sp.Tasks[tid].LocalShadowRequestParent >= 0 {
				if current_did, ok := task2drone[sp.Tasks[tid].LocalShadowRequestParent]; ok {
				// The assignment has the current drone 
					var time2fly float64 = sp.fra.droneBackend.RemainingFlyingTime(sp.Drones[current_did].ID)
					var time2target float64 = sp.fra.droneBackend.FlyingTimeEst(sp.Drones[did].ID, sp.Drones[did].Loc, sp.Tasks[tid].Loc)

					if time2fly - 5  > time2target {
						// add cost?
					} else {
						r -= 1000.0 - (time2target - (time2fly - 5))
					}

				} else {
				// The assignment doesn't have the current drone
					r = r + 100.0 // should be penilized because the request is a non-interruptible one. 
				}
			} else {
				current_did_global := sp.Tasks[tid].ShadowRequestParentDroneID

				var time2fly float64 = sp.fra.droneBackend.RemainingFlyingTime(current_did_global)
				var time2target float64 = sp.fra.droneBackend.FlyingTimeEst(sp.Drones[did].ID, sp.Drones[did].Loc, sp.Tasks[tid].Loc)

				if time2fly - 5  > time2target {
					// add cost?
				} else {
					r -= 1000.0 - (time2target - (time2fly - 5))
				}
			}

		}
	}

	return r 
}
func (sp *ScheduleProblem) WorkloadImbalanceCost(assignment []int) float64 {
	if len(sp.PredictedTaskCluster) <= 1 {
		return 0.0
	}

	// compute the finish time of each 'cluster' in a greedy way 
	// each drone is corresponding to one cluster, so just loop through all drones 

	taskmap := make(map[int]bool)

	for _, nexttask := range assignment {
		taskmap[nexttask] = true 
		if nexttask == -1 {
			return 0.0 
		}
	}


	var maxcost float64 = 0.0 
	var mincost float64 = 100000000000000.0 


	for did, drone := range sp.Drones {
		clusterID := sp.DroneInfo[did].ClusterID
		nextTask := assignment[did]

		if nextTask < 0 {
			continue
		}

		var cost float64 = 0.0 

		// if the next task is a real task 
		if nextTask < len(sp.Tasks) {
			cost = sp.fra.droneBackend.FlyingTimeEst(did, drone.Loc, sp.Tasks[nextTask].Loc)
			// TODO use predicted duration ?
			cost += sp.Tasks[nextTask].Duration


			// the nextTask could belong to the same cluster
			if sp.TaskInfo[nextTask].ClusterID == clusterID {
				// use a greedy algorithm to cover all unassigned tasks in the cluster

				sameClusterTask := make(map[int][3]float64)
				currentLoc := sp.Tasks[nextTask].Loc


				for tid, task := range sp.Tasks {
					if sp.TaskInfo[tid].ClusterID == clusterID {
						if _, ok := taskmap[tid]; ok {
							continue
						} else {
							sameClusterTask[tid] = task.Loc 
						}
					}
				}

				for {
					if len(sameClusterTask) > 0 {
						newNextTask := findClosestKey(currentLoc, sameClusterTask, distance)
						cost += sp.fra.droneBackend.FlyingTimeEst(did, currentLoc, sp.Tasks[newNextTask].Loc)
						// TODO use predicted duration ?
						cost += sp.Tasks[newNextTask].Duration

						currentLoc = sp.Tasks[newNextTask].Loc

						delete(sameClusterTask, newNextTask)
					} else {
						break
					}
				}


				// TODO fly to the closest next predicted task
				// should change this to the closest but most 'recent' (no dependency) predicted task.

				predictedTask := make(map[int][3]float64)

				for tid, task := range sp.PredictedTasks {
					if sp.PredictedTaskClusterAssignment[tid] == clusterID {
						predictedTask[tid] = task.Location 
					}
				}


				if len(predictedTask) > 0 {
					nextPredictedTask := findClosestKey(currentLoc, predictedTask, distance)
					cost += sp.fra.droneBackend.FlyingTimeEst(did, currentLoc, sp.PredictedTasks[nextPredictedTask].Location)
				}

			} else { // Or, the next Task belongs to a different cluster

				// TODO fly to the closest next predicted task
				// should change this to the closest but most 'recent' (no dependency) predicted task.

				currentLoc := sp.Tasks[nextTask].Loc
				predictedTask := make(map[int][3]float64)

				for tid, task := range sp.PredictedTasks {
					if sp.PredictedTaskClusterAssignment[tid] == clusterID {
						predictedTask[tid] = task.Location 
					}
				}


				if len(predictedTask) > 0 {
					nextPredictedTask := findClosestKey(currentLoc, predictedTask, distance)
					cost += sp.fra.droneBackend.FlyingTimeEst(did, currentLoc, sp.PredictedTasks[nextPredictedTask].Location)
				}

			}

		} else { // the next task is a prediction task 
			nextTask = nextTask-len(sp.Tasks)

			cost = sp.fra.droneBackend.FlyingTimeEst(did, drone.Loc, sp.PredictedTasks[nextTask].Location)
			
			// if the predicted task belongs to the same cluster (most often)
			if sp.PredictedTaskClusterAssignment[nextTask] == clusterID {
				// do nothing

			} else { // this should not happen, it may happen from the 'findFirstAssignment' function

				// fly the drone back to the right cluster ...

				predictedTask := make(map[int][3]float64)
				currentLoc := sp.PredictedTasks[nextTask].Location

				for tid, task := range sp.PredictedTasks {
					if sp.PredictedTaskClusterAssignment[tid] == clusterID {
						predictedTask[tid] = task.Location 
					}
				}

				if len(predictedTask) > 0 {
					nextPredictedTask := findClosestKey(currentLoc, predictedTask, distance)
					cost += sp.fra.droneBackend.FlyingTimeEst(did, currentLoc, sp.PredictedTasks[nextPredictedTask].Location)
				}

			}


		}

		if cost > maxcost {
			maxcost = cost 
		}

		if cost < mincost {
			mincost = cost 
		}


	}

	//fmt.Println("Workload Imbalance Cost", mincost, maxcost, (maxcost-mincost) / (maxcost + 1.0))


	//return (maxcost-mincost) / (maxcost + 1.0)
	return maxcost
}


func (sp *ScheduleProblem) MutualTaskCost(assignment []int) float64{

	coveredTask := make(map[int]float64)

	for tid, _ := range sp.TaskInfo {
		sp.TaskInfo[tid].Utility = 0.0
	}

	for _, tid := range assignment {
		if tid < 0 {
			continue
		}

		if tid < len(sp.Tasks) {
			sp.TaskInfo[tid].Utility = 1.0
			coveredTask[tid] = 1.0
		}
	}

	// use Utility field to count the overlap...

	

	//for tid, pred := range sp.PerTaskPrediction {
	for _, tid := range assignment {
		if tid < 0 {
			continue 
		}

		if tid < len(sp.Tasks) {
			
			pred := sp.PerTaskPrediction[tid] 

			if pred != nil {
				if len(pred.Nodes) > 0 {

					if pred.Nodes[0].CancelOtherTask == true {
						
						// TODO this is slow, change it to RTree
						for tid2, _ := range sp.TaskInfo {
							if tid2 == tid {
								continue
							}
							d := distance(sp.Tasks[tid].Loc, sp.Tasks[tid2].Loc)
							if d < pred.Nodes[0].CancelOtherTaskRadius { 
								sp.TaskInfo[tid2].Utility += (1.0-d/pred.Nodes[0].CancelOtherTaskRadius) * pred.Nodes[0].Prob
								// if sp.TaskInfo[tid2].Utility < 0.0 {
								// 	sp.TaskInfo[tid2].Utility = 0.0
								// }

								coveredTask[tid2] = 1.0 / sp.TaskInfo[tid2].Utility
							}

						}

					}
				}

			}
		}
	}

	var total_utility float64 = 0.0
	//var total_utility_expect float64 = 0.0

	for _,v := range coveredTask {
		total_utility += v
		//total_utility_expect += 1.0

	}

	// for _, tid := range assignment {
	// 	if tid < len(sp.Tasks) {
	// 		total_utility += sp.TaskInfo[tid].Utility
	// 		total_utility_expect += 1.0
	// 	}
	// }

	return -total_utility
}

func (sp *ScheduleProblem) Time2Cost(assignment []int) float64 {
	var cost float64 = 0.0

	for did, tid := range assignment {
		if tid < 0 {
			continue 
		}

		if tid < len(sp.Tasks) {
			ft := sp.fra.droneBackend.FlyingTimeEst(did, sp.Drones[did].Loc, sp.Tasks[tid].Loc)
			cost += ft*ft
		}

	}

	return cost
}


func (sp *ScheduleProblem) EfficientRouteCost(assignment []int) float64 {
	var cost float64 = 0.0

	loc_center := [3]float64{0.0,0.0,0.0}
	var tn float64 = 0 

	for _, task := range sp.Tasks {
		loc_center = locAdd(task.Loc, loc_center)
		tn += 1
	}

	for _, task := range sp.PredictedTasks {
		loc_center = locAdd(task.Location, loc_center)
		tn += task.Prob
	}



	if tn > 0 {
		loc_center = locDivide(loc_center, float64(tn))
	
		for did, tid := range assignment {
			if tid < 0 {
				continue 
			}

			if tid < len(sp.Tasks) {
				
				t1 := sp.fra.droneBackend.FlyingTimeEst(did, sp.Drones[did].Loc, loc_center)
				t2 := sp.fra.droneBackend.FlyingTimeEst(did, sp.Tasks[tid].Loc, loc_center)

				var EfficientRouteReward float64 = t2 - t1 

				if EfficientRouteReward < 0.0 {
					EfficientRouteReward = 0.0
				}

				cost -= EfficientRouteReward * 1.0 

			}

		}
	}

	return cost
}

func (sp *ScheduleProblem) EfficientRouteFurthestTaskHeuristic(assignment []int) float64 {
	var cost float64 = 0.0

	loc_center := [3]float64{0.0,0.0,0.0}
	tn := 0 

	for _, task := range sp.Tasks {
		loc_center = locAdd(task.Loc, loc_center)
		tn += 1
	}

	if tn > 0 {
		loc_center = locDivide(loc_center, float64(tn))


		furthest_tid := 0
		d_max := 0.0 

		for tid, task := range sp.Tasks {
			d := distance(task.Loc, loc_center)
			if d > d_max {
				furthest_tid = tid
			}
		}

		for did, tid := range assignment {
			if tid < 0 {
				continue
			}

			if tid == furthest_tid {

				vec1 := locSubtract(loc_center, sp.Drones[did].Loc)
				vec2 := locSubtract(sp.Tasks[tid].Loc, sp.Drones[did].Loc)

				if locDot(vec1, vec2) < 0.0 {
					cost -= 1.0
				}
			}
		}
	}

	return cost 
}





func (sp *ScheduleProblem) BeforeRandomAssignment(assignment []int) {

	taskAssigned := make([]bool, len(sp.Tasks) + len(sp.PredictedTasks)) 

	for i:=0; i<len(sp.Tasks) + len(sp.PredictedTasks); i++ {
		taskAssigned[i] = false
	}

	for _, nexttask := range assignment {
		if nexttask < 0 {
			continue
		}

		//if nexttask < len(sp.Tasks) {
		taskAssigned[nexttask] = true 
		//}
	}

	sp.unassignedTask = make([]int,0)

	for i:=0; i<len(sp.Tasks) + len(sp.PredictedTasks); i++ {
		if taskAssigned[i] == false {
			sp.unassignedTask = append(sp.unassignedTask, i)
		}
	}

	sp.unassignedDrone = make([]int,0)
	for did,tid := range assignment {
		if tid < 0 {
			sp.unassignedDrone = append(sp.unassignedDrone, did)
		}
	}



}

func (sp *ScheduleProblem) DroneTaskAffinity(did int, tid int) bool {
	if tid < len(sp.Tasks) {
		req := sp.Tasks[tid]

		if req.acquiredDroneID == -1 {
			return true 
		} else if req.acquiredDroneID != sp.Drones[did].ID {
			return false 
		}
	}

	return true
}

const (
	RELOCATE = 0 
	SWAP = 1
	DEACTIVE = 2
	ACTIVE = 3 
)

// A simpler version 
func (sp *ScheduleProblem) RandomAssignmentBasic(in_assignment []int) []int {
	out_assignment := make([]int, len(in_assignment))

	copy(out_assignment, in_assignment)

	var random_type int = RELOCATE

	if len(sp.Tasks) + len(sp.PredictedTasks) > len(sp.Drones) {
		coin := rand.Intn(3)

		if coin == 0 {
			random_type = RELOCATE
		} else if coin == 1{
			random_type = SWAP
		} else if coin == 2 {
			random_type = ACTIVE
		} else if coin == 3 {
			random_type = DEACTIVE
		}

	} else { // random swap assignment 
		coin := rand.Intn(3)

		if coin == 0 {
			random_type = SWAP
		} else if coin == 1 {
			random_type = DEACTIVE
		} else if coin == 2 {
			random_type = ACTIVE // may have a unchanged output 
		}

	}


	if random_type == RELOCATE {
		did := rand.Intn(len(sp.Drones))
		tid := sp.unassignedTask[rand.Intn(len(sp.unassignedTask))]
		out_assignment[did] = tid 
	}

	if random_type == SWAP {
		did1 := rand.Intn(len(sp.Drones))
		did2 := rand.Intn(len(sp.Drones))
		out_assignment[did1], out_assignment[did2] = out_assignment[did2], out_assignment[did1]
	}

	if random_type == ACTIVE {
		if len(sp.unassignedDrone) > 0 && len(sp.unassignedTask) > 0 {
			did := sp.unassignedDrone[rand.Intn(len(sp.unassignedDrone))]
			tid := sp.unassignedTask[rand.Intn(len(sp.unassignedTask))]
			out_assignment[did] = tid  
		}
	}

	if random_type == DEACTIVE {
		did := rand.Intn(len(sp.Drones))
		out_assignment[did] = -1 
	}

	return out_assignment

}

func (sp *ScheduleProblem) RandomAssignment(in_assignment []int) []int {

	out_assignment := make([]int, len(in_assignment))

	copy(out_assignment, in_assignment)


	if len(sp.PredictedTaskCluster) <= 1 {
		// if there are unassigned tasks 

		var random_type int = RELOCATE

		if len(sp.Tasks) > len(sp.Drones) {
			coin := rand.Intn(3)

			if coin == 0 {
				random_type = RELOCATE
			} else if coin == 1{
				random_type = SWAP
			} else if coin == 2 {
				random_type = ACTIVE
			} else if coin == 3 {
				random_type = DEACTIVE
			}

		} else { // random swap assignment 
			coin := rand.Intn(2)

			if coin == 0 {
				random_type = SWAP
			} else if coin == 1 {
				random_type = DEACTIVE
			}

		}


		if random_type == RELOCATE {
			did := rand.Intn(len(sp.Drones))
			tid := sp.unassignedTask[rand.Intn(len(sp.unassignedTask))]
			out_assignment[did] = tid 
		}

		if random_type == SWAP {
			did1 := rand.Intn(len(sp.Drones))
			did2 := rand.Intn(len(sp.Drones))
			out_assignment[did1], out_assignment[did2] = out_assignment[did2], out_assignment[did1]
		}

		if random_type == ACTIVE {
			if len(sp.unassignedDrone) > 0 && len(sp.unassignedTask) > 0 {
				did := sp.unassignedDrone[rand.Intn(len(sp.unassignedDrone))]
				tid := sp.unassignedTask[rand.Intn(len(sp.unassignedTask))]
				out_assignment[did] = tid  
			}
		}

		if random_type == DEACTIVE {
			did := rand.Intn(len(sp.Drones))
			out_assignment[did] = -1 
		}


	} else {
		// choose a random drone 
		did := rand.Intn(len(sp.Drones))
		clusterID := sp.DroneInfo[did].ClusterID

		if len(sp.TaskPerCluster[clusterID]) == 0 {
			if out_assignment[did] >= len(sp.Tasks) {
				// choose another drone, stole a task from it if possible 
				did2 := rand.Intn(len(sp.Drones))

				nTask := len(sp.TaskPerCluster[sp.DroneInfo[did2].ClusterID])
				if nTask == 0 {
					// do nothing 


				} else if nTask == 1 {
					out_assignment[did2] =  sp.DroneClosestPrediction[did2] + len(sp.Tasks)
					out_assignment[did] = sp.TaskPerCluster[sp.DroneInfo[did2].ClusterID][0]
				} else {

					rind := rand.Intn(nTask)
					tid := sp.TaskPerCluster[sp.DroneInfo[did2].ClusterID][rind]

					if tid == out_assignment[did2] {
						out_assignment[did] = tid 
						out_assignment[did2] = sp.TaskPerCluster[sp.DroneInfo[did2].ClusterID][(rind + rand.Intn(nTask-1) + 1) % nTask]  // a random tid but not the same as tid

					} else {
						out_assignment[did] = tid
					}
				}

			} else if out_assignment[did] >= 0 {
				// choose to fly to its own prediceted task 
				// here is a bug, the old "out_assignment[did]"" need to be reassigned 
				old_tid_cluster := sp.TaskInfo[out_assignment[did]].ClusterID

				for did2, info := range sp.DroneInfo {
					if info.ClusterID == old_tid_cluster {
						out_assignment[did2] = out_assignment[did]
						break
					}
				}

				out_assignment[did] = sp.DroneClosestPrediction[did] + len(sp.Tasks)

			} else {
				// out_assignment[did] == -1 
				// do nothing????


			}
		} else {
			// do nothing?
			// handle old tid 
			if out_assignment[did] >= len(sp.Tasks) {
				tid := sp.TaskPerCluster[clusterID][rand.Intn(len(sp.TaskPerCluster[clusterID]))]
				out_assignment[did] = tid

			} else if out_assignment[did] >= 0{

				old_tid_cluster := sp.TaskInfo[out_assignment[did]].ClusterID
				if old_tid_cluster == clusterID {
					tid := sp.TaskPerCluster[clusterID][rand.Intn(len(sp.TaskPerCluster[clusterID]))]
					out_assignment[did] = tid 
				} else {
					for did2, info := range sp.DroneInfo {
						if info.ClusterID == old_tid_cluster {
							out_assignment[did2] = out_assignment[did]
							break
						}
					}

					tid := sp.TaskPerCluster[clusterID][rand.Intn(len(sp.TaskPerCluster[clusterID]))]
					out_assignment[did] = tid
				}
			} else {
				// out_assignment[did] == -1
			}
		}
	}

	return out_assignment
}

func (sp *ScheduleProblem) RejectAssignment(in_assignment []int) bool {
	for did, tid := range in_assignment {
		if tid == -1 {
			continue
		}

		if sp.DroneTaskAffinity(did,tid) == false {
			return true 
		}
	}

	return false
}

// set non-achievable assignment to -1 (none)
// set RTH drone to -1 
func (sp *ScheduleProblem) FilterAssignment(in_assignment []int) []int {
	
	out_assignment := make([]int, len(in_assignment))

	for did, tid := range in_assignment {
		out_assignment[did] = tid
		
		if tid == -1 {
			continue
		}

		if sp.DroneTaskAffinity(did,tid) == false {
			out_assignment[did] = -1
		}

		if sp.Drones[did].RTH == true {
			out_assignment[did] = -1	
		}
	}

	return out_assignment
}

// Greedy just base on distance 
// optimize for minimizing the total distance
func (sp *ScheduleProblem) FindFirstAssignment() []int {
	// greedy 
	var pairs AssignmentList
	var max_distance float64 = 0.0 


	// Efficient Route Cost 
	loc_center := [3]float64{0.0,0.0,0.0}
	tn := 0 

	for _, task := range sp.Tasks {
		loc_center = locAdd(task.Loc, loc_center)
		tn += 1
	}

	if tn > 0 {
		loc_center = locDivide(loc_center, float64(tn))
	}


	for did, drone := range sp.Drones {
		for tid, task := range sp.Tasks {
			if sp.DroneTaskAffinity(did,tid) == false {
				continue
			}

			if sp.Drones[did].RTH == true {
				continue 
			}

			var ass DroneAssignment 

			ass.DroneId = did
			ass.RequestId = tid

			ass.Cost = sp.fra.droneBackend.FlyingTimeEst(drone.ID, drone.Loc, task.Loc)


			t1 := sp.fra.droneBackend.FlyingTimeEst(drone.ID, drone.Loc, loc_center)
			t2 := sp.fra.droneBackend.FlyingTimeEst(drone.ID, task.Loc, loc_center)

			var EfficientRouteReward float64 = t2 - t1 

			if EfficientRouteReward < 0.0 {
				EfficientRouteReward = 0.0
			}

			ass.Cost -= EfficientRouteReward * 4.0 


			if ass.Cost > max_distance {
				max_distance = ass.Cost 
			}

			pairs.ass = append(pairs.ass, ass)
		}
	}

	for did, drone := range sp.Drones {
		for tid, task := range sp.PredictedTasks {
			if sp.DroneTaskAffinity(did,tid) == false {
				continue
			}

			var ass DroneAssignment 

			ass.DroneId = did
			ass.RequestId = tid + len(sp.Tasks)

			ass.Cost = sp.fra.droneBackend.FlyingTimeEst(drone.ID, drone.Loc, task.Location)

			ass.Cost += max_distance // make sure the predicted tasks always have higher cost 

			pairs.ass = append(pairs.ass, ass)
		}
	}

	sort.Sort(&pairs)

	task_map := make(map[int]bool)
	drone_map := make(map[int]bool)


	

	assignment := make([]int, sp.assignment_size)


	for ind, _ := range assignment {
		assignment[ind] = -1 
	}

	for _, ass := range pairs.ass {
		rid := ass.RequestId
		did := ass.DroneId 

		//fmt.Println(rid, did, ass.Cost)

		if _,ok:= task_map[rid]; ok {
			continue
		}

		if _,ok := drone_map[did]; ok {
			continue
		}
		//fmt.Println("-->?",rid, did, ass.Cost, sp.PredictedTaskCluster)

		//same cluster requirement?
		if len(sp.PredictedTaskCluster) > 1 {
			if rid < len(sp.Tasks) {
				if sp.TaskInfo[rid].ClusterID != sp.DroneInfo[did].ClusterID {
					continue
				}
			} else {
				if sp.PredictedTaskClusterAssignment[rid-len(sp.Tasks)] != sp.DroneInfo[did].ClusterID {
					continue 
				}
			}
		}

		//fmt.Println("-->",rid, did, ass.Cost)
		task_map[rid] = true
		drone_map[did] = true 

		assignment[did] = rid 
		
	}

	return assignment
}





// Suppose we know the prediction of each request 
// - per-request prediction (for cancelling tasks)
// - global prediction (for workload balancing)
//
// Step 1: Do clusterring on the global prediction + existing request
// Step 1: In the clusterring, consider the total running time, trying to
//         balance each cluster 
// Step 2: Assign each drone to the nearest cluster
// Step 3: Find the best assignment using simulated annealing 
//
//
// XXXX Step 3: Assign weights
// XXXX        (1) intra-cluster weights, prefer flying to the farest point first 
// XXXX        (2) inter-cluster weights, prefer flying/or staying within the same cluster  
// 
// XXXX Step 4: For each assignment, if the task has 'cancel' prediction, reduce the utility. 
// 
// List all the weights, train it using RL? from the Log  



// BeeCluster's Scheduler (main entry of this file)
func (fra *Framework) ScheduleBeeCluster() {
	waitStepSignal("SchedulerBeeCluster Begin")

	//fmt.Println("are we here?")

	drones := fra.droneBackend.GetDrones()
	//var assignments []DroneAssignment


	justReachTargetReqIds := make(map[int]int)


	var free_drone int = 0
	var open_request int = 0 
	//var min_assignments int = 0 

	for i:=0; i<len(drones); i++ {
		PrintLog(LogDebug, fmt.Sprintln("drone ", i, "phase",drones[i].Phase, "battery", drones[i].BatteryLevel, drones[i].RTH ))
		if drones[i].Phase == 2 {
			justReachTargetReqIds[fra.drone2request[i]] = 1
		} else if drones[i].IsDroneOnline && drones[i].RTH == false {
			free_drone += 1
		}
	}

	for j:=0; j<fra.requestNum;j++ { // todo: optimize this for-loop...
		if _,ok := justReachTargetReqIds[j]; !ok && (fra.Requests[j].phase < 2) {
			open_request += 1 			
		}
	}

	// if free_drone < open_request {
	// 	min_assignments = free_drone
	// } else {
	// 	min_assignments = open_request
	// }

	// request center 

	center := [3]float64{0,0,0}
	counter := 0.0

	for j:=0; j<fra.requestNum; j++ {
		if _,ok := justReachTargetReqIds[j]; !ok && (fra.Requests[j].phase < 2) {
			center[0] += fra.Requests[j].Loc[0]
			center[1] += fra.Requests[j].Loc[1]
			center[2] += fra.Requests[j].Loc[2]
			counter += 1.0
		}
	}

	if counter > 0.5 {
		center[0] /= counter
		center[1] /= counter
		center[2] /= counter
	}


	t0 := makeTimestampMicro()
	globalPredicion := fra.getGlobalPrediction()
	t1 := makeTimestampMicro()
	fra.LogPredictionTime += float64(t1-t0)/1000000.0
	fra.AppendLog()
	//time.Sleep(time.Duration(10000 * time.Microsecond))

	

	// Setup the schedule problem 
	sp := new(ScheduleProblem)


	// // workload balancing 
	// // do k-means partitioning

	sp.PredictedTaskClusterAssignment, sp.PredictedTaskCluster = fra.PredictionPartitioning(globalPredicion, len(drones))

	sp_drone_map := make(map[int]int)
	sp_request_map := make(map[int]int)


	var drone_locs [][3]float64

	for i:=0; i<len(drones); i++ {
		if drones[i].IsDroneOnline {
			// consider all the drones (not for real system)
			//sp_drone_map[len(sp.Drones)] = i 
			sp_drone_map[i] = len(sp.Drones)
			sp.Drones = append(sp.Drones, drones[i])
			drone_locs = append(drone_locs, drones[i].Loc)
		}
	}

	//fmt.Println(len(drones), len(sp.Drones))

	drone2cluster := findBestMatch(drone_locs, sp.PredictedTaskCluster, distance)

	for i:=0; i<len(drones); i++ {
		if drones[i].IsDroneOnline {
			var info ScheduleProblemDroneInfo

			info.ClusterID = drone2cluster[i]

			sp.DroneInfo = append(sp.DroneInfo, &info)
		}
	}



	for j:=0; j<fra.requestNum; j++ {
		if _,ok := justReachTargetReqIds[j]; !ok && (fra.Requests[j].phase < 2) {
			sp_request_map[len(sp.Tasks)] = j 
			req := fra.Requests[j]
			sp.Tasks = append(sp.Tasks, &req)	
			
			perTaskPrediction := fra.getPredictionForNode(req.sessionID, req.CodeBlockName)
			sp.PerTaskPrediction = append(sp.PerTaskPrediction, perTaskPrediction)	
			
			// PrintLog(LogDebug, fmt.Sprintln(" Request ID = ", j, "Dumping Predictions", perTaskPrediction))
			// if perTaskPrediction != nil {
			// 	for _, vn := range perTaskPrediction.Nodes {
			// 		PrintLog(LogDebug, fmt.Sprintln(" Req ",j," type ", vn.NodeType, " depth ", vn.Depth, " loc ", vn.Location, " cancel? ", vn.CancelOtherTask))
			// 	}
			// }

			var info ScheduleProblemTaskInfo
			info.ClusterID = findClosestIndex(req.Loc, sp.PredictedTaskCluster, distance)
			sp.TaskInfo = append(sp.TaskInfo, &info)


			// add shadow request 
			if req.nonInterruptibleFlag == true {
				var shadowReq Request

				shadowReq = req 
				shadowReq.IsShadowRequest = true 
				//shadowReq.IsShadowRequestParentDynamic = true 
				shadowReq.LocalShadowRequestParent = len(sp.Tasks)-1
				shadowReq.ShadowRequestParentDroneID = -1

				sp.ShadowTasks = append(sp.ShadowTasks, shadowReq)
				sp.Tasks = append(sp.Tasks, &shadowReq)

				sp.PerTaskPrediction = append(sp.PerTaskPrediction, sp.PerTaskPrediction[len(sp.PerTaskPrediction)-1])
				sp.TaskInfo = append(sp.TaskInfo, sp.TaskInfo[len(sp.TaskInfo)-1])
			}
		}

		// add shadow request (when the paraent request is being handled)
		if _,ok := justReachTargetReqIds[j]; ok || (fra.Requests[j].phase == 2) {
			req := fra.Requests[j]
			if req.nonInterruptibleFlag == true {
				var shadowReq Request

				shadowReq = req 
				shadowReq.IsShadowRequest = true
				//shadowReq.IsShadowRequestParentDynamic = false 
				shadowReq.LocalShadowRequestParent = -1

				shadowReq.ShadowRequestParentDroneID = req.droneHandler // TO VERIFY, this might be undefined ...

				sp.ShadowTasks = append(sp.ShadowTasks, shadowReq)
				sp.Tasks = append(sp.Tasks, &shadowReq)

				perTaskPrediction := fra.getPredictionForNode(req.sessionID, req.CodeBlockName)
				sp.PerTaskPrediction = append(sp.PerTaskPrediction, perTaskPrediction)	
			
				var info ScheduleProblemTaskInfo
				info.ClusterID = findClosestIndex(req.Loc, sp.PredictedTaskCluster, distance)
				sp.TaskInfo = append(sp.TaskInfo, &info)
			}
		}
	}



	sp.PredictedTasks = globalPredicion.Nodes 
	sp.fra = fra 

	sp.assignment_size = len(sp.Drones)

	// // workload balancing 
	// // do k-means partitioning
	//sp.PredictedTaskClusterAssignment, sp.PredictedTaskCluster = fra.PredictionPartitioning(globalPredicion, len(drones))


	// ready to solve 
	if len(sp.Tasks) == 0 {
		return 
	}

	sp.weights = fra.cfg.BeeClusterWeights

	assignment := sp.Solve() 



	//fmt.Println(assignment)

	var result_assignments []DroneAssignment

	for i:=0; i<len(drones); i++ {
		if drones[i].IsDroneOnline == false {
			continue
		}

		if drones[i].Phase != 2 {
			if assignment[sp_drone_map[i]] < 0 {
				continue
			}

			if assignment[sp_drone_map[i]] < len(sp.Tasks) {
				if sp.Tasks[assignment[sp_drone_map[i]]].IsShadowRequest == false { 
					var ass DroneAssignment
					ass.DroneId = i 
					ass.RequestId = sp_request_map[assignment[sp_drone_map[i]]]

					result_assignments = append(result_assignments, ass)
				}
			}
		}
	} 



	for _,ass:= range result_assignments {
		
			rid := ass.RequestId
			droneid := ass.DroneId
			cost := ass.Cost 

			v:=fra.Requests[rid]

			if v.associatedDrone != -1 && v.associatedDrone != droneid {
				// don't assign two drones to the same request 
				// cancel the other drone 
				other_drone_id := v.associatedDrone

				if drones[other_drone_id].Phase == 2 {
					// should ignore new assignment 
					PrintLog(LogDebug, fmt.Sprintf("[scheduler] ignore duplicate assignment %d on %d\n", droneid, rid))
					continue 
				}

				// only if the other drone is also flying to the same target
				if fra.drone2request[other_drone_id] == rid {
					fra.droneBackend.UnsetTarget(other_drone_id)
					PrintLog(LogInfo, fmt.Sprintf("[scheduler] unassign drone %d to %d\n", other_drone_id, rid))
				}
			}

			PrintLog(LogInfo, fmt.Sprintf("[scheduler] assign drone %d to %d, cost %.2f\n", droneid, rid, cost))

			v.associatedDrone = droneid
			v.phase = 1
			fra.Requests[rid] = v
			
			fra.drone2request[droneid] = rid 

			fra.droneBackend.SetTarget(droneid, fra.Requests[rid].Loc)
			fra.droneBackend.SetSpeculationTarget(droneid, [][3]float64{})
		
	}

	for i:=0; i<len(drones); i++ {
		if drones[i].IsDroneOnline == false {
			continue
		}

		if drones[i].Phase == 0 {
			if assignment[sp_drone_map[i]] < 0 {
				continue
			}

			// assign drone to shadow tasks
			if assignment[sp_drone_map[i]] < len(sp.Tasks) {
				if sp.Tasks[assignment[sp_drone_map[i]]].IsShadowRequest {
					fra.droneBackend.SetSpeculationTarget(i, [][3]float64{sp.Tasks[assignment[sp_drone_map[i]]].Loc})
				}
			}

			if assignment[sp_drone_map[i]] >= len(sp.Tasks) {

				fra.droneBackend.SetSpeculationTarget(i, [][3]float64{sp.PredictedTasks[assignment[sp_drone_map[i]]-len(sp.Tasks)].Location})

			}
		}
	} 

	if vis_log {
		// todo 
	}

	waitStepSignal("SchedulerBeeCluster End")
}

// The following two functions are 'external' functions
func (fra *Framework) getPredictionForNode(sessionID int, name string) *Prediction {
	return fra.applicationContexts[sessionID].dag.getPredictionForNode(name)
}

func (fra *Framework) getGlobalPrediction() *Prediction {
	var mPrediction Prediction 

	for _, context := range fra.applicationContexts {
		if context.isActive {
			pred := context.dag.getGlobalPrediction()
			for _, vn := range pred.Nodes {
				if vn.CancelOtherTask == false {
					mPrediction.Nodes = append(mPrediction.Nodes, vn)
				}
			}
			//mPrediction.Nodes = append(mPrediction.Nodes, pred.Nodes...)
			mPrediction.TS = pred.TS
		}
	} 

	return &mPrediction 
}


type taskgroup struct {
	ids []int 
}

func (fra *Framework) PredictionPartitioning(preds *Prediction, k int) ([]int, [][3]float64) {
	
	var basedepth int = 0 
	depthcounter := make(map[int]int)
	assignment := make([]int, len(preds.Nodes))
	cluster := make([][3]float64, k)
	cluster_acu := make([][3]float64, k)
	cluster_c := make([]float64, k)
	cluster_size := make([]float64, k)


	if len(preds.Nodes) < k {
		return []int{},[][3]float64{} 
	}

	

	var mindepth int = 100000000
	for _, node := range preds.Nodes {
		d := node.Depth 
		if d < mindepth{
			mindepth = d 
		}
		if _, ok := depthcounter[d]; ok {
			depthcounter[d] = depthcounter[d] + 1
		} else {
			depthcounter[d] = 1
		}
	}
	

	var cancel_prediction_counter int = 0 
	for _, node := range preds.Nodes {
		if node.CancelOtherTask == true && node.Depth == mindepth {
			cancel_prediction_counter += 1
		}
	}

	// TODO add a filter to remove the cancel prediction
	// There could be a mix of cancel prediction and other normal predictions 
	// Fix this !!!

	if len(preds.Nodes) - cancel_prediction_counter < k {
		return []int{},[][3]float64{}  
	}




	cc := 0 
	for {
		
		if c,ok := depthcounter[basedepth]; ok {
			cc += c 
			if cc >= k {
				break
			}
		}

		basedepth += 1
	}

	

	// step 1, clusterring on the first few layers

	// initial clusters
	// important... random 
	// old version
	// ind := 0 
	// for _, node := range preds.Nodes {
	// 	if node.Depth <= basedepth {
	// 		cluster[ind] = node.Location
	// 		cluster_size[ind] = 1.0
	// 		ind += 1
	// 		if ind == k {
	// 			break
	// 		}
	// 	}
	// }


	// Option 1 
	// choose the farthest one 
	// candidate := make(map[int][3]float64)
	// var first bool = true
	
	// for ind, node := range preds.Nodes {
	// 	if node.Depth <= basedepth {
	// 		if first {
	// 			cluster[0] = node.Location 
	// 			cluster_size[0] = 1.0

	// 			first = false 
	// 		} else {
	// 			candidate[ind] = node.Location
	// 		}
	// 	}
	// }

	// ind := 1

	// for {
	// 	if ind >= k {
	// 		break
	// 	}

	// 	squareDist := make(map[int]float64)
	// 	for i:=0; i<ind; i++ {
	// 		for k,v := range candidate {
	// 			if _,ok:= squareDist[k]; ok {
	// 				squareDist[k] += distance(v, cluster[i])
	// 			} else {
	// 				squareDist[k] = distance(v, cluster[i])
	// 			}
	// 		}
	// 	}

	// 	var min_k int = 0
	// 	var min_dist float64 = 1000000000.0

	// 	for k,v := range squareDist {
	// 		if v < min_dist {
	// 			min_dist = v
	// 			min_k = k
	// 		}
	// 	}

	// 	// k is the next cluster 
	// 	cluster[ind] = candidate[min_k]
	// 	cluster_size[ind] = 1.0
	// 	ind += 1

	// 	delete(candidate, min_k)
	// } 



	// Option 2 max coverage (works much better)
	candidate := make(map[int][3]float64)
	mindistance := make(map[int]float64)

	var first bool = true
	
	for ind, node := range preds.Nodes {
		if node.Depth <= basedepth {
			if first {
				cluster[0] = node.Location 
				cluster_size[0] = 1.0

				first = false 
			} else {
				candidate[ind] = node.Location
				mindistance[ind] = distance(node.Location, cluster[0])
			}
		}
	}

	ind := 1

	for {
		if ind >= k {
			break
		}

		var max_coverage int = 0
		var max_coverage_ind int = 0
		for ind1,loc1 := range candidate {
			coverage := 0
			for ind2, loc2 := range candidate {
				d := distance(loc1, loc2)

				if d < mindistance[ind2] {
					coverage += 1
				}
			}

			if coverage > max_coverage {
				max_coverage = coverage
				max_coverage_ind = ind1
			}
		}


		// max_coverage_ind is the next cluster 
		cluster[ind] = candidate[max_coverage_ind]
		cluster_size[ind] = 1.0
		

		loc1:=cluster[ind]

		for ind2, loc2 := range candidate {
			d := distance(loc1, loc2)

			if d < mindistance[ind2] {
				mindistance[ind2] = d
			}
		}

		ind += 1
		delete(candidate, max_coverage_ind)
	} 


	//basedepth = 100000


	//fmt.Println(cluster)



	// do k-means
	for it := 0; it < 30; it ++ {
		// assignment
		for ic, _ := range cluster {
			cluster_acu[ic] = [3]float64{0.0, 0.0, 0.0}
			cluster_c[ic] = 0.0 
		}


		for ind, node := range preds.Nodes{
			//if node.Depth <= basedepth {

				var best_d float64 = 10000000.0 
				var best_cluster int = 0 

				for ic, loc := range cluster {
					d := distance(loc, node.Location) // * (1.0+rand.Float64()*(29.0-float64(it))/300.0)// * cluster_size[ic]
					if d < best_d {
						best_d = d 
						best_cluster = ic 
					}
				}

				//
				// if node.Depth <= basedepth {
				// 	best_cluster = assignment[ind]
				// }

				assignment[ind]=best_cluster
				cluster_acu[best_cluster] = locAdd(locMul(node.Location,node.Prob), cluster_acu[best_cluster])
				cluster_c[best_cluster] += node.Prob
			//}
		}


		// update cluster

		for ic, _ := range cluster {
			//if it % 2 == 1 {

			if cluster_c[ic] != 0.0 {
				cluster[ic] = locDivide(cluster_acu[ic], cluster_c[ic])
			}

			
			//}
			cluster_size[ic] = 1.0 + cluster_c[ic] / 100.0
		}
	}


	// step 2, fix the elements in the cluster, do another clustering 

	// for it := 0; it < 1; it ++ {
	// 	// assignment
	// 	for ic, _ := range cluster {
	// 		cluster_acu[ic] = [3]float64{0.0, 0.0, 0.0}
	// 		cluster_c[ic] = 0.0 
	// 	}


	// 	for ind, node := range preds.Nodes{
			
	// 		if node.Depth > basedepth {
	// 			var best_d float64 = 10000000.0 
	// 			var best_cluster int = 0 

	// 			for ic, loc := range cluster {
	// 				d := distance(loc, node.Location) * cluster_size[ic]
	// 				if d < best_d {
	// 					best_d = d 
	// 					best_cluster = ic 
	// 				}
	// 			}
				
	// 			assignment[ind]=best_cluster
	// 		}
			
	// 		ic:= assignment[ind]  

	// 		cluster_acu[ic] = locAdd(locMul(node.Location,node.Prob), cluster_acu[ic])
	// 		cluster_c[ic] += node.Prob
			
	// 	}

	// 	if it == 0 {
	// 		break
	// 	}

	// 	// update cluster
	// 	for ic, _ := range cluster {
	// 		cluster[ic] = locDivide(cluster_acu[ic], cluster_c[ic])
	// 		cluster_size[ic] = 1.0 + cluster_c[ic] / 100.0
	// 	}
	// }


	//fmt.Println(assignment, cluster)
	if vis_log {
		DebugVisualizationPartitioning(preds, assignment, cluster, "os/web/partitioning.png")
	}

	return assignment, cluster
}

func (sp *ScheduleProblem) DebugVisualization(assignment []int, filename string) {
	if vis_log == false {
		return 
	}


	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})
	gc := draw2dimg.NewGraphicContext(img)
	gc.SetStrokeColor(color.RGBA{0xff, 0x00, 0x00, 0xff})
	gc.SetLineWidth(3)


	colors := []color.RGBA{color.RGBA{0xff, 0x00, 0x00, 0xff}, color.RGBA{0x00, 0xff, 0x00, 0xff}, color.RGBA{0x00, 0x00, 0xff, 0xff},color.RGBA{0xff, 0x00, 0xff, 0xff}, color.RGBA{0xff, 0xff, 0x00, 0xff}, color.RGBA{0x00, 0xff, 0xff, 0xff}}
	colors_dark := []color.RGBA{color.RGBA{0x88, 0x00, 0x00, 0xff}, color.RGBA{0x00, 0x88, 0x00, 0xff}, color.RGBA{0x00, 0x00, 0x88, 0xff},color.RGBA{0x88, 0x00, 0x88, 0xff}, color.RGBA{0x88, 0x88, 0x00, 0xff}, color.RGBA{0x00, 0x88, 0x88, 0xff}}

	for ind, drone := range sp.Drones {
		loc := drone.Loc

		clusterid := sp.DroneInfo[ind].ClusterID

		gc.SetStrokeColor(colors[clusterid% len(colors)])

		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-7,y-7)
		gc.LineTo(x+7,y-7)
		gc.LineTo(x+7,y+7)
		gc.LineTo(x-7,y+7)
		gc.LineTo(x-7,y-7)
		gc.Close()
		gc.FillStroke()
	}

	for ind, task := range sp.Tasks {
		loc := task.Loc 

		clusterid := sp.TaskInfo[ind].ClusterID

		//fmt.Println(clusterid, len(colors), )

		if clusterid == -1 {
			continue 
		}

		gc.SetStrokeColor(colors[clusterid% len(colors)])

		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()

	}

	for ind, task := range sp.PredictedTasks {
		loc := task.Location

		if ind >= len(sp.PredictedTaskClusterAssignment) {
			continue
		}

		clusterid := sp.PredictedTaskClusterAssignment[ind]

		if clusterid == -1 {
			continue 
		}

		gc.SetStrokeColor(colors_dark[clusterid% len(colors)])

		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()

	}

	for ind, ass := range assignment {
		loc := sp.Drones[ind].Loc
		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		x2 := x
		y2 := y

		if ass >= len(sp.Tasks) {
			gc.SetStrokeColor(color.RGBA{0x00, 0x00, 0xaf, 0xff})
			loc := sp.PredictedTasks[ass - len(sp.Tasks)].Location
			x2 = loc[0] + 400.0
			y2 = loc[1] + 400.0 

		} else if ass >= 0 {
			gc.SetStrokeColor(color.RGBA{0xaf, 0x00, 0x00, 0xff})
			loc := sp.Tasks[ass].Loc

			x2 = loc[0] + 400.0
			y2 = loc[1] + 400.0 
		} else {
			gc.SetStrokeColor(color.RGBA{0x00, 0x00, 0x00, 0xff})
		}

		gc.MoveTo(x,y)
		gc.LineTo(x2,y2)
		gc.Close()
		gc.FillStroke()

	}


	
	f, err := os.Create(filename)
	if err != nil {
		PrintLog(LogError, fmt.Sprintln("DebugVisualizationAssignment Error", err))
	} else {
		png.Encode(f, img)	
		f.Close()
	}

}

func DebugVisualizationPartitioning(pred *Prediction, assignment []int, cluster [][3]float64, name string) {
	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})
	gc := draw2dimg.NewGraphicContext(img)
	gc.SetStrokeColor(color.RGBA{0xff, 0x00, 0x00, 0xff})
	gc.SetLineWidth(3)


	colors := []color.RGBA{color.RGBA{0xff, 0x00, 0x00, 0xff}, color.RGBA{0x00, 0xff, 0x00, 0xff}, color.RGBA{0x00, 0x00, 0xff, 0xff},color.RGBA{0xff, 0x00, 0xff, 0xff}, color.RGBA{0xff, 0xff, 0x00, 0xff}, color.RGBA{0x00, 0xff, 0xff, 0xff}}

	for ind, node := range pred.Nodes {
		loc := node.Location

		gc.SetStrokeColor(colors[assignment[ind]% len(colors)])

		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()
	}

	for ind, loc := range cluster {

		gc.SetStrokeColor(colors[ind % len(colors)])

		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-5,y-5)
		gc.LineTo(x+5,y-5)
		gc.LineTo(x+5,y+5)
		gc.LineTo(x-5,y+5)
		gc.LineTo(x-5,y-5)
		gc.Close()
		gc.FillStroke()
	}





	f, err := os.Create(name)
	if err != nil {
		PrintLog(LogError, fmt.Sprintln("DebugVisualizationPartitioning Error", err))
	} else {
		png.Encode(f, img)	
		f.Close()
	}


}

