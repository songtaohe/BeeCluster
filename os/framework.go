package main 

/*
 * This file implements the core of BeeCluster.
 * 
 */

import (
	"sync"
	//"net"
	"fmt"
	"encoding/json"
	"net/http"
	"net/url"
	"log"
	"io/ioutil"
	"math/rand"
	"time"
	"os"
	"path/filepath"
	"strings"
	"strconv"
	//"os"
	//"github.com/draffensperger/golp"
	"sort"
	//"os"
)



// A request has three phases
// (0) a new request, without an associated drone
// (1) a request with an associated drone, but the drone is not ready
// (2) a request with an associated drone an the drone is ready to use
// greater than 2  --> can be freed 

type Request struct{
	requestID		int 

	Loc				[3]float64 
	Sensors			map[string]int
	Duration		float64
	ScheduledTime	float64
	NoLaterThan		float64

	associatedDrone		int // only for phase 1, the associated drone may change

	droneHandlerChannel	chan int // when the phase is 2, the drone's id will appear in this channel. 
	droneHandler     	int 

	acquiredDroneID 	 int // -1 --> nil ... should change this to pointer
	nonInterruptibleFlag bool // when this is true, the system dispatch a drone for handover if needed. 

	phase           	int // 0,1,2 

	ScopeName	 		string 
	CodeBlockName		string 

	sessionID  			int 

	// for scheduler
	IsShadowRequest 	bool
	//IsShadowRequestParentDynamic bool // whether we need to check the parent 
	ShadowRequestParentDroneID int // global pointer
	LocalShadowRequestParent int  
}

type LogEntry struct {
	MetaInfo	string // some marker
	TS 			float64
	Req 		Request
	Sensor      interface{}
	DroneID     int 
	Action 		string 
	Loc 		[3]float64

}

type LogList []LogEntry 

type Frame struct {
	Locs   [][3] float64
}

func (f *Frame) copy() Frame {
	var f2 Frame 

	for i:=0;i<len(f.Locs);i++ {
		f2.Locs = append(f2.Locs, [3]float64{f.Locs[i][0], f.Locs[i][1], f.Locs[i][2]} )
	}
	//fmt.Println(f2)

	return f2
}

func generateFrames(Log []LogEntry, timewindow float64) []Frame{
	var ret []Frame 
	var frame Frame
	var ts_last float64 = -timewindow-1.0

	for _, log := range Log {
		if log.MetaInfo == "droneRequest" {
			ts_now := log.TS 

			if ts_now > ts_last + timewindow {
				if len(frame.Locs) > 0 {

					ret = append(ret, frame.copy())

					ts_last = ts_now 
					frame = Frame{}

					frame.Locs = append(frame.Locs, log.Req.Loc)
				} else {
					frame.Locs = append(frame.Locs, log.Req.Loc)
					ts_last = ts_now 
				}
			} else {
				frame.Locs = append(frame.Locs, log.Req.Loc)
			}
		}
	}

	if len(frame.Locs) > 0 {
		ret = append(ret, frame.copy())
	}

	return ret 
}

// Store context for each application client. 
type applicationContext struct{
	dag 	*DAG 
	name 			string
	AppID 		string 
	SessionID	int 
	isActive 	bool 

	ScopeName2RequestID map[string]int 
	CancelledScopeName map[string]int 
	ScopeName2FirstLocation map[string][3]float64
}


const (
	CoordinateSched 			= "CO"
	RandomSplitPlusTSPSched 	= "RandomTSP"
	MTSPSched				 	= "MTSP"
	BeeclusterSched 			= "Beecluster"
)

const (
	RealDroneBackEnd 			= "RealDrone"
	SimulatorBackEnd			= "simulator"
)


type FrameworkConfig struct {
	Scheduler string `json:"scheduler"` 
	BackendType string `json:"backend"`
	BeeClusterWeights map[string]float64 `json:"SchedulerWeights"`
	SimulatorConfig string `json:"simulatorconfig"`
	DronehubConfig string `json:"dronehubconfig"`

}

func ReadFrameworkConfig(fname string) FrameworkConfig {
	bytes, err := ioutil.ReadFile(fname)
	if err != nil {
		panic(err)
	}
	var cfg FrameworkConfig
	if err := json.Unmarshal(bytes, &cfg); err != nil {
		panic(err)
	}
	return cfg
}


//   python                      go
//   step (1)
//   newRequest    ---->
//   reqID         <----    Create a new request, return the reqID
// 
//   step (2) 
//   waitForDrone  ---->
//   droneHandler  <----    When there is a drone ready for the task, return the drone handler to the python program
//							If the request is cancelled by the python program, return a null drone handler
//
//   step (3)
//   releaseDrone  ---->    Release the drone handler


type Framework struct {
	mutex sync.Mutex

	droneBackend 	DroneBackend
	clock 			Clock 

	// callback from dronebackend
	ReachTargetCallBack	chan int 

	// callback from program
	DroneReleaseCallBack	chan int
	DroneRequestCallBack	chan int

	// Scheduler	
	ReScheduleCallBack		chan int  // this is more like a timer ... 
	TimeTriggerCallBack     chan float64


	Requests		map[int]Request 
	requestNum		int
	deadRequestNum  int 


	drone2request	map[int]int 

	Log 			[]LogEntry

	resetCounter 	int 
	bkCounter		int 

	//ScopeName2RequestID map[string]int // Moved to application context
	//CancelledScopeName map[string]int // Moved to application context

	// some data field for scheduling 

	prediction_request_center [3]float64 
	prediction_request_center_enable bool 

	// application contexts 

	applicationContexts []applicationContext

	//ScopeName2FirstLocation map[string][3]float64 // Moved to application context

	cfg FrameworkConfig

	LogFile string 
	LogPredictionTime float64
	LogScheduleTime float64
	LogStartTS int64
}

func MakeFramework(configfile string, block bool) *Framework {
	f := new(Framework)

	f.ReachTargetCallBack = make(chan int, 1280)
	f.DroneRequestCallBack = make(chan int, 5120) // batch?
	f.DroneReleaseCallBack = make(chan int, 5120)
	f.TimeTriggerCallBack = make(chan float64, 1)
	f.ReScheduleCallBack = make(chan int, 16) 
	//f.ScopeName2RequestID = make(map[string]int)
	//f.ScopeName2FirstLocation = make(map[string][3]float64)
	//f.CancelledScopeName = make(map[string]int)

	f.LogFile = "system_log/log-"+time.Now().Format("2006-01-02-15-04-05")
	f.LogPredictionTime = 0.0 
	f.LogScheduleTime = 0.0
	f.LogStartTS = makeTimestamp()

	f.Requests = make(map[int]Request)
	f.requestNum = 0 

	f.drone2request = make(map[int]int)
	f.resetCounter = 0 
	f.bkCounter = 0
	f.deadRequestNum = 0 


	f.cfg = ReadFrameworkConfig(configfile)

	if f.cfg.BackendType == SimulatorBackEnd {
		sim := MakeSimulator(f.cfg.SimulatorConfig, f.ReachTargetCallBack)
		sim.ReScheduleCallBack = f.ReScheduleCallBack
		f.droneBackend = sim 
		f.clock = sim 
	} else {
		bk := MakeDroneHub(f.cfg.DronehubConfig, f.ReachTargetCallBack)
		bk.ReScheduleCallBack = f.ReScheduleCallBack
		f.droneBackend = bk 
		f.clock = bk

	}
	
	go f.DebugPrint()

	if block {
		f.listenerLoop()
	} else {
		go f.listenerLoop()
	}

	return f
}

func (fra *Framework) AppendLog() {
	s := fmt.Sprintf("%.3f %.6f %.6f\n", float64(makeTimestamp() - fra.LogStartTS)/1000.0, fra.LogScheduleTime, fra.LogPredictionTime)
	f, _ := os.OpenFile(fra.LogFile, os.O_APPEND|os.O_RDWR|os.O_CREATE, 0666)
	f.WriteString(s)
	f.Close()
}

// abort log 

func (fra *Framework) LogReset() {
	fra.mutex.Lock()
	defer fra.mutex.Unlock()

	var entry LogEntry

	entry.MetaInfo = "reset"
	entry.TS = 0.0 

	fra.Log = []LogEntry{}
	fra.Log = append(fra.Log, entry)

}


func (fra *Framework) DumpLog(filename string) {
	fra.mutex.Lock()
	defer fra.mutex.Unlock()

	bytes, _ := json.MarshalIndent(fra.Log, "", "    ")

	err := ioutil.WriteFile(filename, bytes, 0644)

	if err != nil {
		panic(err)
	}
}

func (fra *Framework) LogDroneRequest(body Request) {

	var entry LogEntry

	entry.Req = body 
	entry.TS = fra.droneBackend.GetTime()

	entry.MetaInfo = "droneRequest"

	fra.Log = append(fra.Log, entry)
}


func (fra *Framework) LogDroneAcquire(id int) {

	var entry LogEntry

	entry.DroneID = id 
	entry.TS = fra.droneBackend.GetTime()

	entry.MetaInfo = "droneAcquire"
	
	fra.Log = append(fra.Log, entry)
}


func (fra *Framework) LogDroneRelease(id int) {

	var entry LogEntry

	entry.DroneID = id 
	entry.TS = fra.droneBackend.GetTime()

	entry.MetaInfo = "droneRelease"
	
	fra.Log = append(fra.Log, entry)
}

func (fra *Framework) LogDroneAct(id int,loc [3]float64,  action string, result interface{}) {

	var entry LogEntry

	entry.DroneID = id 
	entry.Sensor = result
	entry.Loc = [3]float64{loc[0], loc[1], loc[2]}
	entry.Action = action
	entry.TS = fra.droneBackend.GetTime()

	entry.MetaInfo = "droneAction"
	
	fra.Log = append(fra.Log, entry)
}

func (fra *Framework) cancelRequest(sessionID int, ScopeName string) {
	fra.mutex.Lock()
	defer fra.mutex.Unlock() 

	fra.applicationContexts[sessionID].CancelledScopeName[ScopeName] = 1

	// cancel an ongoing request 
	if reqID,ok := fra.applicationContexts[sessionID].ScopeName2RequestID[ScopeName]; ok {

		//reqID := fra.ScopeName2RequestID[ScopeName]

		if fra.Requests[reqID].phase <= 1 {
			req := fra.Requests[reqID]
			req.phase = 3
			fra.Requests[reqID]  = req
			//fmt.Println("before channel ", ScopeName, reqID)
			fra.Requests[reqID].droneHandlerChannel <- -1
			//fmt.Println("after channel ", ScopeName, reqID)

		}

		//TODO justReachTargetReqIds

	}


}


func (fra *Framework) newRequest(body Request) int{
	fra.mutex.Lock()
	defer fra.mutex.Unlock() 

	if _,ok := fra.applicationContexts[body.sessionID].CancelledScopeName[body.ScopeName]; ok {
		return -1
	} 


	id := fra.requestNum 

	fra.applicationContexts[body.sessionID].ScopeName2RequestID[body.ScopeName] = id 
	body.requestID = id 
	body.phase = 0 
	body.associatedDrone = -1
	body.droneHandlerChannel = make(chan int, 1)
	body.droneHandler = -1 
	body.ScheduledTime = fra.clock.GetTime()

	fra.Requests[id] = body 

	fra.requestNum += 1

	fra.LogDroneRequest(body)

	fra.clock.SetTriggerCMP(body.ScheduledTime + 1.0, body.ScheduledTime, fra.TimeTriggerCallBack)

	fra.DroneRequestCallBack <- id 

	return id 
}


func (fra *Framework) waitForDrone(reqID int) int{

	fra.mutex.Lock()

	ch := fra.Requests[reqID].droneHandlerChannel

	fra.DebugAddToWaitingList(reqID)

	fra.mutex.Unlock() 

	droneHandler := <- ch

	// if droneHandler!= fra.Requests[reqID].associatedDrone {
	// 	fmt.Println("Error", droneHandler, reqID, fra.Requests[reqID].associatedDrone)
	// 	panic("Error")
	// }

	PrintLog(LogDebug, fmt.Sprintln("get drone ", droneHandler, reqID))


	fra.mutex.Lock()
	defer fra.mutex.Unlock() 

	fra.DebugRemoveFromWaitingList(reqID)

	fra.LogDroneAcquire(droneHandler)

	v := fra.Requests[reqID]
	v.droneHandler = droneHandler
	fra.Requests[reqID] = v

	fra.drone2request[droneHandler] = reqID

	return droneHandler
}
	

func (fra *Framework) releaseDrone(droneID int) {
	fra._releaseDrone(droneID, true)
}

func (fra *Framework) releaseDroneNoLock(droneID int) {
	fra._releaseDrone(droneID, false)
}

func (fra *Framework) _releaseDrone(droneID int, lock bool) {
	if lock {
		fra.mutex.Lock()
		defer fra.mutex.Unlock()
	}



	reqID := fra.drone2request[droneID]

	v := fra.Requests[reqID]
	v.phase = 3
	fra.Requests[reqID]=v // waiting for GC

	fra.LogDroneRelease(droneID)

	PrintLog(LogDebug, fmt.Sprintln("release drone", droneID, reqID))
	fra.droneBackend.Clear(droneID)



	fra.DroneReleaseCallBack <- droneID
}


// a FIFO implementation
func (fra *Framework) ScheduleFIFO() {
	//fra.mutex.Lock()
	//defer fra.mutex.Unlock()

	// add lock
	// find a free drone
	var free_drone_id int = -1 
	drones := fra.droneBackend.GetDrones()
	for i:=0; i<len(drones); i++ {
		if len(drones[i].TargetLocation) == 0 {
			free_drone_id = i
			break
		}
	}


	if free_drone_id != -1 {
		for i:=0; i<fra.requestNum; i++ {
			if fra.Requests[i].phase == 0 {
				//fmt.Println("schedule drone", free_drone_id, " to request",i)

				v:=fra.Requests[i]
				v.associatedDrone = free_drone_id
				v.phase = 1
				fra.Requests[i] = v
				
				fra.drone2request[free_drone_id] = i 

				fra.droneBackend.SetTarget(free_drone_id, fra.Requests[i].Loc)

				break
			}
		}
	}
}

// defined in scheduler.go 
type DroneAssignment struct {
	DroneId      int 
	RequestId    int
	Cost         float64 
}

type AssignmentList struct {
	ass  []DroneAssignment
}

func (a *AssignmentList) Len() int {
	return len(a.ass)
}

func (a *AssignmentList) Swap(i,j int) {

	a.ass[i], a.ass[j] = a.ass[j], a.ass[i]

}

func (a *AssignmentList) Less(i,j int) bool {
	return a.ass[i].Cost < a.ass[j].Cost 
}


// The code below requires a lp solver library that may not be easy to install,
// so we just remove it. 

// Use IP for existing requests. (minimize sum of the waiting time)
// Then, if there are sparse drone, schedule them to the predicted locations 

/*func (fra *Framework) ScheduleIP() {
	
	
	drones := fra.droneBackend.GetDrones()
	var assignments []DroneAssignment


	justReachTargetReqIds := make(map[int]int)


	var free_drone int = 0
	var open_request int = 0 
	var min_assignments int = 0 

	for i:=0; i<len(drones); i++ {
		fmt.Println("drone ", i, "phase",drones[i].Phase )
		if drones[i].Phase == 2 {
			justReachTargetReqIds[fra.drone2request[i]] = 1
		} else {
			free_drone += 1
		}
	}

	for j:=0; j<fra.requestNum;j++ { // todo: optimize this for-loop...
		if _,ok := justReachTargetReqIds[j]; !ok && (fra.Requests[j].phase < 2) {
			open_request += 1 			
		}
	}

	if free_drone < open_request {
		min_assignments = free_drone
	} else {
		min_assignments = open_request
	}

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



	// no batching but use re-planning
	for i:=0; i<len(drones); i++ {
		if drones[i].Phase != 2 {
			free_drone += 1 
			for j:=0; j<fra.requestNum;j++ { // todo: optimize this for-loop...
				if _,ok := justReachTargetReqIds[j]; !ok && (fra.Requests[j].phase < 2) {
					var ass DroneAssignment 

					ass.DroneId = i 
					ass.RequestId = j 
					var ft = fra.droneBackend.FlyingTimeEst(i, drones[i].Loc, fra.Requests[j].Loc) + 1.0
					//var wt = fra.droneBackend.GetTime() + ft - fra.Requests[j].ScheduledTime

					//ass.Cost = wt / (ft*ft)
					//ass.Cost = wt*wt / ft 

					ass.Cost = ft

					// var age = wt - ft 

					// if ft < 30 {
					// 	ass.Cost = ft * 1.0/(age+1.0)
					// } else {
					// 	ass.Cost = ft 
					// }
					
					
					var future_aware = 1.0 
					var center_aware = 1.0
					// if fra.prediction_request_center_enable {
					// 	future_aware = 1.0/(distance(fra.prediction_request_center, fra.Requests[j].Loc) + 1.0) + 0.1
					
					// 	//fmt.Println(i, future_aware)
					// }

					// if counter > 0.5 {
					// 	center_aware = 1.0/(distance(center, fra.Requests[j].Loc) + 1.0) + 0.1
					// 	center_aware = 1.0/center_aware
					// }



					ass.Cost = ass.Cost * future_aware * center_aware 
					//ass.Cost = 1.0/center_aware * future_aware


					assignments = append(assignments, ass)
				}
			}
		}
	}

	var num int = len(assignments)

	if num > 0 {

		
		task_map := make(map[int][]golp.Entry)
		drone_map := make(map[int][]golp.Entry)
		for ind, a := range assignments {
			task_map[a.RequestId] = append(task_map[a.RequestId], golp.Entry{ind,0.9})
			drone_map[a.DroneId] = append(drone_map[a.DroneId], golp.Entry{ind,0.9})
		}

		lp := golp.NewLP(0, num)

		// one task one drone
		for _, v := range task_map {
			lp.AddConstraintSparse(v, golp.LE, 1.0)
		}

		for _, v := range drone_map {
			lp.AddConstraintSparse(v, golp.LE, 1.0)
		}

		// min assignments 
		assignments_map := []golp.Entry{}
		for i:=0; i<num; i++ {
			assignments_map = append(assignments_map, golp.Entry{i, 1.0})
		}

		lp.AddConstraintSparse(assignments_map, golp.GE, float64(min_assignments)-0.5)


		var obj []float64 

		for i:=0; i<num; i++ {
			obj = append(obj, assignments[i].Cost)
			lp.SetInt(i, true)
		}

		lp.SetObjFn(obj)
		lp.Solve()
	
		fmt.Printf("Objective value: %v\n", lp.Objective())
	  	
		// update assignment 
		vars := lp.Variables()

		for i:=0; i< lp.NumCols(); i++ {
			if vars[i] > 0.5 {
				rid := assignments[i].RequestId
				droneid := assignments[i].DroneId
				cost := assignments[i].Cost 

				

				v:=fra.Requests[rid]

				if v.associatedDrone != -1 && v.associatedDrone != droneid {
					// don't assign two drones to the same request 
					// cancel the other drone 
					other_drone_id := v.associatedDrone

					if drones[other_drone_id].Phase == 2 {
						// should ignore new assignment 
						fmt.Printf("[scheduler] ignore duplicate assignment %d on %d\n", droneid, rid)
						continue 
					}

					// only if the other drone is also flying to the same target
					if fra.drone2request[other_drone_id] == rid {
						fra.droneBackend.UnsetTarget(other_drone_id)
						fmt.Printf("[scheduler] unassign drone %d to %d\n", other_drone_id, rid)
					}
				}

				fmt.Printf("[scheduler] assign drone %d to %d, cost %.2f\n", droneid, rid, cost)

				v.associatedDrone = droneid
				v.phase = 1
				fra.Requests[rid] = v
				
				fra.drone2request[droneid] = rid 

				fra.droneBackend.SetTarget(droneid, fra.Requests[rid].Loc)
				fra.droneBackend.SetSpeculationTarget(droneid, [][3]float64{})
			}
		}
	} 
}
*/

// Greedy Scheduler 
func (fra *Framework) ScheduleGreedy() {
	
	drones := fra.droneBackend.GetDrones()
	var assignments []DroneAssignment


	justReachTargetReqIds := make(map[int]int)


	var free_drone int = 0
	var open_request int = 0 
	//var min_assignments int = 0 

	for i:=0; i<len(drones); i++ {
		PrintLog(LogDebug, fmt.Sprintln("drone ", i, "phase",drones[i].Phase ))
		if drones[i].Phase == 2 {
			justReachTargetReqIds[fra.drone2request[i]] = 1
		} else {
			if drones[i].IsDroneOnline {
				free_drone += 1
			}
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



	// no batching but use re-planning
	for i:=0; i<len(drones); i++ {
		if drones[i].IsDroneOnline == false {
			continue
		}
		
		if drones[i].Phase != 2 {
			free_drone += 1 
			for j:=0; j<fra.requestNum;j++ { // todo: optimize this for-loop...
				if _,ok := justReachTargetReqIds[j]; !ok && (fra.Requests[j].phase < 2) {
					var ass DroneAssignment 

					ass.DroneId = i 
					ass.RequestId = j 
					var ft = fra.droneBackend.FlyingTimeEst(i, drones[i].Loc, fra.Requests[j].Loc) + 1.0

					ass.Cost = ft
					
					var future_aware = 1.0 
					var center_aware = 1.0

					ass.Cost = ass.Cost * future_aware * center_aware 
					//ass.Cost = 1.0/center_aware * future_aware


					assignments = append(assignments, ass)
				}
			}
		}
	}

	var num int = len(assignments)

	if num > 0 {
		asslist := AssignmentList{assignments}
		sort.Sort(&asslist)

		task_map := make(map[int]bool)
		drone_map := make(map[int]bool)


		var result_assignments []DroneAssignment

		for _, ass := range asslist.ass {
			rid := ass.RequestId
			did := ass.DroneId 

			if _,ok:= task_map[rid]; ok {
				continue
			}

			if _,ok := drone_map[did]; ok {
				continue
			}


			task_map[rid] = true
			drone_map[did] = true 


			result_assignments = append(result_assignments, ass)


		}

		//for i:=0; i< lp.NumCols(); i++ {
		for _,ass:= range result_assignments {
			
				rid := ass.RequestId
				droneid := ass.DroneId
				//cost := ass.Cost 

				

				v:=fra.Requests[rid]

				if v.associatedDrone != -1 && v.associatedDrone != droneid {
					// don't assign two drones to the same request 
					// cancel the other drone 
					other_drone_id := v.associatedDrone

					if drones[other_drone_id].Phase == 2 {
						// should ignore new assignment 
						//fmt.Printf("[scheduler] ignore duplicate assignment %d on %d\n", droneid, rid)
						continue 
					}

					// only if the other drone is also flying to the same target
					if fra.drone2request[other_drone_id] == rid {
						fra.droneBackend.UnsetTarget(other_drone_id)
						//fmt.Printf("[scheduler] unassign drone %d to %d\n", other_drone_id, rid)
					}
				}

				//fmt.Printf("[scheduler] assign drone %d to %d, cost %.2f\n", droneid, rid, cost)

				v.associatedDrone = droneid
				v.phase = 1
				fra.Requests[rid] = v
				
				fra.drone2request[droneid] = rid 

				fra.droneBackend.SetTarget(droneid, fra.Requests[rid].Loc)
				fra.droneBackend.SetSpeculationTarget(droneid, [][3]float64{})
			
		}
	} 
}



// call this after Schedule IP , if there are free drones, send them to speculation targets.
/*
func (fra *Framework) ScheduleSpeculation() {
	//return 

	// don't use this now 
	return 


	fra.droneBackend.(*Simulator).Pause()
	defer fra.droneBackend.(*Simulator).Resume()

	fra.mutex.Lock()
	defer fra.mutex.Unlock()

	

	frames := generateFrames(fra.Log, 2.0)
	//fmt.Println("frames", len(frames), frames[len(frames)-1])

	if len(frames) < 3 {
		// not enough frames to do prediction 
		return 
	} else {


		drones := fra.droneBackend.GetDrones()

		
		var free_drone int = 0
		var open_request int = 0 
		var min_assignments int = 0 

		free_drone_marker := make(map[int]int)

		for i:=0; i<len(drones); i++ {
			if drones[i].Phase == 0 {
				free_drone += 1
				free_drone_marker[i] = 1
			}
		}

		

		addr := "127.0.0.1:8010"
		conn, err := net.Dial("tcp", addr)

		//defer conn.Close()

		if err != nil {
			log.Fatalln(err)
		}

		bytes, _ := json.Marshal(frames[len(frames)-3:len(frames)])

		conn.Write(bytes)

		buff := make([]byte, 8192)
		n, _ := conn.Read(buff)

		var predictions [][][3]float64
		if err := json.Unmarshal( buff[:n], &predictions); err != nil {
			//fmt.Println(string(buf))
			panic(err)
		}

		conn.Write(bytes[0:3])

		// update this
		fra.prediction_request_center = [3]float64{0.0, 0.0, 0.0}

		var cc int = 0;

		for _, locs := range predictions {
			for _, loc := range locs {
				cc += 1 

				fra.prediction_request_center[0] += loc[0]
				fra.prediction_request_center[1] += loc[1]
				fra.prediction_request_center[2] += loc[2]
			}
		}

		if cc != 0 {
			fra.prediction_request_center[0] /= float64(cc)
			fra.prediction_request_center[1] /= float64(cc)
			fra.prediction_request_center[2] /= float64(cc)


			fra.prediction_request_center_enable = true
		} else {
			fra.prediction_request_center_enable = false
		}


		if free_drone == 0 {
			return 
		}



		for _, locs := range predictions {
			if len(locs) == 0 {
				continue 
			}

			prediction_locs:=locs 

			open_request = len(prediction_locs)

			log.Println("[speculation] Number of Predicted Requests:", open_request)

			var assignments []DroneAssignment

			free_drone = 0

			for _,v := range free_drone_marker {
				if v == 1 {
					free_drone += 1
				}
			}

			if free_drone == 0 {
				break
			}



			// generate assignments 
			for i:=0; i<len(drones); i++ { 
				if drones[i].Phase == 0 {  // Virtual Assignment 
					if free_drone_marker[i] == 1{
						for j :=0; j<open_request; j ++ {
							// if j>free_drone {
							// 	break
							// }

							var ass DroneAssignment 

							ass.DroneId = i
							ass.RequestId = j 
							var ft = fra.droneBackend.FlyingTimeEst(i, drones[i].Loc, prediction_locs[j]) + 1.0
							var wt = ft

							//ass.Cost = wt / (ft*ft)
							//ass.Cost = wt*wt / ft 
							ass.Cost = wt*wt
							
							assignments = append(assignments, ass)

						}
					}
				}
			}

			var num int = len(assignments)
			min_assignments = free_drone
			if open_request < min_assignments  {
				min_assignments = open_request
			}


			if num > 0 {

				task_map := make(map[int][]golp.Entry)
				drone_map := make(map[int][]golp.Entry)
				for ind, a := range assignments {
					task_map[a.RequestId] = append(task_map[a.RequestId], golp.Entry{ind,0.9})
					drone_map[a.DroneId] = append(drone_map[a.DroneId], golp.Entry{ind,0.9})
				}

				lp := golp.NewLP(0, num)

				// one task one drone
				for _, v := range task_map {
					lp.AddConstraintSparse(v, golp.LE, 1.0)
				}

				for _, v := range drone_map {
					lp.AddConstraintSparse(v, golp.LE, 1.0)
				}

				// min assignments 
				assignments_map := []golp.Entry{}
				for i:=0; i<num; i++ {
					assignments_map = append(assignments_map, golp.Entry{i, 1.0})
				}

				lp.AddConstraintSparse(assignments_map, golp.GE, float64(min_assignments)-0.5)


				var obj []float64 

				for i:=0; i<num; i++ {
					obj = append(obj, assignments[i].Cost)
					lp.SetInt(i, true)
				}

				lp.SetObjFn(obj)

				fmt.Println("[speculation]", num)
				lp.Solve()
			
				//fmt.Printf("[speculation] Objective value: %v\n", lp.Objective())
			  	
				// update assignment 
				vars := lp.Variables()

				for i:=0; i< lp.NumCols(); i++ {
					if vars[i] > 0.5 {
						rid := assignments[i].RequestId
						droneid := assignments[i].DroneId
						free_drone_marker[droneid] = 0
						cost := assignments[i].Cost 

						if drones[droneid].Phase != 0 {
							continue
						}

						fmt.Printf("[speculation] assign drone %d to %d, cost %.2f\n", droneid, rid, cost)

						fra.droneBackend.SetSpeculationTarget(droneid, [][3]float64{prediction_locs[rid]})
					}
				}
			}
		}

	}	
}
*/


// This is the real schedule entry
func (fra *Framework) Schedule() {
	// The simulator may run super fast... so pause it 

	if fra.cfg.BackendType == SimulatorBackEnd {
		fra.droneBackend.(*Simulator).Pause()
		defer fra.droneBackend.(*Simulator).Resume()
	}


	fra.mutex.Lock()
	defer fra.mutex.Unlock()

	t0 := makeTimestampMicro()

	if fra.cfg.Scheduler == BeeclusterSched {
		
		fra.ScheduleBeeCluster()
	} else if fra.cfg.Scheduler == CoordinateSched {
		fra.ScheduleGreedy()
	}


	t1 := makeTimestampMicro()

	fra.LogScheduleTime += float64(t1-t0)/1000000.0
	fra.AppendLog()

	//fra.ScheduleFIFO()
	//fra.ScheduleIP()
	//fra.ScheduleGreedy()
	

	//fra.ScheduleSpeculation()
}


// the main message listener loop of beecluster framework
func (fra *Framework) listenerLoop() {
	go fra.frameworkListener()
	go fra.droneListener()
	go fra.httpListener()

	for {

		type_of_message := -1 

		var msg int 
		var msg_ts float64

		select {
		case msg = <- fra.DroneRequestCallBack:
			type_of_message = 1 

		case msg = <- fra.DroneReleaseCallBack:
			type_of_message = 2

		case msg = <- fra.ReScheduleCallBack:
			type_of_message = 3

		case msg = <- fra.ReachTargetCallBack:
			type_of_message = 4

		case msg_ts = <- fra.TimeTriggerCallBack:
			type_of_message = 5

		}

		if type_of_message == 1 {

			// maybe reschedule?
			fra.Schedule()

		} else if type_of_message == 2 {

			fra.Schedule()
			//fra.ScheduleSpeculation()

	    } else if type_of_message == 3 {

	    	fra.Schedule()

		} else if type_of_message == 4 {

			//fmt.Println("drone reach target", msg, fra.drone2request[msg])

			fra.mutex.Lock()
			reqID := fra.drone2request[msg]
			PrintLog(LogDebug, fmt.Sprintln("req", reqID,fra.Requests[reqID].phase ))
			if fra.Requests[reqID].phase <= 1 {
				//fmt.Println("before msg",reqID)
				fra.Requests[reqID].droneHandlerChannel <- msg 
				//fmt.Println("after msg",reqID)
				v := fra.Requests[reqID]
				v.phase = 2 

				fra.Requests[reqID] = v 
			} else {
				
				fra.releaseDroneNoLock(msg)
			}
			fra.mutex.Unlock() 

		} else if type_of_message == 5 {
			// speculative 
			PrintLog(LogDebug, fmt.Sprintln("timer trigger todo speculation ", msg_ts))
			//fra.ScheduleSpeculation()

		}
	}
}

// control endpoint 
func (fra *Framework) frameworkListener() {

	frameworkListener := http.NewServeMux()
	frameworkListener.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		bytes, _ := ioutil.ReadAll(r.Body)
		defer r.Body.Close()

		var cmd frameworkCMD 
		if err := json.Unmarshal(bytes, &cmd); err != nil {
			//fmt.Println(string(buf))
			panic(err)
		}
		if cmd.CMD == "newSession" {
			// input: AppID
			// output: a local session id 
			// action: - create application context 
			//         - load historical dags 

			fra.mutex.Lock()

			var sessionID int = len(fra.applicationContexts)
			PrintLog(LogInfo, fmt.Sprintf("New Session Created. Session ID = %d, APP ID = %s .\n", sessionID, cmd.AppID))

			var appContext applicationContext
			

			if _, err := os.Stat("data"); os.IsNotExist(err) {
				os.Mkdir("data",0755)
			}

			if _, err := os.Stat("data/"+cmd.AppID); os.IsNotExist(err) {
				os.Mkdir("data/"+cmd.AppID,0755)
			}

			//var files []string 
			var AppDags []*DAG 

			err := filepath.Walk("data/"+cmd.AppID, func(path string, info os.FileInfo, err error) error {
        		if strings.HasSuffix(path, ".gob") {
        			AppDags = append(AppDags, LoadDag(path)[0])
        			PrintLog(LogInfo, fmt.Sprintln("Load Historical DTG ", path))
        		}

        		return nil
   			})

   			if err != nil {
   				panic(err)
   			}

			appContext.dag = MakeDAG(AppDags)
			appContext.AppID = cmd.AppID
			appContext.SessionID = sessionID

			appContext.ScopeName2RequestID = make(map[string]int)
			appContext.CancelledScopeName = make(map[string]int)
			appContext.ScopeName2FirstLocation = make(map[string][3]float64)

			appContext.isActive = true 

			fra.applicationContexts = append(fra.applicationContexts, appContext)


			fra.mutex.Unlock()

			

			retBytes, _ := json.Marshal(sessionID)

			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)

		}

		if cmd.CMD == "stopSession" {
			// input: AppID and session id 
			fra.applicationContexts[cmd.SessionID].isActive = false
			PrintLog(LogInfo, fmt.Sprintf("Session Closed. Session ID = %d, APP ID = %s .\n", cmd.SessionID, cmd.AppID))
			
			retBytes, _ := json.Marshal(0)
			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)
		}

		if cmd.CMD == "stopSessionAndSaveDTG" {
			// input: AppID and session id 
			// action: store dag 
			fra.mutex.Lock()

			fra.applicationContexts[cmd.SessionID].isActive = false

			path := "data/" + cmd.AppID + "/" + makeTimestampString() + ".gob"
			fra.applicationContexts[cmd.SessionID].dag.SaveDag(path)

			fra.mutex.Unlock()
			PrintLog(LogInfo, fmt.Sprintln("[INFO] Save DTG to ", path))
			PrintLog(LogInfo, fmt.Sprintf("Session Closed. Session ID = %d, APP ID = %s .\n", cmd.SessionID, cmd.AppID))
			
			retBytes, _ := json.Marshal(0)
			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)
		}


		if cmd.CMD == "newRequest" {
			var req Request 

			req.Loc = [3]float64{cmd.X, cmd.Y, cmd.Z}
			req.ScopeName = cmd.ScopeName 
			req.CodeBlockName = cmd.NodeName
			req.sessionID = cmd.SessionID
			req.acquiredDroneID = cmd.AcquireDroneID
			req.nonInterruptibleFlag = cmd.NonInterruptibleFlag

			//fmt.Println("bytes:", string(bytes),req.acquiredDroneID,cmd.AcquireDroneID, cmd.NonInterruptibleFlag)
			//fmt.Println("req acquiredDroneID", req.acquiredDroneID )

			fra.mutex.Lock()

			if _, ok:= fra.applicationContexts[req.sessionID].ScopeName2FirstLocation[req.ScopeName]; ok == false {
				fra.applicationContexts[req.sessionID].ScopeName2FirstLocation[req.ScopeName] = [3]float64{cmd.X, cmd.Y, cmd.Z} 
				fra.applicationContexts[req.sessionID].dag.AddDeferredLocation(req.ScopeName, [3]float64{cmd.X, cmd.Y, cmd.Z})
			}
			//fmt.Println("codeblock", cmd.NodeName)
			// todo update code block's first location if applied
			if _, ok:= fra.applicationContexts[req.sessionID].ScopeName2FirstLocation[cmd.NodeName]; ok == false {
				fra.applicationContexts[req.sessionID].ScopeName2FirstLocation[cmd.NodeName] = [3]float64{cmd.X, cmd.Y, cmd.Z} 
				fra.applicationContexts[req.sessionID].dag.AddDeferredLocation(cmd.NodeName, [3]float64{cmd.X, cmd.Y, cmd.Z})
			}


			fra.mutex.Unlock()
			
			reqID := fra.newRequest(req)
			retBytes, _ := json.Marshal(reqID)

			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)

		} else if cmd.CMD == "cancelRequest" {

			fra.cancelRequest(cmd.SessionID, cmd.ScopeName)

			retBytes, _ := json.Marshal("ack")

			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)
		

		} else if cmd.CMD == "getDrone" {
			droneID := fra.waitForDrone(cmd.ID)

			retBytes, _ := json.Marshal(droneID)
			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)


		} else if cmd.CMD == "releaseDrone" {

			fra.releaseDrone(cmd.ID)

			retBytes, _ := json.Marshal("ack")
			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)

		} else if cmd.CMD == "addDependency" {
			//fmt.Println("add addDependency", cmd )
			if fra.cfg.BackendType == SimulatorBackEnd {
				fra.droneBackend.(*Simulator).Pause()
			}

			fra.applicationContexts[cmd.SessionID].dag.AddDependency(cmd.NodeName, cmd.Dependencies, cmd.ScopeName, cmd.MetaInfo)
			
			if fra.cfg.BackendType == SimulatorBackEnd {
				fra.droneBackend.(*Simulator).Resume()
			}
			
			retBytes, _ := json.Marshal("ack")
			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)

		} else if cmd.CMD == "logAction" {
			// TODO log the action in the dag (for prediction)

			retBytes, _ := json.Marshal("ack")
			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)
		}
			
	})

	u, _ := url.Parse("http://127.0.0.1:8008")
	log.Fatal(http.ListenAndServe(u.Host, frameworkListener))
}

// python client <--> drone endpoint 
func (fra *Framework) droneListener() {

	droneListener := http.NewServeMux()
	droneListener.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {

		// update drone state in drone hub
		// send new update to physical drone 

		bytes, _ := ioutil.ReadAll(r.Body)
		defer r.Body.Close()

		var cmd droneCMD 
		if err := json.Unmarshal(bytes, &cmd); err != nil {
			//fmt.Println(string(buf))
			panic(err)
		}


		if cmd.CMD == "flyto" {
			drone_id := cmd.ID
			loc := [][3]float64{{cmd.X, cmd.Y, cmd.Z}}

			c := fra.droneBackend.SetWayPath(drone_id, loc)

			// wait? 
			_ = <- c

			retBytes, _ := json.Marshal("ack")
			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)


		} else {
			drone_id := cmd.ID

			tt1 := makeTimestamp()

			ret := fra.droneBackend.Sense(drone_id, cmd.CMD)

			tt2 := makeTimestamp()

			//fmt.Println("Debug sensing time:", tt2-tt1)
			fra.droneBackend.LogSenseTime(float64(tt2-tt1)/1000.0)

			drones := fra.droneBackend.GetDrones()

			fra.LogDroneAct(drone_id, drones[drone_id].Loc, cmd.CMD, ret)


			retBytes, _ := json.Marshal(ret)

			w.Header().Set("Content-Type", "application/json")
			w.Write(retBytes)
			
		}

		})

	u, _ := url.Parse("http://127.0.0.1:8009")
	log.Fatal(http.ListenAndServe(u.Host, droneListener))

}






type visDrone struct {
	X	float64 `json"x"`
	Y	float64 `json"y"`
	Z	float64 `json"z"`
	ID  int     `json"id"`
	Phase int   `json"phas"`

}

// web endpoint 
func (fra *Framework) httpListener() {
	fileServer := http.FileServer(http.Dir("os/web/"))
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		//if r.URL.Path == "/" {
			w.Header().Set("Cache-Control", "no-cache")

		//}
		fileServer.ServeHTTP(w, r)
	})

	// http.HandleFunc("/bk.png", func(w http.ResponseWriter, r *http.Request) {
	// 	//fmt.Println("load bk ")

	// 	w.Header().Set("Cache-Control", "no-store")
			
	// 	fileServer.ServeHTTP(w, r)
	// })


	http.HandleFunc("/drones", func(w http.ResponseWriter, r *http.Request) {

		_drones := fra.droneBackend.GetDrones()

		var drones []visDrone

		for i:=0; i<len(_drones); i++ {

			drones = append(drones, visDrone{_drones[i].Loc[0],_drones[i].Loc[1],_drones[i].Loc[2], i, _drones[i].Phase})

		}

		bytes, err := json.Marshal(drones)
		if err != nil {
			fmt.Println(drones)
			panic(err)
		}
		w.Header().Set("Content-Type", "application/json")
		w.Write(bytes)
	})


	http.HandleFunc("/reset", func(w http.ResponseWriter, r *http.Request) {
		
		fra.resetCounter += 1 

		if fra.cfg.BackendType == SimulatorBackEnd {
			//fmt.Println("run ", fra.resetCounter, "time: ", fra.droneBackend.(*Simulator).LastActionTS - fra.droneBackend.(*Simulator).FirstActionTS)
		}

		req := r.URL.Query()

		if seed, ok := req["seed"]; ok {
			s,_ := strconv.Atoi(seed[0])
			rand.Seed(int64(s))
		}

		if fra.cfg.BackendType == SimulatorBackEnd {
			//fmt.Println("t1")
			fra.droneBackend.(*Simulator).Reset()
			//fmt.Println("t2")
		}

		fra.mutex.Lock()
		//fra.ScopeName2RequestID = make(map[string]int)
		//fra.ScopeName2FirstLocation = make(map[string][3]float64)
		//fra.CancelledScopeName = make(map[string]int)

		fra.cfg = ReadFrameworkConfig("os/configs/framework/framework_config.json")


		//if fra.applicationContexts[0].dag.nodecounter > 0 {
		//	fra.applicationContexts[0].dag.SaveDag("daglog")
		//}

		//dags := LoadDag("daglog")
		fra.applicationContexts = []applicationContext{}



		fra.mutex.Unlock()


		fra.DumpLog(fmt.Sprintf("log/log%d.json", fra.resetCounter))
		fra.LogReset()

		//fmt.Println("t3")

		// w.Header().Set("Content-Type", "application/json")
		// w.Write(([]byte)"ack")

		bytes, err := json.Marshal("ack")
		if err != nil {
			panic(err)
		}
		w.Header().Set("Content-Type", "application/json")
		w.Write(bytes)

	})

	http.HandleFunc("/control", func(w http.ResponseWriter, r *http.Request) {
		req := r.URL.Query()
		if cmd, ok := req["cmd"]; ok {
			if cmd[0] == "storedag" {
				PrintLog(LogInfo, fmt.Sprintln("Store dag to tmpdaglog"))
				fra.applicationContexts[0].dag.SaveDag("tmpdaglog")
			}

			if cmd[0] == "loaddag" {
				PrintLog(LogInfo, fmt.Sprintln("Load dag from tmpdaglog"))
				hdag := LoadDag("tmpdaglog")

				fra.applicationContexts[0].dag.historyDags = append(fra.applicationContexts[0].dag.historyDags, hdag[0])
				PrintLog(LogInfo, fmt.Sprintln("number of historical dags", len(fra.applicationContexts[0].dag.historyDags)))
			}

		}

		bytes, err := json.Marshal("ack")
		if err != nil {
			panic(err)
		}
		w.Header().Set("Content-Type", "application/json")
		w.Write(bytes)

	})

	http.HandleFunc("/debug", func(w http.ResponseWriter, r *http.Request) {
		req := r.URL.Query()
		if cmd, ok := req["cmd"]; ok {
			if cmd[0] == "storedag" {
				PrintLog(LogDebug, fmt.Sprintln("Store dag to tmpdaglog"))
				fra.applicationContexts[0].dag.SaveDag("tmpdaglog")
			}


			if cmd[0] == "dagreplay" {
				filename := req["file"][0]
				dag := LoadDag(filename)[0]
				dag.debuglog = true
				PrintLog(LogDebug, fmt.Sprintln("load dag", filename))
				for k,n := range dag.Name2Node {
					PrintLog(LogDebug, fmt.Sprintln(n.Ordering, k))
				}


				nodename := req["nodename"][0]

				if _,ok := dag.Name2Node[nodename]; ok {
					_ = dag.match(dag.Name2Node[nodename], []*DAG{dag})
				} else {
					PrintLog(LogDebug, fmt.Sprintln("node not found", nodename))
				}

			}

			if cmd[0] == "set_quiet_log" {
				quiet_log = !quiet_log
				PrintLog(LogDebug, fmt.Sprintln("Set quiet log to", quiet_log))
			}

			if cmd[0] == "set_vis_log" {
				vis_log = !vis_log
				PrintLog(LogDebug, fmt.Sprintln("Set vis log to", vis_log))
			}

			if cmd[0] == "set_stepping_mode" {
				stepping_mode = !stepping_mode
				PrintLog(LogDebug, fmt.Sprintln("Set stepping mode to", stepping_mode))
			}

			if cmd[0] == "step" {
				//stepping_count += 1
				var inc int = 1
				if d, ok := req["value"]; ok {
					inc,_ = strconv.Atoi(d[0])
				}

				increaseSteppingCounter(inc)
			}
			

		}

		bytes, err := json.Marshal("ack\n")
		if err != nil {
			panic(err)
		}
		w.Header().Set("Content-Type", "application/json")
		w.Write(bytes)


	})



	http.HandleFunc("/updatebk", func(w http.ResponseWriter, r *http.Request) {
		fra.bkCounter += 1
		if fra.cfg.BackendType == SimulatorBackEnd {
			fra.droneBackend.(*Simulator).UpdateBackground(fmt.Sprintf("os/web/bk.png"))
		}

		//fmt.Println("deadlock?")
		// if fra.bkCounter != 1 {
		// 	os.Remove(fmt.Sprintf("os/web/bk%d.png", fra.bkCounter-1))
		// }

		bytes, err := json.Marshal(fra.bkCounter)
		if err != nil {
			panic(err)
		}
		w.Header().Set("Content-Type", "application/json")
		w.Write(bytes)

	})


	u, err := url.Parse("http://localhost:8007")
	if err != nil {
		panic(err)
	}
	PrintLog(LogInfo, fmt.Sprintf("[Scheduler web] listening on %s", u.Host))
	//log.Printf("[Scheduler web] listening on %s", u.Host)
	log.Fatal(http.ListenAndServe(u.Host, nil))

}