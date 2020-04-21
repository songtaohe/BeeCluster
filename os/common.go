package main 

/*
 * This file implements common structures, interfaces, and functions. 
 * 
 */

import (
	"math"
	"time"
	"fmt"
	"sort"
	"sync"
	"github.com/oddg/hungarian-algorithm"
)

var quiet_log bool = true
var vis_log bool = false
var framework_log bool = false 
var stepping_mode bool = false
var stepping_count int = 0 
var stepping_mutex sync.Mutex


func waitStepSignal(tag string) {
	if stepping_mode == false {
		return 
	} else {
		fmt.Println("Hit BP -->", tag)
		for {
			time.Sleep(time.Duration(10000) * time.Microsecond) // 10ms
			var ret bool = false

			stepping_mutex.Lock()

			if stepping_count > 0 {
				stepping_count -= 1
				ret = true
			}

			stepping_mutex.Unlock()

			if ret {
				break
			}
		}
		fmt.Println("Move over BP -->", tag)
	}
}

func increaseSteppingCounter(d int){
	stepping_mutex.Lock()

	stepping_count += d 

	stepping_mutex.Unlock()
}


func makeTimestampString() string {
    return time.Now().Format("20060102150405")
}

func makeTimestamp() int64 {
    return time.Now().UnixNano() / int64(time.Millisecond)
}

func makeTimestampMicro() int64 {
    return time.Now().UnixNano() / int64(time.Microsecond)
}

func printtime(told int64, str string) {
	if quiet_log == false {
		fmt.Println("Time elasped",str, makeTimestamp() - told)
	}
}



type DistanceFunc func([3]float64, [3]float64) float64 


func distance(p1 [3]float64, p2 [3]float64) float64 {
	a := p1[0] - p2[0]
	b := p1[1] - p2[1]
	c := (p1[2] - p2[2])/3.0 // altitude inaccurate

	return math.Sqrt(a*a + b*b + c*c)
}

func distanceRaw(p1 [3]float64, p2 [3]float64) float64 {
	a := p1[0] - p2[0]
	b := p1[1] - p2[1]
	c := p1[2] - p2[2]

	return math.Sqrt(a*a + b*b + c*c)
}

func distance2(p1 [2]float64, p2 [2]float64) float64 {
	a := p1[0] - p2[0]
	b := p1[1] - p2[1]

	return math.Sqrt(a*a + b*b)
}


func segmentDistance(p11 [3]float64, p12 [3]float64, p21 [3]float64, p22 [3]float64) float64 {
	
	d := distance(p11,p12)

	inc := 0.2/d 

	alpha := 0.0 

	dmin := 10000000.0

	for {

		x := p11[0] * alpha + (1.0-alpha) * p12[0]
		y := p11[1] * alpha + (1.0-alpha) * p12[1]
		z := p11[2] * alpha + (1.0-alpha) * p12[2]

		_d := point2segmentDistance(p21, p22, [3]float64{x,y,z})
		
		if dmin > _d {
			dmin = _d 
		}

		alpha += inc
		if alpha > 1.0 {
			break
		}
	}

	//fmt.Println("segmentDistance", p11,p12, p21,p22, "distance", dmin)

	return dmin 


}


func point2segmentDistance(p11 [3]float64, p12 [3]float64, p2 [3]float64) float64 {

	d := distance(p11,p12)

	inc := 0.5/d 

	alpha := 0.0 

	dmin := 10000000.0

	for {

		x := p11[0] * alpha + (1.0-alpha) * p12[0]
		y := p11[1] * alpha + (1.0-alpha) * p12[1]
		z := p11[2] * alpha + (1.0-alpha) * p12[2]

		_d := distance([3]float64{x,y,z}, p2)
		
		if dmin > _d {
			dmin = _d 
		}

		alpha += inc
		if alpha > 1.0 {
			break
		}
	}


	return dmin 
}

func findClosestIndex(loc [3]float64, locs [][3]float64, dist_func DistanceFunc) int {

	var minimal_dist float64 = 1000000000.0 
	var ret_ind int  = -1 

	for ind, loc_ := range locs {
		d:=dist_func(loc, loc_)
		
		if d < 0 {
			continue 
		}

		if d < minimal_dist {
			minimal_dist = d 
			ret_ind = ind 
		}
	}

	return ret_ind 
}

func findClosestKey(loc [3]float64, locs map[int][3]float64, dist_func DistanceFunc) int {

	var minimal_dist float64 = 1000000000.0 
	var ret_ind int  = -1 

	for ind, loc_ := range locs {
		d:=dist_func(loc, loc_)
		
		if d < 0 {
			continue 
		}

		if d < minimal_dist {
			minimal_dist = d 
			ret_ind = ind 
		}
	}

	return ret_ind 
}

// defined in scheduler.go 
type localAssignment struct {
	ind1    int 
	ind2    int
	cost   float64 
}

type localAssignmentList struct {
	ass  []localAssignment
}

func (a *localAssignmentList) Len() int {
	return len(a.ass)
}

func (a *localAssignmentList) Swap(i,j int) {

	a.ass[i], a.ass[j] = a.ass[j], a.ass[i]

}

func (a *localAssignmentList) Less(i,j int) bool {
	return a.ass[i].cost < a.ass[j].cost 
}

// locs1 match locs2 
func findBestMatch(locs1 [][3]float64, locs2 [][3]float64, dist_func DistanceFunc) map[int]int {
	if len(locs1) == len(locs2){
		return findBestMatchHungarian(locs1, locs2, dist_func)
	} else {
		return findBestMatchGreedy(locs1, locs2, dist_func)
	}
	
}

func findBestMatchHungarian(locs1 [][3]float64, locs2 [][3]float64, dist_func DistanceFunc) map[int]int {
	//findBestMatchGreedy(locs1, locs2, dist_func)

	a := make([][]int, len(locs1))
	for i:=0;i<len(locs1);i++ {
		a[i] = make([]int, len(locs2))
		for j:=0; j<len(locs2);j++ {
			a[i][j] = int(distance(locs1[i], locs2[j]))
		}
	}

	r, _ := hungarianAlgorithm.Solve(a)
	fmt.Println(r,a)
	ret := make(map[int]int)

	for i:=0; i< len(locs1); i++ {
		ret[i] = r[i]
	}


	return ret 
}



func findBestMatchGreedy(locs1 [][3]float64, locs2 [][3]float64, dist_func DistanceFunc) map[int]int {
	var assList localAssignmentList

	for ind1,loc1 := range locs1 {
		for ind2, loc2 := range locs2 {
			var ass localAssignment

			ass.ind1 = ind1 
			ass.ind2 = ind2 

			ass.cost = dist_func(loc1, loc2)

			assList.ass = append(assList.ass, ass)
		}
	} 

	map1 := make(map[int]bool)
	map2 := make(map[int]bool)

	ret := make(map[int]int)

	sort.Sort(&assList)

	for _, ass := range assList.ass {
		ind1 := ass.ind1 
		ind2 := ass.ind2 

		if _,ok := map1[ind1]; ok {
			continue
		}

		if _,ok := map2[ind2]; ok {
			continue
		}

		map1[ind1] = true 
		map2[ind2] = true 

		ret[ind1] = ind2 
	}

	return ret 
}




type Transformation struct {
	NoLocation 		bool
	OnlyBias		bool
	center			[3]float64
	angle 			float64
	bias			[3]float64
	scale			float64

}

func ApplyTransformation(p1 [3]float64, trans Transformation) [3]float64 {

	p := locSubtract(p1, trans.center)

	if trans.OnlyBias == false {
		p = locRotate(p, trans.angle)
	} 

	return locAdd(trans.center,locAdd(p, trans.bias))
}


func locAdd(p1 [3]float64, p2 [3]float64) [3]float64{
	return [3]float64{p1[0]+p2[0], p1[1]+p2[1], p1[2]+p2[2]}
}

func locSubtract(p1 [3]float64, p2 [3]float64) [3]float64{
	return [3]float64{p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[2]}
}

func locDivide(p1 [3]float64, f float64) [3]float64{
	return [3]float64{p1[0]/f, p1[1]/f, p1[2]/f}
}

func locMul(p1 [3]float64, f float64) [3]float64{
	return [3]float64{p1[0]*f, p1[1]*f, p1[2]*f}
}

func locDot(p1 [3]float64, p2 [3]float64) float64{
	return p1[0] * p2[0] + p1[1]*p2[1] + p1[2]*p2[2]
}

func locRotate(p1 [3]float64, angle float64) [3]float64 {
	x := math.Cos(angle) * p1[0] - math.Sin(angle) * p1[1]
	y := math.Sin(angle) * p1[0] + math.Cos(angle) * p1[1]

	return [3]float64{x,y,p1[2]}
}

func arrAdd(a1 []float64, a2 []float64) []float64 {
	var ret []float64

	for i:=0;i<len(a1); i++ {
		ret = append(ret, a1[i]+a2[i])
	}

	return ret 
}

func arrMedian(a []float64) float64 {
	var sum float64 = 0.0 

	for i:=0; i<len(a);i++{
		sum += a[i]
	}

	var psum float64 = 0.0 

	for i:=0; i<len(a);i++{

		psum += a[i]
		if psum > sum * 0.5 {
			return float64(i) + 0.5
		}
	}

	return 0.0
}

func arrAdd20(a1 [20]float64, a2 [20]float64) [20]float64 {
	var ret [20]float64

	for i:=0;i<len(a1); i++ {
		ret[i] = a1[i]+a2[i]
	}

	return ret 
}

func arrMedian20(a [20]float64) float64 {
	var sum float64 = 0.0 

	for i:=0; i<len(a);i++{
		sum += a[i]
	}

	var psum float64 = 0.0 

	for i:=0; i<len(a);i++{

		psum += a[i]
		if psum > sum * 0.5 {
			return float64(i) + 0.5
		}
	}
	return 0.0
}

type DroneBackend interface{
	//Drones  //[]Drone
	GetDrones() []*Drone 
	SetTarget(n int, loc [3]float64)
	UnsetTarget(n int)
	SetSpeculationTarget(n int, loc [][3]float64)
	Clear(n int)
	SetWayPath(n int, loc [][3]float64) chan int 
	SetReachTargetCallBack(callback chan int)
	Sense(n int, name string) interface{}
	GetTime() float64
	FlyingTimeEst(n int, loc1 [3]float64, loc2 [3]float64) float64
	RemainingFlyingTime(n int) float64
	LogSenseTime(tt float64)
	//UpdateBackground() 
}

type Clock interface{
	SetTriggerCMP(time_tri float64, time_cmp float64, callback chan float64)
	GetTime() float64
}



type frameworkCMD	struct {
	CMD  string `json:"cmd"`
	X	 float64 `json:"x"`
	Y 	 float64 `json:"y"`
	Z	 float64 `json:"z"`
	ID	 int 	 `json:"id"`
	ScopeName string `json:"scopename"`
	NodeName string	`json:"nodename"`
	Dependencies []string `json:"dependencies"`
	MetaInfo 	string `json:"metainfo"`
	ActionName	string `json:"actionname"`
	ActionRet	string `json:"actionret"`
	AcquireDroneID int `json:"droneid"`
	NonInterruptibleFlag bool `json:"noninterruptibleflag"`
	AppID 		string `json:"appid`	 // global
	SessionID	int `json:"sessionid` // temporial 
}

type droneCMD	struct {
	CMD  string `json:"cmd"`
	X	 float64 `json:"x"`
	Y 	 float64 `json:"y"`
	Z	 float64 `json:"z"`
	ID	 int 	 `json:"id"`
	AppID 		string `json:"appid`	 // global
	SessionID	int `json:"sessionid` // temporial
}

type ActionResult struct {
	Status		int `json:"status"`
	Data 		interface{} `json:"data"`
}