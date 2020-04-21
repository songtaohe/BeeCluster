package main 

/*
 * This file implements the level-1 simulator of BeeCLuster 
 * 
 */

import (
	"sync"
	"time"
	"encoding/json"
	"io/ioutil"
	"math"
	"math/rand"
	"fmt"
	"strings"
	"strconv"
)


// This is a basic drone simulator 
// - no wind
// - no acc/de-acc time 
// phase 0 free
// phase 1 flying to target
// phase 2 acquired by app


type Drone struct {
	Loc	[3]float64  // x, y, z  # in meters

	BatteryCap		float64  // in second
	BatteryLevel	float64  // in second (the remaining flying time)
	BatteryReplaceTime float64 // a counter to simulate battery replacement time 

	FlyingSpeed		float64 // meters per second 
	CurrentSpeed	float64 
	AccRate			float64

	Sensors			map[string]int // has sensors 

	TargetLocation	[][3]float64 //
	UserSpaceWayPath	[][3]float64 //
	SpeculationTarget  [][3]float64 //

	AvailableTS		float64 // time the drone will be available

	reachWayPath	chan int 

	// Status 		
	Phase 			int 
	Sleep 			bool 
	RTH  			bool 

	ID 				int 

	// for real drone 
	IsDroneOnline	bool 
	LastSeen 		int64 
	LastSeenTS		float64

	BatteryRatio 	float64

	mATCTicket 		*ATCTicket

	DroneCurrentAction map[int]string 

	Actions 		[]string 
	ActionReturn 	[]chan interface{}

}

type ReplayLogEntry struct {
	TS  float64 `json"ts"`
	Locs  [][3]float64 `json"locs"`
	Phases []int 	`json"phases"`
}

type simulatedEnvironment interface{
	Sense(loc [3]float64, time float64, action string) interface{}
	UpdateBackground(time float64, name string)	
}



type Simulator struct {
	NDrone	int

	Drones  []*Drone

	TimeFromStart 	float64
	FirstActionTS	float64
	LastActionTS	float64
	RealTime		bool 
	StepSize   		float64 

	HomeLocations [][3]float64


	DroneReady		[]bool 
	IsDroneFlying	[]bool 

	mutex	sync.Mutex
	resetMutex sync.Mutex

	callback		chan int 

	ReScheduleCallBack chan int 


	envs 			map[string]simulatedEnvironment

	paused			bool 

	config 			SimConfig
	configfile      string 

	TimeTrigger 	chan float64
	TimeTriggerTime float64 

	ReplayLogPtr    int  
	Logs 			[]ReplayLogEntry

	pauseCounter	int

}


func (sim *Simulator) SetTriggerCMP(time_tri float64, time_cmp float64, callback chan float64) {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	if time_cmp > sim.TimeTriggerTime {
		sim.TimeTriggerTime = time_tri 
		sim.TimeTrigger = callback 
	}


}

type SimConfig struct{
	NDrone  int `json:"ndrone"`
	Loc 	[3]float64 `json:"location"`
	Env 	string `json:"env"`
	FlyingSpeed float64 `json:"speed"`
	BatteryTime	float64 `json:"batteryTime`
	ChangeBatteryTime float64 `json:"changeBatteryTime`
	AccRate		float64 `json:"acc""`
	SimSpeed	int `json:"simspeed"`
	RealTimeSpeed int `json:"realspeed"`
	SenseDelay  float64 `json:"senseDelay"`
	Replay      bool `json:"replay"`
	ReplayFile  string `json:"replayfile"`

}

func ReadConfig(fname string) SimConfig {
	bytes, err := ioutil.ReadFile(fname)
	if err != nil {
		panic(err)
	}
	var cfg SimConfig
	if err := json.Unmarshal(bytes, &cfg); err != nil {
		panic(err)
	}
	return cfg
}




func MakeSimulator(configname string, callback chan int) *Simulator {
	sim := new(Simulator)
	sim.callback = callback
	// TODO load config file 
	sim.configfile = configname
	sim.config = ReadConfig(sim.configfile)

	sim.NDrone = sim.config.NDrone


	for i:=0; i< sim.NDrone; i++ {
		drone := new(Drone)
		drone.ID = i 
		drone.IsDroneOnline = true // always true for simulator

		drone.Loc[0] = sim.config.Loc[0]
		drone.Loc[1] = sim.config.Loc[1]
		drone.Loc[2] = sim.config.Loc[2]

		drone.BatteryCap = sim.config.BatteryTime
		drone.BatteryLevel = sim.config.BatteryTime
		drone.BatteryReplaceTime = sim.config.ChangeBatteryTime

		drone.FlyingSpeed = sim.config.FlyingSpeed
		drone.CurrentSpeed = 0.0 
		drone.AccRate = sim.config.AccRate

		drone.Sensors = make(map[string]int)

		drone.Sensors["pm25"] = 1
		drone.Sensors["RGBCamera"] = 1
		drone.Sensors["ThermalCamera"] = 1
		drone.Phase = 0 
		drone.RTH = false

		sim.DroneReady = append(sim.DroneReady, false)
		sim.IsDroneFlying = append(sim.IsDroneFlying, false)
		sim.Drones = append(sim.Drones, drone)




	}

	sim.StepSize = 0.01 // too small?
	sim.RealTime = true 
	sim.TimeFromStart = 0.0  
	sim.HomeLocations = append(sim.HomeLocations, [3]float64{0.0, 0.0, 0.0})
	sim.FirstActionTS = -1.0 

	if sim.config.Replay == true {
		bytes, err := ioutil.ReadFile(sim.config.ReplayFile)
		if err != nil {
			panic(err)
		}
		
		if err := json.Unmarshal(bytes, &(sim.Logs)); err != nil {
			panic(err)
		}


		sim.ReplayLogPtr = 0
	}


	sim.pauseCounter = 0

	sim.ResetEnv()
	sim.paused = false

	go sim.Run()

	return sim 
}

func (sim *Simulator) SetReachTargetCallBack(callback chan int) {
	sim.callback = callback
}

func (sim *Simulator) Log() {
	var entry ReplayLogEntry

	entry.TS = sim.TimeFromStart 

	for _, drone := range sim.Drones {
		entry.Locs = append(entry.Locs, drone.Loc)
		entry.Phases = append(entry.Phases, drone.Phase )
	}

	sim.Logs = append(sim.Logs, entry)

}


func (sim *Simulator) LogSenseTime(tt float64) {
	
}


// run the simulator for one step 
func (sim *Simulator) Step() {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()


	var acc float64 = 2.8
	var dacc float64 = 3.8 
	var air_r float64 = 0.0006
	var d_tp float64 = 20.0 
	var s_slow float64 = 2.0 


	if sim.paused == true {
		return 
	}

	//fmt.Println("mark", sim.paused)

	if int(sim.TimeFromStart * 100) % 10 == 0 && sim.config.Replay == false {
		sim.Log()
	}

	sim.TimeFromStart += sim.StepSize

	if sim.TimeFromStart > sim.TimeTriggerTime && sim.TimeTriggerTime > 1.0 {
		sim.TimeTriggerTime = 0.0
		sim.TimeTrigger <- sim.TimeFromStart
	}


	if sim.config.Replay == true {
		if sim.ReplayLogPtr < len(sim.Logs) {
			entry := sim.Logs[sim.ReplayLogPtr]

			for i:=0; i<sim.NDrone; i++ {
				drone := sim.Drones[i]
				drone.Loc = entry.Locs[i] 
				drone.Phase = entry.Phases[i]

				// a hack for visualization
				//if drone.Phase == 2 {
				//	sim.env.Sense(drone.Loc, sim.TimeFromStart, "sense")
				//}
			} 

			if sim.TimeFromStart > entry.TS {
				sim.ReplayLogPtr += 1
			}
		} else  {
			sim.ReplayLogPtr = 0
			sim.TimeFromStart = 0.0
		}

		return 
	}


	// if int(sim.TimeFromStart*100) % 100 == 0 {
	// 	fmt.Println(sim.LastActionTS - sim.FirstActionTS, sim.Drones[0])
	// } 

	var reschedule bool = false

	for i:=0; i<sim.NDrone; i++ {
		drone := sim.Drones[i]

		var timeToHome float64 = sim.FlyingTimeEst(i, drone.Loc, sim.HomeLocations[0])

		if timeToHome + 30 > drone.BatteryLevel && drone.RTH == false {
			drone.RTH = true
			drone.BatteryReplaceTime = 0.0
			reschedule = true 
		}

		if drone.RTH == false {
			if sim.DroneReady[i] == false {
				if len(drone.TargetLocation) == 0 {
					
					// fly to the speculation target
					if len(drone.SpeculationTarget) != 0 {
						target := drone.SpeculationTarget[0]
						d := distance(target, drone.Loc)


						//t := drone.CurrentSpeed / drone.AccRate 


						air_acc := air_r * drone.CurrentSpeed * drone.CurrentSpeed
						if d > d_tp {
							if drone.CurrentSpeed < drone.FlyingSpeed {
								drone.CurrentSpeed += (acc - air_acc) * sim.StepSize
							}
						} else {
							if drone.CurrentSpeed > s_slow {
								drone.CurrentSpeed -= (dacc + air_acc) * sim.StepSize
							}

							if drone.CurrentSpeed < s_slow {
								drone.CurrentSpeed += (acc - air_acc) * sim.StepSize
							}

							if drone.CurrentSpeed < 0.1 {
								drone.CurrentSpeed = 0.1
							}
						}


						// if d < 0.5 * drone.AccRate * t * t {
						// 	drone.CurrentSpeed -= drone.AccRate * sim.StepSize

						// 	if drone.CurrentSpeed < 0.1 {
						// 		drone.CurrentSpeed = 0.1
						// 	}

						// } else if drone.CurrentSpeed < drone.FlyingSpeed {
						// 	drone.CurrentSpeed += drone.AccRate * sim.StepSize
						// }


						l := sim.StepSize * drone.CurrentSpeed

						if l > d {
							drone.Loc = target 
							
						} else {
							a := l/d 

							drone.Loc[0] = drone.Loc[0] + a * (target[0] - drone.Loc[0])
							drone.Loc[1] = drone.Loc[1] + a * (target[1] - drone.Loc[1])
							drone.Loc[2] = drone.Loc[2] + a * (target[2] - drone.Loc[2])

						}

					}
				} else {
					target := drone.TargetLocation[0]
					d := distance(target, drone.Loc)

					//t := drone.CurrentSpeed / drone.AccRate 

					air_acc := air_r * drone.CurrentSpeed * drone.CurrentSpeed
					if d > d_tp {
						if drone.CurrentSpeed < drone.FlyingSpeed {
							drone.CurrentSpeed += (acc - air_acc) * sim.StepSize
						}
					} else {
						if drone.CurrentSpeed > s_slow {
							drone.CurrentSpeed -= (dacc + air_acc) * sim.StepSize
						}

						if drone.CurrentSpeed < s_slow {
							drone.CurrentSpeed += (acc - air_acc) * sim.StepSize
						}

						if drone.CurrentSpeed < 0.1 {
							drone.CurrentSpeed = 0.1
						}
					}

					// if d < 0.5 * drone.AccRate * t * t {
					// 	drone.CurrentSpeed -= drone.AccRate * sim.StepSize

					// 	if drone.CurrentSpeed < 0.1 {
					// 		drone.CurrentSpeed = 0.1
					// 	}

					// } else if drone.CurrentSpeed < drone.FlyingSpeed {
					// 	drone.CurrentSpeed += drone.AccRate * sim.StepSize
					// }


					l := sim.StepSize * drone.CurrentSpeed

					if l > d || d < 1.0{
						drone.Loc = target 
						sim.DroneReady[i] = true // trigger a call back?
						//fmt.Println("before signal", i)
						drone.Phase = 2 
						sim.callback <- i // this channel needs to be large enough !
						//fmt.Println("after signal", i)

					} else {
						a := l/d 

						drone.Loc[0] = drone.Loc[0] + a * (target[0] - drone.Loc[0])
						drone.Loc[1] = drone.Loc[1] + a * (target[1] - drone.Loc[1])
						drone.Loc[2] = drone.Loc[2] + a * (target[2] - drone.Loc[2])

					}
				}
			} else {
				if len(drone.UserSpaceWayPath) == 0 {
					// do nothing here
				} else {
					target := drone.UserSpaceWayPath[0]
					d := distance(target, drone.Loc)

					//t := drone.CurrentSpeed / drone.AccRate 
					air_acc := air_r * drone.CurrentSpeed * drone.CurrentSpeed
					if d > d_tp {
						if drone.CurrentSpeed < drone.FlyingSpeed {
							drone.CurrentSpeed += (acc - air_acc) * sim.StepSize
						}
					} else {
						if drone.CurrentSpeed > s_slow {
							drone.CurrentSpeed -= (dacc + air_acc) * sim.StepSize
						}

						if drone.CurrentSpeed < s_slow {
							drone.CurrentSpeed += (acc - air_acc) * sim.StepSize
						}

						if drone.CurrentSpeed < 0.1 {
							drone.CurrentSpeed = 0.1
						}
					}
						
					// if d < 0.5 * drone.AccRate * t * t {
						
					// 	drone.CurrentSpeed -= drone.AccRate * sim.StepSize


					// 	if drone.CurrentSpeed < 0.1 {
					// 		drone.CurrentSpeed = 0.1
					// 	}
						
					// } else if drone.CurrentSpeed < drone.FlyingSpeed {
					// 	drone.CurrentSpeed += drone.AccRate * sim.StepSize
					// }


					l := sim.StepSize * drone.CurrentSpeed

					if l > d {
						drone.Loc = target 
						//sim.DroneReady[i] = true
						drone.reachWayPath <- 1
						drone.UserSpaceWayPath = drone.UserSpaceWayPath[1:]

					} else {
						a := l/d 

						drone.Loc[0] = drone.Loc[0] + a * (target[0] - drone.Loc[0])
						drone.Loc[1] = drone.Loc[1] + a * (target[1] - drone.Loc[1])
						drone.Loc[2] = drone.Loc[2] + a * (target[2] - drone.Loc[2])

					}
				}
			}
		} else { // RTH
			target := sim.HomeLocations[0] 
			d := distance(target, drone.Loc)

			//t := drone.CurrentSpeed / drone.AccRate 
			air_acc := air_r * drone.CurrentSpeed * drone.CurrentSpeed
			if d > d_tp {
				if drone.CurrentSpeed < drone.FlyingSpeed {
					drone.CurrentSpeed += (acc - air_acc) * sim.StepSize
				}
			} else {
				if drone.CurrentSpeed > s_slow {
					drone.CurrentSpeed -= (dacc + air_acc) * sim.StepSize
				}

				if drone.CurrentSpeed < s_slow {
					drone.CurrentSpeed += (acc - air_acc) * sim.StepSize
				}

				if drone.CurrentSpeed < 0.1 {
					drone.CurrentSpeed = 0.1
				}
			}
			
			l := sim.StepSize * drone.CurrentSpeed

			if l > d {
				drone.Loc = target 
				
			} else {
				a := l/d 

				drone.Loc[0] = drone.Loc[0] + a * (target[0] - drone.Loc[0])
				drone.Loc[1] = drone.Loc[1] + a * (target[1] - drone.Loc[1])
				drone.Loc[2] = drone.Loc[2] + a * (target[2] - drone.Loc[2])
			}
		}


		//if sim.IsDroneFlying[i] == true{
		//drone.BatteryLevel -= sim.StepSize
		//}

		// if drone is at (near) home, the battery doesn't drain (simulation)
		if distance(drone.Loc, sim.HomeLocations[0]) > 5 {
			drone.BatteryLevel -= sim.StepSize
		}		

		if distance(drone.Loc, sim.HomeLocations[0]) < 10 && drone.RTH == true {
			if drone.BatteryReplaceTime > sim.config.ChangeBatteryTime {
		 		drone.BatteryReplaceTime = 0.0
				drone.BatteryLevel = drone.BatteryCap
				drone.RTH = false
				reschedule = true
			} else {
				drone.BatteryReplaceTime += sim.StepSize
			}
		}
	}

	if reschedule {
		sim.ReScheduleCallBack <- 1
	}
}

// simulator loop 
func (sim *Simulator) Run() {
	for {
		if sim.paused==false{
			sim.Step()
		}

		flag := false

		for i:=0; i<sim.NDrone; i++ {
			if sim.DroneReady[i] == true && sim.Drones[i].Sleep == false{
				flag = true
				break
			}
		}

		// if there is no drone flying toward any targets, slow down the simulator 
		allidle := true
		for i:=0; i<sim.NDrone; i++ {
			if sim.Drones[i].Phase != 0 {
				allidle = false 
				break 
			}
		}

		// when the flag is true, the python code is involved. In this case,
		// the simulator tries to simulate real time. Otherwise, the simulator
		// runs as fast as it can to save time. 
		
		if flag == true {
			sim.LastActionTS = sim.TimeFromStart

			if sim.FirstActionTS < 0.0 {
				sim.FirstActionTS = sim.TimeFromStart
			}
		}

		

		if (flag == true || allidle == true) && sim.config.Replay == false{
			// real-time
			//time.Sleep(time.Duration(int(sim.StepSize*1000000.0)) * time.Microsecond)
			time.Sleep(time.Duration(int(sim.StepSize*1000000.0)/sim.config.RealTimeSpeed) * time.Microsecond)
		} else {
			// fast version 
			time.Sleep(time.Duration(int(sim.StepSize*1000000.0)/sim.config.SimSpeed) * time.Microsecond)
		}
	}
}

// internal usage 
func (sim *Simulator) SetTarget(n int, loc [3]float64) {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	drone := sim.Drones[n]
	drone.TargetLocation = [][3]float64{loc}

	if drone.Phase == 2 {
		panic("set target error, phase was 2, cannot set it back to 1")
	}

	drone.Phase = 1 
	//fmt.Println(sim.Drones[n], sim.DroneReady[n])
}

func (sim *Simulator) UnsetTarget(n int) {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	drone := sim.Drones[n]
	drone.TargetLocation = [][3]float64{}
	drone.Phase = 0
	//fmt.Println(sim.Drones[n], sim.DroneReady[n])
}

// internal usage 
func (sim *Simulator) SetSpeculationTarget(n int, loc [][3]float64) {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	drone := sim.Drones[n]
	if len(loc) != 0 {
		drone.SpeculationTarget = [][3]float64{loc[0]}
	} else {
		drone.SpeculationTarget = [][3]float64{}
	}
	//drone.Phase = 1 
	//fmt.Println(sim.Drones[n], sim.DroneReady[n])
}


// clear the 'ready' drone 
func (sim *Simulator) Clear(n int) {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	drone := sim.Drones[n]
	sim.DroneReady[n] = false
	drone.TargetLocation = drone.TargetLocation[1:]
	drone.Phase = 0 
	//fmt.Println(sim.Drones[n])
} 


//  rpc call from user program. They have to acquire the drone first 
func (sim *Simulator) SetWayPath(n int, loc [][3]float64) chan int {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	drone := sim.Drones[n]
	drone.UserSpaceWayPath = loc

	drone.reachWayPath = make(chan int,len(loc))


	return drone.reachWayPath
}


func (sim * Simulator) GetDrones() []*Drone {
	return sim.Drones
}


func (sim * Simulator) Sense(n int, name string) interface{} {
	sim.mutex.Lock()
	//defer sim.mutex.Unlock()

	if name == "GetTime" {
		sim.mutex.Unlock()  
		return sim.TimeFromStart
	}

	if name == "GetDuration" {
		sim.mutex.Unlock()  
		PrintLog(LogDebug, fmt.Sprintln("GetDuration", sim.LastActionTS - sim.FirstActionTS))
		return sim.LastActionTS - sim.FirstActionTS
	}

	if strings.HasPrefix(name, "VirtualSleep_") {
		t0 := sim.TimeFromStart

		delay , _ := strconv.ParseFloat(strings.Split(name,"_")[1], 64)
		//fmt.Println("delay", delay)
		t1 := t0 + delay

		sim.mutex.Unlock()

		sim.Drones[n].Sleep = true 
		for {
			
			//fmt.Println(t1)
			if t1 < sim.TimeFromStart {
				break
			}

			time.Sleep(time.Duration(int(sim.StepSize*1000000.0)/sim.config.SimSpeed) * time.Microsecond)
			
		}
		sim.Drones[n].Sleep = false


		return "ack"
	}
	
	var ret ActionResult 

	if sim.Drones[n].RTH == true {
		ret.Status = -1
		sim.mutex.Unlock()  
		return ret 
	}

	// name = envname:actionname
	envname := strings.Split(name,":")[0]
	actionname := strings.Split(name,":")[1]


	ret_ := sim.envs[envname].Sense(sim.Drones[n].Loc, sim.TimeFromStart, actionname)
	ret.Data = ret_ 

	// virtual sleep 
	t0 := sim.TimeFromStart

	t1 := t0 + sim.config.SenseDelay 

	sim.mutex.Unlock()  

	sim.Drones[n].Sleep = true 
	for {
		
		//fmt.Println(t1)
		if t1 < sim.TimeFromStart {
			break
		}

		time.Sleep(time.Duration(int(sim.StepSize*1000000.0)/sim.config.SimSpeed) * time.Microsecond)
		
	}
	sim.Drones[n].Sleep = false

	return ret
}



func (sim * Simulator) GetTime() float64 {
	return sim.TimeFromStart
}

func (sim * Simulator) ResetTime() {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	sim.TimeFromStart = 0.0 
	sim.FirstActionTS = -1.0 
	sim.LastActionTS = 0.0

}

func (sim *Simulator) ResetEnv(){

	t0 := makeTimestamp()

	sim.envs = make(map[string]simulatedEnvironment)

	sim.envs["gp"] = MakeGaussianProcessEnv(20)
	sim.envs["staticPollution"] = MakeStaticPollutionEnv(10)
	sim.envs["dynamicPollution"] = MakeDynamicPollutionEnv(10)
	sim.envs["roadtracer"] = MakeRoadTracerEnv()
	sim.envs["orthophoto"] = MakeOrthophotoEnv(16,0.0)
	sim.envs["wifi"] = MakeWifiEnv()


	PrintLog(LogDebug, fmt.Sprintln("time spent resetting envs", makeTimestamp() - t0))

	//sim.env = MakeGaussianProcessEnv(20)
	//sim.env = MakeStaticPollutionEnv(10)
	//sim.env = MakeDynamicPollutionEnv(10)
	//sim.env.UpdateBackground(0.0)
	//sim.env = MakeRoadTracerEnv()
}


func (sim *Simulator) UpdateBackground(name string) {
	sim.resetMutex.Lock()
	defer sim.resetMutex.Unlock()

	purename := strings.Split(name, ".")[0]

	for k,env := range sim.envs {
		env.UpdateBackground(sim.TimeFromStart, purename + "_" + k + ".png")
	}
	//sim.env.UpdateBackground(sim.TimeFromStart, name)
}



// maybe not use pause and resume 
func (sim * Simulator) Pause() {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()
	
	sim.pauseCounter += 1
	PrintLog(LogDebug, fmt.Sprintln("simulator paused", sim.pauseCounter))

	sim.paused = true 
}

func (sim * Simulator) Resume() {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()
	
	sim.pauseCounter -= 1
	PrintLog(LogDebug, fmt.Sprintln("simulator resumed", sim.pauseCounter))

	if sim.pauseCounter == 0 {
		sim.paused = false
	}
}


func (sim * Simulator) Reset() {
	sim.mutex.Lock()
	defer sim.mutex.Unlock()

	sim.resetMutex.Lock()
	defer sim.resetMutex.Unlock()

	sim.DroneReady = []bool{}
	sim.IsDroneFlying = []bool{}
	sim.Drones = []*Drone{}

	if sim.config.Replay == false {
		bytes, _ := json.MarshalIndent(sim.Logs, "", "    ")

		err := ioutil.WriteFile("simulator_log.json", bytes, 0644)

		if err != nil {
			PrintLog(LogError, fmt.Sprintln(err))
		}
	}



	sim.config = ReadConfig(sim.configfile)
	sim.NDrone = sim.config.NDrone

	sim.ResetEnv()


	PrintLog(LogInfo, fmt.Sprintln("reset simulator"))

	for i:=0; i< sim.NDrone; i++ {
		drone := new(Drone)
		drone.ID = i
		drone.IsDroneOnline = true
		
		drone.Loc[0] = sim.config.Loc[0] + 4*(rand.Float64() - 0.5)
		drone.Loc[1] = sim.config.Loc[1] + 4*(rand.Float64() - 0.5)
		drone.Loc[2] = sim.config.Loc[2]

		drone.BatteryCap = sim.config.BatteryTime
		drone.BatteryLevel = sim.config.BatteryTime
		drone.BatteryReplaceTime = 0.0 //sim.config.ChangeBatteryTime
		

		drone.FlyingSpeed = sim.config.FlyingSpeed
		drone.CurrentSpeed = 0.0 
		drone.AccRate = sim.config.AccRate

		drone.Phase = 0
		drone.RTH = false

		drone.Sensors = make(map[string]int)

		drone.Sensors["pm25"] = 1
		drone.Sensors["RGBCamera"] = 1
		drone.Sensors["ThermalCamera"] = 1

		sim.DroneReady = append(sim.DroneReady, false)
		sim.IsDroneFlying = append(sim.IsDroneFlying, false)
		sim.Drones = append(sim.Drones, drone)
	}



	sim.TimeFromStart = 0.0 
	sim.FirstActionTS = -1.0 
	sim.LastActionTS = 0.0
	sim.TimeTriggerTime = -0.0 
	sim.Logs = []ReplayLogEntry{}


	if sim.config.Replay == true {
		bytes, err := ioutil.ReadFile(sim.config.ReplayFile)
		if err != nil {
			panic(err)
		}
		
		if err := json.Unmarshal(bytes, &(sim.Logs)); err != nil {
			panic(err)
		}


		sim.ReplayLogPtr = 0
	}



}


func (sim * Simulator) FlyingTimeEst(n int, loc1 [3]float64, loc2 [3]float64) float64 {


	t0 := sim.Drones[n].CurrentSpeed/sim.Drones[n].AccRate

	d := distance(loc1, loc2) + t0*t0*sim.Drones[n].AccRate // 

	t:= sim.Drones[n].FlyingSpeed / sim.Drones[n].AccRate

	if d > t*t * sim.Drones[n].AccRate {
		return (d-t*t * sim.Drones[n].AccRate)/sim.Drones[n].FlyingSpeed + t*2
	} else {

		return math.Sqrt(d / sim.Drones[n].AccRate) * 2
	}

}

func (sim * Simulator) RemainingFlyingTime(n int) float64 {
	var timeToHome float64 = sim.FlyingTimeEst(n, sim.Drones[n].Loc, sim.HomeLocations[0])
	return sim.Drones[n].BatteryLevel - (timeToHome + 30)
}
