package main 

/*
 * This file implements the drone backend for real drones.
 * The interface is defined in common.go
 */
import (
	"sync"
	"time"
	"encoding/json"
	"io/ioutil"
	"net/http"
	"net/url"
	"log"
	"math"
	"fmt"
	"os"
)


type DroneHub struct {
	NDrone	int

	Drones  []*Drone

	BaseTime		int64
	TimeFromStart 	float64
	FirstActionTS	float64
	LastActionTS	float64
	//RealTime		bool 
	//StepSize   		float64 

	HomeLocations [][3]float64

	DroneReady		[]bool 
	IsDroneFlying	[]bool 

	mutex	sync.Mutex
	resetMutex sync.Mutex

	callback		chan int 

	envs 			map[string]simulatedEnvironment

	paused			bool 

	config 			DroneHubConfig
	configfile      string 

	TimeTrigger 	chan float64
	TimeTriggerTime float64 

	ReScheduleCallBack chan int 

	ReplayLogPtr    int  
	Logs 			[]ReplayLogEntry

	pauseCounter	int

	mATC 			*ATC

	TotalSenseTime float64
}


type DroneHubConfig struct {
	NDrone  int `json:"maxdrone"`
	Loc 	[3]float64 `json:"location"`
	FlyingSpeed float64 `json:"speed"`
	AccRate		float64 `json:"acc""`

	Port 	string `json:"port"`

	Local2GlobalBiasList [][3]float64 `json:"bias"`

}

func ReadDroneHubConfig(fname string) DroneHubConfig {
	bytes, err := ioutil.ReadFile(fname)
	if err != nil {
		panic(err)
	}
	var cfg DroneHubConfig
	if err := json.Unmarshal(bytes, &cfg); err != nil {
		panic(err)
	}
	return cfg
}


func MakeDroneHub(configname string, callback chan int) *DroneHub {
	hub := new(DroneHub)
	hub.callback = callback
	hub.configfile = configname
	hub.config = ReadDroneHubConfig(configname)
	hub.TotalSenseTime = 0.0 
	hub.NDrone = hub.config.NDrone // the maximum number of drones 

	hub.BaseTime = makeTimestamp()

	for i:=0; i<hub.NDrone; i++ {
		drone := new(Drone)
		drone.ID = i 

		drone.IsDroneOnline = false 

		drone.Phase = 0  

		drone.FlyingSpeed = hub.config.FlyingSpeed
		drone.AccRate = hub.config.AccRate


		hub.DroneReady = append(hub.DroneReady, false)
		hub.IsDroneFlying = append(hub.IsDroneFlying, false)
		hub.Drones = append(hub.Drones, drone)
	}

	hub.TimeFromStart = 0.0  
	hub.HomeLocations = append(hub.HomeLocations, [3]float64{0.0, 0.0, 0.0})
	hub.FirstActionTS = -1.0 

	hub.mATC = MakeATC(hub.NDrone)

	hub.Run()

	return hub 
}



func (hub *DroneHub) Run() {
	go hub.DroneListener()

}



// messages

type DroneToHub struct {
	DroneID 	int  `json"id"`   // need to be unique

	LocXYZ	[]float64 `json"locxyz"`
	LocGPS	[]float64 `json"locgps"`

	Battery	float64 `json"battery"`

	RawInfo string `json"rawinfo"`


	PayLoadDatas 	[]string  `json"payloaddata"` // for sensing data such as image 
	PayLoadUIDs		[]int `json"payloaduid"` 

	TransmissionTime float64  `json"TransmissionTime"`
	ActionTime float64 `json"ActionTime"`
}


type HubToDrone struct {

	Local2GlobalBias	[3]float64 `json"local2globalbias"`			

	Actions 		[]string `json"action"` 
	ActionUIDs 		[]int `json"actionUID"`
}

func (hub *DroneHub) AppendLog(logfile string, s string) {
	
	f, _ := os.OpenFile(logfile, os.O_APPEND|os.O_RDWR|os.O_CREATE, 0666)
	f.WriteString(s)
	f.Close()
}

func (hub *DroneHub) LogSenseTime(tt float64) {
	hub.mutex.Lock()
	hub.TotalSenseTime += tt
	hub.mutex.Unlock()
}

func (hub *DroneHub) DroneListener() {

	callbacks := make(map[int]chan interface{})
	var uid int = 0 
	msg_counter := make([]int, len(hub.Drones))

	// TODO add timeout 

	logfile := "drone_trace_log/log-"+time.Now().Format("2006-01-02-15-04-05")
	hub.AppendLog(logfile, "start\n")

	var ActionTransmissionTime float64 = 0.0 
	var ActionTime float64 = 0.0 
	var FlyingTime float64 = 0.0 
	var IsDroneFlying bool = false

	var last_action_time float64 = 0.0 

	FlyingTimeTS := makeTimestamp() 

	go hub.DroneTimeout()

	controlPlane := http.NewServeMux()
	controlPlane.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {

		// update drone state in drone hub
		// send new update to physical drone 

		bytes, _ := ioutil.ReadAll(r.Body)
		defer r.Body.Close()

		var msg_in DroneToHub

		if err := json.Unmarshal(bytes, &msg_in); err != nil {
			//fmt.Println(string(buf))
			panic(err)
		}

		hub.mutex.Lock()


		if msg_in.ActionTime > 0.0001 {
			ActionTime += msg_in.ActionTime
			//ActionTransmissionTime += msg_in.TransmissionTime
		}

		if last_action_time > 0.0001 {
			fmt.Println("time", msg_in.TransmissionTime, msg_in.ActionTime, last_action_time)

			//ActionTime += last_action_time
			ActionTransmissionTime += msg_in.TransmissionTime
		}

		if len(msg_in.PayLoadDatas) > 0 {
			last_action_time = 1
		} else {
			last_action_time = 0
		}


		//last_action_time = msg_in.ActionTime


		// wall time; drone id; flying time; action transmission time, action time (driver); 
		hub.AppendLog(logfile, fmt.Sprintf("%.3f %d %.3f %.3f %.3f %.3f %s", hub.GetTime(), msg_in.DroneID, FlyingTime, ActionTransmissionTime,ActionTime, hub.TotalSenseTime,  msg_in.RawInfo))

		var action_str string = "standby"
		
		did := msg_in.DroneID 

		msg_counter[did] += 1 

		if msg_counter[did] % 10 == 0 {
			hub.mATC.dumpQueue()
			fmt.Println("Drone ",did, "RawInfo", msg_in.RawInfo)
		}


		hub.Drones[did].LastSeenTS = hub.GetTime()



		if hub.Drones[did].IsDroneOnline == false {
			log.Println("New drone online, drone id", did)
			hub.Drones[did].IsDroneOnline = true 
			hub.ReScheduleCallBack <- 1
		}


		for ind, payload := range msg_in.PayLoadDatas {
			payloaduid := msg_in.PayLoadUIDs[ind]

			if _,ok := callbacks[payloaduid]; ok  {

				//fmt.Println("payload", payload)

				callbacks[payloaduid] <- payload 

				delete(callbacks, payloaduid)
			}
		}

		//last_transmission_time = msg_in.TransmissionTime

		hub.Drones[did].Loc[0] = msg_in.LocXYZ[0]
		hub.Drones[did].Loc[1] = msg_in.LocXYZ[1]
		hub.Drones[did].Loc[2] = msg_in.LocXYZ[2]

		hub.mATC.setDroneLocations(did, hub.Drones[did].Loc)

		//fmt.Println(did, hub.Drones[did].Loc, msg_in)

		hub.Drones[did].BatteryRatio = msg_in.Battery

		i := did 
		drone := hub.Drones[i]
		IsDroneFlying = true

		if hub.DroneReady[i] == false {
			if len(drone.TargetLocation) == 0 {
				// fly to the speculation target
				if len(drone.SpeculationTarget) != 0 {
					target := drone.SpeculationTarget[0]

					// ATC protocol 
					if drone.mATCTicket != nil && drone.mATCTicket.Target == target {
						if drone.mATCTicket.Solution != ATC_PROCEED {
							hub.mATC.updateTicket(drone.mATCTicket)
						}
					} else {
						if drone.mATCTicket != nil {
							hub.mATC.cancelTicket(drone.mATCTicket)
						}

						drone.mATCTicket = hub.mATC.getTicket(drone.Loc, target, did)
					}

					// ATC cmd 
					t := drone.mATCTicket
					atc_cmd := fmt.Sprintf("_atc_%d_wp1_%.6f_%.6f_%.6f_wp2_%.6f_%.6f_%.6f_hp_%.6f_%.6f_%.6f", t.Solution, t.ControlPoint1[0], t.ControlPoint1[1], t.ControlPoint1[2], t.ControlPoint2[0], t.ControlPoint2[1], t.ControlPoint2[2], t.HoldPoint[0], t.HoldPoint[1], t.HoldPoint[2])
					action_str = fmt.Sprintf("flyto_%.6f_%.6f_%.6f_%s", target[0], target[1], target[2], atc_cmd)
					

					d := distance(target, drone.Loc)
					if d < 1.0 {
						drone.SpeculationTarget = [][3]float64{}
						IsDroneFlying = false
						//hub.mATC.cancelTicket(drone.mATCTicket)
					} 
				}else{
					IsDroneFlying = false
				}

			} else {
				target := drone.TargetLocation[0]

				// ATC protocol 
				if drone.mATCTicket != nil && drone.mATCTicket.Target == target {
					if drone.mATCTicket.Solution != ATC_PROCEED {
						//fmt.Println("call updateTicket")
						hub.mATC.updateTicket(drone.mATCTicket)
						fmt.Println("call updateTicket", *drone.mATCTicket)
					}
				} else {
					if drone.mATCTicket != nil {
						hub.mATC.cancelTicket(drone.mATCTicket)
					}

					drone.mATCTicket = hub.mATC.getTicket(drone.Loc, target, did)
				}


				t := drone.mATCTicket
				atc_cmd := fmt.Sprintf("_atc_%d_wp1_%.6f_%.6f_%.6f_wp2_%.6f_%.6f_%.6f_hp_%.6f_%.6f_%.6f", t.Solution, t.ControlPoint1[0], t.ControlPoint1[1], t.ControlPoint1[2], t.ControlPoint2[0], t.ControlPoint2[1], t.ControlPoint2[2], t.HoldPoint[0], t.HoldPoint[1], t.HoldPoint[2])
				action_str = fmt.Sprintf("flyto_%.6f_%.6f_%.6f_%s", target[0], target[1], target[2], atc_cmd)
					
				d := distance(target, drone.Loc)

				if d < 1.0 {

					hub.DroneReady[i] = true 
					fmt.Println("before signal", i)
					drone.Phase = 2 
					hub.callback <- i 
					fmt.Println("after signal", i)
					IsDroneFlying = false
					//hub.mATC.cancelTicket(drone.mATCTicket)
				} 

			}
		} else {
			if len(drone.UserSpaceWayPath) == 0 {
				IsDroneFlying = false
			} else {
				target := drone.UserSpaceWayPath[0]

				// ATC protocol 
				if drone.mATCTicket != nil && drone.mATCTicket.Target == target {
					if drone.mATCTicket.Solution != ATC_PROCEED {
						hub.mATC.updateTicket(drone.mATCTicket)
					}
				} else {
					if drone.mATCTicket != nil {
						hub.mATC.cancelTicket(drone.mATCTicket)
					}

					drone.mATCTicket = hub.mATC.getTicket(drone.Loc, target, did)
				}

				t := drone.mATCTicket
				atc_cmd := fmt.Sprintf("_atc_%d_wp1_%.6f_%.6f_%.6f_wp2_%.6f_%.6f_%.6f_hp_%.6f_%.6f_%.6f", t.Solution, t.ControlPoint1[0], t.ControlPoint1[1], t.ControlPoint1[2], t.ControlPoint2[0], t.ControlPoint2[1], t.ControlPoint2[2], t.HoldPoint[0], t.HoldPoint[1], t.HoldPoint[2])
				action_str = fmt.Sprintf("flyto_%.6f_%.6f_%.6f_%s", target[0], target[1], target[2], atc_cmd)
					
				d := distance(target, drone.Loc)

				if d < 1.0 {
					drone.reachWayPath <- 1
					drone.UserSpaceWayPath = drone.UserSpaceWayPath[1:]

					IsDroneFlying = false
					//hub.mATC.cancelTicket(drone.mATCTicket)
				}
			}
		}


		if IsDroneFlying == true {
			FlyingTime += float64(makeTimestamp() - FlyingTimeTS)/1000.0
		}

		FlyingTimeTS = makeTimestamp() 

		var msg_out HubToDrone 

		msg_out.Actions = append(msg_out.Actions, action_str)

		if len(drone.Actions) != 0 {
			for ind, action_s := range drone.Actions {
				msg_out.Actions = append(msg_out.Actions, action_s)
				msg_out.ActionUIDs = append(msg_out.ActionUIDs, uid)
				callbacks[uid] = drone.ActionReturn[ind]
				uid += 1
			}

			// clear the actions 
			drone.Actions = []string{}
			drone.ActionReturn = []chan interface{}{}
		}

		hub.mutex.Unlock()

		msg_out.Local2GlobalBias = hub.config.Local2GlobalBiasList[did]

		retBytes, err := json.Marshal(msg_out)
		if err != nil {
			panic(err)
		}

		w.Header().Set("Content-Type", "application/json")
		w.Write(retBytes)
	})



	u, err := url.Parse("http://192.168.1.21:" + hub.config.Port)

	if err != nil {

		if err != nil {
			panic(err)
		}
	}
	log.Printf("[DroneHub] listening on %s", u.Host)

	err = http.ListenAndServe(u.Host, controlPlane)

	if err != nil {
		fmt.Println(err, "try localhost")

		u, _ = url.Parse("http://localhost:" + hub.config.Port)

		log.Fatal(http.ListenAndServe(u.Host, controlPlane))
	}

}


func (hub *DroneHub) DroneTimeout() {
	for {
		t_now := hub.GetTime()

		hub.mutex.Lock()

		var reschedule bool = false 

		for did,drone := range hub.Drones {
			if drone.IsDroneOnline && t_now - drone.LastSeenTS > 10.0 {
				drone.IsDroneOnline = false 
				reschedule = true 
				log.Println("(warning) Drone offline, drone id", did)
			}
		}

		if reschedule {
			hub.ReScheduleCallBack <- 1
		}


		hub.mutex.Unlock()
		


		time.Sleep(time.Duration(1000) * time.Microsecond)
	}
}


func (hub *DroneHub) GetDrones() []*Drone {
	return hub.Drones
}


func (hub *DroneHub) SetTarget(n int, loc [3]float64) {
	hub.mutex.Lock()
	defer hub.mutex.Unlock()

	drone := hub.Drones[n]
	drone.TargetLocation = [][3]float64{loc}

	if drone.Phase == 2 {
		panic("set target error, phase was 2, cannot set it back to 1")
	}

	drone.Phase = 1 
}

func (hub *DroneHub) UnsetTarget(n int) {
	hub.mutex.Lock()
	defer hub.mutex.Unlock()

	drone := hub.Drones[n]
	drone.TargetLocation = [][3]float64{}
	drone.Phase = 0

	if drone.mATCTicket != nil {
		hub.mATC.cancelTicket(drone.mATCTicket)
	}
}

func (hub *DroneHub) SetSpeculationTarget(n int, loc [][3]float64) {
	hub.mutex.Lock()
	defer hub.mutex.Unlock()

	drone := hub.Drones[n]
	if len(loc) != 0 {
		drone.SpeculationTarget = [][3]float64{loc[0]}
	} else {
		drone.SpeculationTarget = [][3]float64{}
		if drone.mATCTicket != nil {
			hub.mATC.cancelTicket(drone.mATCTicket)
		}

	}

}

func (hub *DroneHub) Clear(n int) {
	hub.mutex.Lock()
	defer hub.mutex.Unlock()

	drone := hub.Drones[n]
	hub.DroneReady[n] = false
	drone.TargetLocation = drone.TargetLocation[1:]
	drone.Phase = 0 
}


func (hub *DroneHub) SetWayPath(n int, loc [][3]float64) chan int {
	hub.mutex.Lock()
	defer hub.mutex.Unlock()

	drone := hub.Drones[n]
	drone.UserSpaceWayPath = loc

	drone.reachWayPath = make(chan int,len(loc))


	return drone.reachWayPath
}

func (hub *DroneHub) SetReachTargetCallBack(callback chan int) {
	hub.callback = callback
}

func (hub *DroneHub) Sense(n int, name string) interface{} {
	hub.mutex.Lock()
	//defer sim.mutex.Unlock()

	if name == "GetTime" {
		hub.mutex.Unlock()  
		return hub.TimeFromStart
	}

	if name == "GetDuration" {
		hub.mutex.Unlock()  
		return hub.LastActionTS - hub.FirstActionTS
	}


	retchan := make(chan interface{},1)

	hub.Drones[n].Actions = append(hub.Drones[n].Actions, name)
	hub.Drones[n].ActionReturn = append(hub.Drones[n].ActionReturn, retchan)

	hub.mutex.Unlock() 

	ret := <- retchan 

	return ret 
}



func (hub *DroneHub) FlyingTimeEst(n int, loc1 [3]float64, loc2 [3]float64) float64 {
	t0 := hub.Drones[n].CurrentSpeed/hub.Drones[n].AccRate

	d := distance(loc1, loc2) + t0*t0*hub.Drones[n].AccRate // 

	t:= hub.Drones[n].FlyingSpeed / hub.Drones[n].AccRate

	if d > t*t * hub.Drones[n].AccRate {
		return (d-t*t * hub.Drones[n].AccRate)/hub.Drones[n].FlyingSpeed + t*2
	} else {

		return math.Sqrt(d / hub.Drones[n].AccRate) * 2
	}

}


func (hub *DroneHub) GetTime() float64 {
	return float64(makeTimestamp() - hub.BaseTime)/1000.0
}
	
func (hub *DroneHub) SetTriggerCMP(time_tri float64, time_cmp float64, callback chan float64) {
	hub.mutex.Lock()
	defer hub.mutex.Unlock()

	if time_cmp > hub.TimeTriggerTime {
		hub.TimeTriggerTime = time_tri 
		hub.TimeTrigger = callback 
	}
}


func (hub *DroneHub) ClockThread() {
	for {
		hub.mutex.Lock()

		hub.TimeFromStart = hub.GetTime()

		if hub.TimeFromStart > hub.TimeTriggerTime && hub.TimeTriggerTime > 1.0 {
			
			hub.TimeTriggerTime = 0.0
			hub.TimeTrigger <- hub.TimeFromStart
		}

		hub.mutex.Unlock()
		time.Sleep(time.Duration(100) * time.Microsecond)
	}
}

func (hub *DroneHub) RemainingFlyingTime(n int) float64 {
	// not implemented

	return 1000.0
}
