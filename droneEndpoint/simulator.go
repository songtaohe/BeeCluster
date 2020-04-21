package main  

import (
	"sync"
	"math"
	"time"
)


const (
	SIMULATOR_PAUSED = 0
	SIMULATOR_RUNNING = 1 
)

type Simulator struct {
	waypath				[][3]float64
	currentPosition 	[3]float64
	battery				float64
	mux  				sync.Mutex

	state				int 

	batteryDrainRate	float64
	flyingSpeed			float64
	waypathThreshold	float64

	CurrentSpeed 		float64

	NoBrake				bool 

	simulationSpeed 	float64

	gps                 [3]float64 
	xyz                 [3]float64 
	batteryVoltage      float64

	Yaw float64

}

func MakeSimulator(initial_position [3]float64) *Simulator{
	sim := new(Simulator)
	sim.currentPosition = initial_position
	sim.battery = 1.0 // full battery at the very begining

	sim.waypathThreshold = 0.5 // 5 meters

	sim.batteryDrainRate = 0.001
	sim.flyingSpeed = 10
	sim.simulationSpeed = 1.0
	sim.CurrentSpeed = 0.0 

	sim.state = SIMULATOR_RUNNING
	sim.Yaw = 0.0
	sim.NoBrake = false 

	go sim.runLoop(50) // step size is  50 ms 


	return sim 
}


func (sim *Simulator) SetWayPath(waypath [][3]float64){
	sim.mux.Lock()
	defer sim.mux.Unlock()

	sim.waypath = waypath

}


func (sim *Simulator) GetCurrentWayPath() [][3]float64 {
	sim.mux.Lock()
	defer sim.mux.Unlock()

	waypath_ret := make([][3]float64, len(sim.waypath))

	copy(waypath_ret, sim.waypath)

	return waypath_ret 
}


func (sim *Simulator) GetCurrentPosition() [3]float64{
	sim.mux.Lock()
	defer sim.mux.Unlock()

	return sim.currentPosition
}


func (sim *Simulator) GetBattery() float64{
	sim.mux.Lock()
	defer sim.mux.Unlock()

	return sim.battery
}

func (sim *Simulator) SetBattery(v float64){
	sim.mux.Lock()
	defer sim.mux.Unlock()

	sim.battery = v
}

func (sim *Simulator) SetState(state int){
	sim.mux.Lock()
	defer sim.mux.Unlock()

	sim.state = state 
}

// Simulate DJI F450 Drones
func (sim *Simulator) runOneStep(timestep float64) {
	//sim.mux.Lock()
	//defer sim.mux.Unlock()
	//fmt.Println(sim.waypath, sim.currentPosition)

	var acc float64 = 2.8
	var dacc float64 = 3.8 
	var air_r float64 = 0.0006
	var d_tp float64 = 20.0 
	var s_slow float64 = 2.0 

	if sim.NoBrake == true {
		s_slow = 5.0 
	}



	if len(sim.waypath) == 0 { // Hover

 	} else {
		dx := sim.waypath[0][0] - sim.currentPosition[0]
		dy := sim.waypath[0][1] - sim.currentPosition[1]
		dz := sim.waypath[0][2] - sim.currentPosition[2]

		l := 0.0000001 + math.Sqrt(dx*dx + dy*dy + dz*dz)



		air_acc := air_r * sim.CurrentSpeed * sim.CurrentSpeed	


		d := l 

		if d > d_tp {
			if sim.CurrentSpeed < sim.flyingSpeed {
				sim.CurrentSpeed += (acc - air_acc) * timestep
			}
		} else {
			if sim.CurrentSpeed > s_slow {
				sim.CurrentSpeed -= (dacc + air_acc) * timestep
			}

			if sim.CurrentSpeed < s_slow {
				sim.CurrentSpeed += (acc - air_acc) * timestep
			}

			if sim.CurrentSpeed < 0.1 {
				sim.CurrentSpeed = 0.1
			}
		}

		l_fly := timestep * sim.CurrentSpeed

		if l_fly > l {
			l_fly = l 
		}

		sim.currentPosition[0] = sim.currentPosition[0] + dx / l * l_fly
		sim.currentPosition[1] = sim.currentPosition[1] + dy / l * l_fly
		sim.currentPosition[2] = sim.currentPosition[2] + dz / l * l_fly

		l = l - l_fly

		if math.Abs(l) < sim.waypathThreshold{
			sim.waypath = sim.waypath[1:] // Remmove the first waypath

		}

 	}

 	sim.battery = sim.battery - timestep * sim.batteryDrainRate
	if sim.battery < 0.0 {
		sim.battery = 0.0
	}

} 

func (sim *Simulator) runLoop(interval int) {
	for {
		time.Sleep(time.Duration(interval) * time.Millisecond)

		sim.mux.Lock()
		
		if sim.state==SIMULATOR_RUNNING {
			sim.runOneStep(float64(interval)/1000.0 * sim.simulationSpeed) 

		} else {
			// DO nothing
		}

		sim.mux.Unlock()
	}	
}

