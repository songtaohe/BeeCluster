package main


/*
#cgo LDFLAGS: -ldl
#include <stdbool.h>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>


void * dlHandle = NULL;
char * buffer = NULL;

bool (*Init_func)(int);
bool (*GetStatus_func)(char *);
bool (*Takeoff_func)();
bool (*Land_func)();
bool (*FlyToPositionLatLon_func)(float, float, float, float);
bool (*FlyToPositionXYZ_func)(float, float, float, float, int, float);
bool (*FlyToPositionXYZFastNoBrake_func)(float, float, float, float, int, float);
bool (*ChangeYaw_func)(float);
void (*SetEnableFlying_func)(int);

void setup(char * filename) {
	dlHandle = dlopen(filename, RTLD_NOW);

	Init_func = dlsym(dlHandle, "Init");
	GetStatus_func = dlsym(dlHandle, "GetStatus");
	Takeoff_func = dlsym(dlHandle, "Takeoff");
	Land_func = dlsym(dlHandle, "Land");
	FlyToPositionXYZ_func = dlsym(dlHandle, "FlyToPositionXYZ");
	FlyToPositionXYZFastNoBrake_func = dlsym(dlHandle, "FlyToPositionXYZFastNoBrake");
	ChangeYaw_func = dlsym(dlHandle, "ChangeYaw");
	SetEnableFlying_func = dlsym(dlHandle, "SetEnableFlying");
	

	buffer = (char*)malloc(1024);
}



bool Init(int timeout)
{
	return Init_func(timeout);
}
char * GetStatus()
{
	GetStatus_func(buffer);

	return buffer;
}

bool Takeoff()
{
	return Takeoff_func();
}
bool Land()
{
	return Land_func();
}

bool FlyToPositionXYZ(float x, float y, float z, float yaw, int timeout, float speed)
{
	return FlyToPositionXYZ_func(x,y,z,yaw,timeout,speed);
}

bool FlyToPositionXYZFastNoBrake(float x, float y, float z, float yaw, int timeout, float speed)
{
	return FlyToPositionXYZFastNoBrake_func(x,y,z,yaw,timeout,speed);
}

bool ChangeYaw(float yaw)
{
	return ChangeYaw_func(yaw);
}

void SetEnableFlying(int flag)
{
	SetEnableFlying_func(flag);
	
}
*/
import "C"


import (
	//"../../common"
	"log"
	//"../../simulator"
	"sync"
	"fmt"
	"strings"
	"strconv"
	"math"
	"time"
	"os/exec"
	"bufio"
	"encoding/base64"
	"os"
	"bytes"
	"encoding/json"
)


var default_height  float64 = 20.0
var default_yaw 	float64 = 0.0
var default_speed   float64 = 10.0

var heading_fix 	float32 = 0.0 // F450, this should be a parameter for each drone. 


const (
	MODE_ON_BOARD_SDK = 0
	MODE_JOYSTICK   = 1
	MODE_SIMULATOR	  = 2
)

const (
	DJI_SUCCESS = 0
	DJI_ERROR_PERMISSION_DENIED = 1
	DJI_ERROR_LOW_PRIORITY = 2
)



// type DJI_Context struct {
// 	waypath 			[]simulator.Position
// 	interruptPosition	simulator.Position
// } // for context switch




type DJI_Wrapper struct {
	mode int // MODE_ON_BOARD_SDK / MODE_MOBILE_SDK / MODE_SIMULATOR
	simulator *Simulator

	current_priority	int 

	mux  			sync.Mutex
	get_status_mux  sync.Mutex 


	waypath_mux     sync.Mutex
	waypath         [][5]float64  // lat,lon,altitude,yaw,speed  
	waypath_flying	bool 
	waypoint_current [5]float64

	isFlyingEnabled  bool

	s_status  string

	initial_location [2]float64
}


func distanceXYZ(p1 [3]float64, p2 [3]float64) float64 {
	d0 := p1[0] - p2[0]
	d1 := p1[1] - p2[1]
	d2 := p1[2] - p2[2]
	return math.Sqrt(d0*d0 + d1*d1 + d2*d2)
}

func distanceGPS(p1 [3]float64, p2 [3]float64) float64 {
	d0 := (p1[0] - p2[0]) * 111111.0
	d1 := (p1[1] - p2[1]) * 111111.0 / math.Cos(p1[0]/180.0*3.1415926)
	d2 := p1[2] - p2[2]
	return math.Sqrt(d0*d0 + d1*d1 + d2*d2)
}

func gps2xyz(p1 [3]float64, p2 [3]float64) [3]float64 {
	d0 := (p2[0] - p1[0]) * 111111.0
	d1 := (p2[1] - p1[1]) * 111111.0 * math.Cos(p1[0]/180.0*3.1415926)
	d2 := p2[2] - p1[2]

	return [3]float64{d0,d1,d2}
}

func xyz2gps(p1 [3]float64, p2 [3]float64) [3]float64 {
	d0 := p1[0] + 1.0/111111.0 * p2[0]
	d1 := p1[1] + 1.0/111111.0 / math.Cos(p1[0]/180.0*3.1415926) * p2[1]
	d2 := p1[2] + p2[2]

	return [3]float64{d0,d1,d2}
}

func max(a float64, b float64) float64 {
	if a > b {
		return a
	} else {
		return b 
	}
}

func min(a float64, b float64) float64 {
	if a > b {
		return b
	} else {
		return a
	}
}





func MakeDJIWrapper(mode int) *DJI_Wrapper{
	w := new(DJI_Wrapper)

	w.mode = mode
	w.current_priority = 0 

	//w.simulator = simulator

	if w.mode == MODE_ON_BOARD_SDK {
	  //C.setup(C.CString("/home/pi/drone_OSDK/build/sample/linux/DJILib/libdjiosdk-dronesystem-wrapper-lib.so"));
	  C.setup(C.CString("/home/pi/drone_OSDK/sample/linux/DJILib/libdjiosdk-dronesystem-wrapper-lib.so"));
	  
	  for C.Init(2) == false {
	  	time.Sleep(time.Duration(2000) * time.Millisecond)
	  	fmt.Println("initialization failed, retry in 5 seconds")
	  }


	} else if w.mode == MODE_SIMULATOR {
		w.simulator = MakeSimulator([3]float64{0.0, 0.0, 0.0})
	}

	w.s_status = ""
	for i:=0; i< 32; i++ {
		w.s_status = w.s_status + "||||||||||||||||"
	}


	// set initial location for simulator 
	status := w.GetStatus()

	w.initial_location[0] = status.GPS[0]
	w.initial_location[1] = status.GPS[1]



	w.waypath_flying = false
	w.isFlyingEnabled = true 

	go w.followWayPathRoutine()

	return w
}


func (wrapper * DJI_Wrapper) followWayPathRoutine() {

	for {
		time.Sleep(time.Duration(100) * time.Millisecond)
		
		var lat,lon,altitude,yaw,speed float64
		var needFly bool = false 

		wrapper.waypath_mux.Lock()
			if len(wrapper.waypath) > 0 {
				lat = wrapper.waypath[0][0]
				lon = wrapper.waypath[0][1]
				altitude = wrapper.waypath[0][2]
				yaw = wrapper.waypath[0][3]
				speed = wrapper.waypath[0][4]


				wrapper.waypoint_current[0] = lat
				wrapper.waypoint_current[1] = lon
				wrapper.waypoint_current[2] = altitude
				wrapper.waypoint_current[3] = yaw
				wrapper.waypoint_current[4] = speed


				//fmt.Println(wrapper.waypath)

				needFly = true 

				// remove the first item 
				wrapper.waypath = wrapper.waypath[1:]

				wrapper.waypath_flying = true


				C.SetEnableFlying(C.int(1))
				wrapper.isFlyingEnabled = true 


			} else {
				wrapper.waypath_flying = false

				wrapper.waypoint_current[0] = 0.0
				wrapper.waypoint_current[1] = 0.0
				wrapper.waypoint_current[2] = 0.0
				wrapper.waypoint_current[3] = 0.0
				wrapper.waypoint_current[4] = 0.0

			}
		wrapper.waypath_mux.Unlock()

		// block operation
		if needFly == true {
			yaw_ := wrapper.PointingHomeYaw()

			wrapper.FlyToGPSYaw(lat, lon, altitude, yaw_, speed)

			wrapper.PointingHome()
		}
	}
}

func (wrapper * DJI_Wrapper) Abort() {
	if wrapper.mode == MODE_ON_BOARD_SDK{
		C.SetEnableFlying(C.int(0))
	} else if wrapper.mode == MODE_SIMULATOR {
		wrapper.simulator.SetWayPath([][3]float64{})

	}
	wrapper.isFlyingEnabled = false 
}


func (wrapper * DJI_Wrapper) setWayPath(waypath [][5]float64) {
	wrapper.waypath_mux.Lock()
	defer wrapper.waypath_mux.Unlock()

	

	if len(wrapper.waypath) > 0 || wrapper.waypath_flying == true {

		//fmt.Println(wrapper.waypath, waypath, wrapper.waypoint_current, wrapper.waypath_flying)

		if len(waypath) >= 1 {
			if math.Abs(waypath[0][0] - wrapper.waypoint_current[0]) < 0.000001 &&
			math.Abs(waypath[0][1] - wrapper.waypoint_current[1]) < 0.000001 &&
			math.Abs(waypath[0][2] - wrapper.waypoint_current[2]) < 0.000001 &&
			math.Abs(waypath[0][3] - wrapper.waypoint_current[3]) < 0.000001 &&
			math.Abs(waypath[0][4] - wrapper.waypoint_current[4]) < 0.000001 {

				//don't need to abort the current action
				if len(waypath) > 1 {
					wrapper.waypath = waypath[1:]
				}

				return
			}
		}

		// clear the waypath
		// wrapper.waypath = [:]
		fmt.Println("abort current action")
		// abort current operation 
		wrapper.Abort()
	}

	wrapper.waypath = waypath
}

func (wrapper * DJI_Wrapper) IsFlying() bool {
	return wrapper.waypath_flying
}

// This is a asynchronized function call. 
func (wrapper * DJI_Wrapper) FollowWaypath(waypath []float64, priority int) int {
	//wrapper.mux.Lock()
	//defer wrapper.mux.Unlock()

	if priority < wrapper.current_priority {
		log.Printf("[DJI Wrapper] Priority")

		return DJI_ERROR_LOW_PRIORITY 
	} else {
		switch wrapper.mode {
		case MODE_ON_BOARD_SDK:

			waypath_ := make([][5]float64, len(waypath)/2)

			for i:=0; i<len(waypath)/2;i++{
				waypath_[i][0] = waypath[i*2]
				waypath_[i][1] = waypath[i*2+1]
				waypath_[i][2] = default_height
				waypath_[i][3] = default_yaw
				waypath_[i][4] = default_speed
			}

			wrapper.setWayPath(waypath_)

			// for i:=0; i<len(waypath)/2;i++{
			// 	wrapper.FlyToGPSYaw(waypath[i*2], waypath[i*2+1], default_height, default_yaw, default_speed)
			// }


		case MODE_JOYSTICK:

		case MODE_SIMULATOR:

			// waypath_ := make([]simulator.Position, len(waypath)/2)

			// for i:=0; i<len(waypath)/2;i++{
			// 	waypath_[i].Lat = waypath[i*2]
			// 	waypath_[i].Lon = waypath[i*2+1]
			// }

			// wrapper.simulator.SetWayPath(waypath_)

			// log.Printf("[DJI Wrapper : SIMULATOR] Set Waypath, num %d", len(waypath)/2)

		}

	}

	return DJI_SUCCESS
}



// This is a asynchronized function call. 
func (wrapper * DJI_Wrapper) FollowWaypath3D(waypath [][5]float64, priority int) int {
	//wrapper.mux.Lock()
	//defer wrapper.mux.Unlock()

	if priority < wrapper.current_priority {
		log.Printf("[DJI Wrapper] Priority")

		return DJI_ERROR_LOW_PRIORITY 
	} else {
		switch wrapper.mode {
		case MODE_ON_BOARD_SDK:

			wrapper.setWayPath(waypath)

			// for i:=0; i<len(waypath)/2;i++{
			// 	wrapper.FlyToGPSYaw(waypath[i*2], waypath[i*2+1], default_height, default_yaw, default_speed)
			// }


		case MODE_JOYSTICK:

		case MODE_SIMULATOR:

			// waypath_ := make([]simulator.Position, len(waypath)/2)

			// for i:=0; i<len(waypath)/2;i++{
			// 	waypath_[i].Lat = waypath[i][0]
			// 	waypath_[i].Lon = waypath[i][1]
			// }

			// wrapper.simulator.SetWayPath(waypath_)

			// log.Printf("[DJI Wrapper : SIMULATOR] Set Waypath, num %d", len(waypath)/2)

		}

	}

	return DJI_SUCCESS
}


// Only USE this function!
// Other functions may not be safe
// This is a synchronized function call. 
func (wrapper * DJI_Wrapper) FlyToXYZYaw(x float64, y float64, z float64, yaw float64, speed float64) {
	wrapper.mux.Lock()
	defer wrapper.mux.Unlock()

	wrapper.isFlyingEnabled = true 

	status := wrapper.GetStatus()

	dist_remaining := distanceXYZ(status.XYZ, [3]float64{x,y,z})
	var action_counter int = 0

	switch wrapper.mode {
		case MODE_ON_BOARD_SDK:	
			for (dist_remaining > 1.0 || action_counter == 0 ) && wrapper.isFlyingEnabled == true{
				time_reserved := max(5.0, max(0.0,dist_remaining-20.0)/speed+min(dist_remaining, 20.0)/2.5+5.0)


				fmt.Println(time_reserved, C.float(x), C.float(y), C.float(z), status.XYZ)

				C.FlyToPositionXYZ(C.float(x), C.float(y), C.float(z), C.float(yaw+float64(heading_fix)), C.int(time_reserved), C.float(speed))



				status = wrapper.GetStatus()
				dist_remaining = distanceXYZ(status.XYZ, [3]float64{x,y,z})

				action_counter += 1

				// re-try at most three times 
				if action_counter > 3 {
					break
				}
			}

		case MODE_JOYSTICK:


		case MODE_SIMULATOR:

			wrapper.simulator.SetWayPath([][3]float64{[3]float64{x,y,z}})

			for (dist_remaining > 1.0 || action_counter == 0) && wrapper.isFlyingEnabled == true{
				time.Sleep(time.Duration(100) * time.Millisecond)

				status = wrapper.GetStatus()
				//status.XYZ[2] = 0.0

				dist_remaining = distanceXYZ(status.XYZ, [3]float64{x,y,z})

				//fmt.Println(dist_remaining)
				action_counter += 1
			}

			wrapper.simulator.Yaw = yaw 


	}
}

// Only USE this function!
// Other functions may not be safe
// This is a synchronized function call. 
// Be super careful when use this function! NO BRAKE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
func (wrapper * DJI_Wrapper) FlyToXYZYawFastNoBrake(x float64, y float64, z float64, yaw float64, speed float64) {
	wrapper.mux.Lock()
	defer wrapper.mux.Unlock()

	wrapper.isFlyingEnabled = true 

	status := wrapper.GetStatus()

	dist_remaining := distanceXYZ(status.XYZ, [3]float64{x,y,z})
	var action_counter int = 0

	switch wrapper.mode {
		case MODE_ON_BOARD_SDK:	
			for (dist_remaining > 2.0 || action_counter == 0 ) && wrapper.isFlyingEnabled == true{
				time_reserved := max(5.0, max(0.0,dist_remaining-20.0)/speed+min(dist_remaining, 20.0)/2.5+5.0)


				fmt.Println(time_reserved, C.float(x), C.float(y), C.float(z), status.XYZ)

				C.FlyToPositionXYZFastNoBrake(C.float(x), C.float(y), C.float(z), C.float(yaw+float64(heading_fix)), C.int(time_reserved), C.float(speed))


				status = wrapper.GetStatus()
				dist_remaining = distanceXYZ(status.XYZ, [3]float64{x,y,z})

				action_counter += 1

				// re-try at most three times 
				if action_counter > 3 {
					break
				}
			}

		case MODE_JOYSTICK:


		case MODE_SIMULATOR:

			wrapper.simulator.NoBrake = true 
			wrapper.simulator.SetWayPath([][3]float64{[3]float64{x,y,z}})

			for (dist_remaining > 1.0 || action_counter == 0) && wrapper.isFlyingEnabled == true{
				time.Sleep(time.Duration(100) * time.Millisecond)

				status = wrapper.GetStatus()
				//status.XYZ[2] = 0.0

				dist_remaining = distanceXYZ(status.XYZ, [3]float64{x,y,z})

				//fmt.Println(dist_remaining)
				action_counter += 1
			}

			wrapper.simulator.Yaw = yaw 
			wrapper.simulator.NoBrake = false

	}
}



// This is a synchronized function call. 
func (wrapper * DJI_Wrapper) FlyToGPSYaw(lat float64, lon float64, altitude float64, yaw float64, speed float64) {
	wrapper.mux.Lock()
	defer wrapper.mux.Unlock()

	wrapper.isFlyingEnabled = true 

	status := wrapper.GetStatus()

	dist_remaining := distanceGPS(status.GPS, [3]float64{lat,lon,altitude})
	var action_counter int = 0

	switch wrapper.mode {
		case MODE_ON_BOARD_SDK:	
			for dist_remaining > 3.0 || action_counter == 0{
				if wrapper.isFlyingEnabled == false {
					break
				}

				time_reserved := max(5.0, max(0.0,dist_remaining-20.0)/speed+min(dist_remaining, 20.0)/2.5+5.0)

				dXYZ := gps2xyz(status.GPS, [3]float64{lat,lon,altitude})

				//fmt.Println(time_reserved, C.float(status.XYZ[0] + dXYZ[0]), C.float(status.XYZ[1] + dXYZ[1]), C.float(status.XYZ[2] + dXYZ[2]),C.float(yaw), C.int(time_reserved), C.float(speed), status.XYZ)


				C.FlyToPositionXYZ(C.float(status.XYZ[0] + dXYZ[0]), C.float(status.XYZ[1] + dXYZ[1]), C.float(status.XYZ[2] + dXYZ[2]),C.float(yaw + float64(heading_fix)), C.int(time_reserved), C.float(speed))


				status = wrapper.GetStatus()
				dist_remaining = distanceGPS(status.GPS, [3]float64{lat,lon,altitude})

				action_counter += 1
			}

		case MODE_JOYSTICK:


		case MODE_SIMULATOR:
			// wrapper.simulator.SetWayPath([]simulator.Position{simulator.Position{Lat:lat,Lon:lon}})

			// for dist_remaining > 3.0 || action_counter == 0{
			// 	time.Sleep(time.Duration(1000) * time.Millisecond)

			// 	status = wrapper.GetStatus()
			// 	status.GPS[2] = 0.0

			// 	dist_remaining = distanceGPS(status.GPS, [3]float64{lat,lon,0})

			// 	action_counter += 1
			// }
			
	}
}





func (wrapper * DJI_Wrapper) ChangeYaw(yaw float32) {
	wrapper.mux.Lock()
	defer wrapper.mux.Unlock()


	switch wrapper.mode {
		case MODE_ON_BOARD_SDK:	
			yaw_ := yaw + float32(heading_fix)

			for {
				if yaw_ <= 180.0 && yaw_ >= -180.0 {
					break
				}

				if yaw_ > 180.0 {
					yaw_ = yaw_ - 360.0
				}

				if yaw_ < -180.0 {
					yaw_ = yaw_ + 360.0
				}
			}

			C.ChangeYaw(C.float(yaw_))


		case MODE_JOYSTICK:

		case MODE_SIMULATOR:

	}
	
		
}

func (wrapper * DJI_Wrapper) PointingHome() {
	status := wrapper.GetStatus()

	target_yaw := math.Atan2(status.XYZ[1], status.XYZ[0]) / math.Pi * 180.0 + 180.0

	wrapper.ChangeYaw(float32(target_yaw))
}

func (wrapper * DJI_Wrapper) PointingHomeYaw() float64{
	status := wrapper.GetStatus()

	target_yaw := math.Atan2(status.XYZ[1], status.XYZ[0]) / math.Pi * 180.0 + 180.0 

	return target_yaw
}


// return the base64 format image string
func (wrapper * DJI_Wrapper) TakePhoto(filename string) string {
	var image_str string = ""

	switch wrapper.mode {
	case MODE_ON_BOARD_SDK:
		
		c := exec.Command("raspistill", "-n", "-o", filename)

		
		err := c.Run()

		if err != nil {
			fmt.Println(err)
			log.Printf("[DJI Wrapper] Failed to run raspistill")
			return image_str
		}

		//
		imgFile, err := os.Open(filename)
		if err != nil {
			log.Printf("[DJI Wrapper] Failed to take photo")
			return image_str
		}

		defer imgFile.Close()

		fInfo, _ := imgFile.Stat()
		var size int64 = fInfo.Size()
		buf := make([]byte, size)

		// read file content into buffer
		fReader := bufio.NewReader(imgFile)
		fReader.Read(buf)

		image_str = base64.StdEncoding.EncodeToString(buf)

	case MODE_JOYSTICK:

	case MODE_SIMULATOR:

	}

	return image_str
}

type PMData struct {
	Time []float64 `json:"time"`
	PM1 []int `json:"pm1"`
	PM2 []int `json:"pm2"`
	PM10 []int `json:"pm10"`
	Part3 []int `json:"part3"`
	Part5 []int `json:"part5"`
	Part1 []int `json:"part1"`
}

func (d PMData) String() string {
	return fmt.Sprintf("PM 2.5 = %v", d.PM1)
}

func ReadPM(nsamples int, interval float64) PMData {
	var outbuf bytes.Buffer
	var out PMData
	cmd := exec.Command("python", "readpm.py", fmt.Sprintf("%d", nsamples), fmt.Sprintf("%0.2f", interval))
	cmd.Stdout = &outbuf
	cmd.Stderr = os.Stderr

	if err := cmd.Start(); err != nil {
		fmt.Println("error running readpm script")
	}
	if err:= cmd.Wait(); err != nil {
		fmt.Println("error waiting for readpm script")
	}
	if err := json.Unmarshal(outbuf.Bytes(), &out); err != nil {
		fmt.Println("error unmarshaling json to output struct")
	}
	return out
}

func (wrapper *DJI_Wrapper) SamplePM(nsamples int, interval float64) PMData {
	var result PMData
	switch wrapper.mode{
	case MODE_ON_BOARD_SDK:
		result = ReadPM(nsamples, interval)
	case MODE_JOYSTICK:

	case MODE_SIMULATOR:
		result = ReadPM(nsamples, interval)
	}
	return result
}

func (wrapper * DJI_Wrapper) ReadSensors() string {
	var sensor_result string = ""

	switch wrapper.mode {
	case MODE_ON_BOARD_SDK:
		out, err := exec.Command("python", "readsensor.py").Output()
	    
	    if err == nil {
	    	output_str := string(out)
	    	sensor_result = output_str
	    } else {
	    	fmt.Println("cannot run read sensor data", err)
	    }

	case MODE_JOYSTICK:

	case MODE_SIMULATOR:

	}

	return sensor_result
}


func (wrapper * DJI_Wrapper) Takeoff() {
	wrapper.mux.Lock()
	defer wrapper.mux.Unlock()


	switch wrapper.mode {
	case MODE_ON_BOARD_SDK:
		C.Takeoff()
	case MODE_JOYSTICK:

	case MODE_SIMULATOR:
	}
}

func (wrapper * DJI_Wrapper) Land() {
	wrapper.mux.Lock()
	defer wrapper.mux.Unlock()


	switch wrapper.mode {
	case MODE_ON_BOARD_SDK:
		C.Land()
	case MODE_JOYSTICK:

	case MODE_SIMULATOR:
	}
}

func (wrapper * DJI_Wrapper) ChangeBattery() {
	switch wrapper.mode {
	case MODE_ON_BOARD_SDK:

	case MODE_JOYSTICK:

	case MODE_SIMULATOR:
		//wrapper.simulator.SetBattery(1.0)
	}
}

func (wrapper * DJI_Wrapper) OnBoard_GetStatusStr() string {  // no need for priority 
	var c_status *C.char = C.GetStatus()

	wrapper.s_status = C.GoString(c_status)
	//fmt.Println(wrapper.s_status)
	return wrapper.s_status
}

type Drone struct {
	ID int `json:"id"`
	//Location Point `json:"location"`
	Battery float32 `json:"battery"` // fraction of available battery, range [0,1]
	GPS    			[3]float64 `json:"gps"`
	XYZ				[3]float64 `json:"xyz"`
	Yaw				float32    `json:"yaw"`
	BatteryVoltage	float32    `json:"batteryVoltage"`

}



func (wrapper * DJI_Wrapper) GetStatus() Drone {  // no need for priority 
	wrapper.get_status_mux.Lock()
	defer wrapper.get_status_mux.Unlock()

	var status Drone

	switch wrapper.mode {
	case MODE_ON_BOARD_SDK:
		status_str := wrapper.OnBoard_GetStatusStr()

		chunks := strings.Split(status_str, "|")

		if len(chunks)<4 {
			return status 
		}

		items := strings.Split(chunks[0], " ")

		// 0: lat, 1: lon, 2: altitude 
		status.GPS[0], _ = strconv.ParseFloat(items[1], 64)
		status.GPS[1], _ = strconv.ParseFloat(items[0], 64)
		status.GPS[2], _ = strconv.ParseFloat(items[2], 64)

		status.GPS[0] = status.GPS[0] / 3.1415926 * 180.0
		status.GPS[1] = status.GPS[1] / 3.1415926 * 180.0

		items = strings.Split(chunks[1], " ")

		status.XYZ[0], _ = strconv.ParseFloat(items[0], 64)
		status.XYZ[1], _ = strconv.ParseFloat(items[1], 64)
		status.XYZ[2], _ = strconv.ParseFloat(items[2], 64)

		// use the z channel as altitude instead of the reading from GPS. 
		status.GPS[2] = status.XYZ[2] 

		items = strings.Split(chunks[2], " ")

		battery_int, _ := strconv.Atoi(items[1])

		status.BatteryVoltage = float32(battery_int) / 1000.0

		if (status.BatteryVoltage > 15.25) {
			status.Battery = (status.BatteryVoltage - 15.25) / 1.25 * (7.5/17.0) + (9.5/17.0)
		} else {
			status.Battery = (status.BatteryVoltage - 14.5) / 0.75 * (9.5/17.0)
		}

		//status.Battery  += 0.2



		//status.Location = common.Point{status.GPS[1], status.GPS[0]}


		items = strings.Split(chunks[4], " ")

		q0, _ := strconv.ParseFloat(items[0], 64)
		q1, _ := strconv.ParseFloat(items[1], 64)
		q2, _ := strconv.ParseFloat(items[2], 64)
		q3, _ := strconv.ParseFloat(items[3], 64)


		q2sqr := q2 * q2 
		t0 := -2.0 * (q2sqr + q3*q3) + 1.0
		t1 := 2.0 * (q1*q2 + q0 * q3)

		status.Yaw = float32(math.Atan2(t1,t0) / math.Pi * 180.0) - heading_fix

		if status.Yaw < -180.0 {
			status.Yaw = status.Yaw  + 360.0
		}



	case MODE_JOYSTICK:

	case MODE_SIMULATOR:

		pos := wrapper.simulator.GetCurrentPosition()

		status.XYZ[0] = pos[0]
		status.XYZ[1] = pos[1]
		status.XYZ[2] = pos[2]

		status.Yaw = float32(wrapper.simulator.Yaw) 

		status.Battery = float32(wrapper.simulator.GetBattery())




		// pos := wrapper.simulator.GetCurrentPosition()
		// battery := wrapper.simulator.GetBattery()
		// status.Location = common.Point{pos.Lon, pos.Lat}
		// status.Battery = float32(battery)
		// status.GPS = [3]float64{pos.Lat, pos.Lon, 0.0}
		// status.BatteryVoltage = status.Battery * 2.0 +  14.80
		// status.Yaw = 0


		//status = fmt.Sprintf("lat:%.6f    lon:%.6f    battery:%.6f\n", pos.Lat, pos.Lon, battery)
		//log.Printf("[DJI Wrapper : SIMULATOR] Get Status %s", status)

	}

	return status
}

