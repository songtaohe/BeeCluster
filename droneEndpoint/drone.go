package main

import (
	"bytes"
	"encoding/json"
	"encoding/base64"
	"io/ioutil"
	"log"
	"net/http"
	"time"
	"strings"
	"strconv"
	"fmt"
	"os"
	"sync"
	"os/exec"
	"math/rand"
	"bufio"
	)

const (
	ATC_HOLD = 0
	ATC_PROCEED = 1 
	ATC_PROCEED_WITH_CONTROL_POINTS = 2 
	ATC_HOLD_AT_POSITION = 3 	
)


func makeTimestamp() int64 {
	return time.Now().UnixNano() / int64(time.Millisecond)
}


type DroneToHub struct {
	DroneID 	int  `json"id"`   // need to be unique

	LocXYZ	[3]float64 `json"locxyz"`
	LocGPS	[3]float64 `json"locgps"`

	Battery	float64 `json"battery"`

	RawInfo string `json"rawinfo"`

	PayLoadDatas 	[]string  `json"payloaddata"` // for sensing data such as image 
	PayLoadUIDs		[]int `json"payloaduid"` 

	TransmissionTime float64 
	ActionTime float64 

}


type HubToDrone struct {
	Local2GlobalBias	[3]float64 `json"local2globalbias"`			


	Actions 		[]string `json"action"` 
	ActionUIDs 		[]int `json"actionUID"`
}


func Post(url string, data interface{}) *HubToDrone {
	requestBody, err := json.Marshal(data)

	if err != nil {
		log.Println(err)
		return nil
	}


	resp, err := http.Post(url, "application/json", bytes.NewBuffer(requestBody))

	if err != nil {
		log.Println(err)
		return nil
	}

	defer resp.Body.Close()

	body, err := ioutil.ReadAll(resp.Body)
	if err != nil {
		log.Println(err)
		return nil
	}

	ret := new(HubToDrone)
	json.Unmarshal(body, ret)
	return ret 

}
type atc_solution struct {
	solution_id 	int 
	wp1 			[3]float64
	wp2				[3]float64
	hp 				[3]float64


}

type phyDrone struct {
	wrapper *DJI_Wrapper
	droneID int 
	url string 

	targetLoc [3]float64
	currentLoc [3]float64

	time_start int64

	mutex	sync.Mutex
	flying2target bool 

	yaw float64

	atc *atc_solution
}


func (drone *phyDrone) flyToTarget() {
	drone.flying2target = true 

	if drone.atc == nil {
		drone.wrapper.FlyToXYZYaw(drone.targetLoc[0], drone.targetLoc[1], drone.targetLoc[2], drone.yaw, 10)
	} else {
		switch drone.atc.solution_id {
		case ATC_PROCEED :
			drone.wrapper.FlyToXYZYaw(drone.targetLoc[0], drone.targetLoc[1], drone.targetLoc[2], drone.yaw, 10)
		
		case ATC_PROCEED_WITH_CONTROL_POINTS:

			d1 := distanceXYZ(drone.currentLoc, drone.atc.wp2)
			d2 := distanceXYZ(drone.atc.wp1, drone.atc.wp2)
			//if d2 < d1 + 1.0 + math.Abs(drone.currentLoc[2] - drone.wp1[2]) {
			if d2 < d1 {
				drone.wrapper.FlyToXYZYawFastNoBrake(drone.atc.wp1[0], drone.atc.wp1[1], drone.atc.wp1[2], drone.yaw, 10)
			}

			if drone.wrapper.isFlyingEnabled == true {

				d1 := distanceXYZ(drone.currentLoc, drone.targetLoc)
				d2 := distanceXYZ(drone.atc.wp2, drone.targetLoc)

				if d2 < d1 {
					drone.wrapper.FlyToXYZYawFastNoBrake(drone.atc.wp2[0], drone.atc.wp2[1], drone.atc.wp2[2], drone.yaw, 10)
				}

				if drone.wrapper.isFlyingEnabled == true {
					drone.wrapper.FlyToXYZYaw(drone.targetLoc[0], drone.targetLoc[1], drone.targetLoc[2], drone.yaw, 10)
				}
			}
		case ATC_HOLD_AT_POSITION:
			drone.wrapper.FlyToXYZYaw(drone.atc.hp[0], drone.atc.hp[1], drone.atc.hp[2], drone.yaw, 10)
		}
	}
	//drone.mutex.Lock() 
	drone.flying2target = false
	//drone.mutex.Unlock()
}



func (drone *phyDrone) updateTarget(loc [3]float64, atc *atc_solution) {
	d := distanceXYZ(loc, drone.targetLoc)
	

	if d < 0.5 {

		dr := distanceXYZ(loc, drone.currentLoc)

		var need_abort bool = false 

		if drone.atc != nil && atc != nil {
			if drone.atc.solution_id != atc.solution_id {
				need_abort = true
			}
		}

		if need_abort == false{
			if dr > 0.9 && drone.flying2target == false{
				drone.targetLoc = loc 
				drone.flying2target = true 
				drone.atc = atc 

				go drone.flyToTarget()
			}
			return 
		}
	}

	drone.targetLoc = loc 
	drone.atc = atc 

	drone.wrapper.Abort()

	// wait until drone.flying2target == false 
	for {
		time.Sleep(time.Duration(1 * time.Millisecond))
		
		if drone.flying2target == false {
			break
		}
	} 



	d = distanceXYZ(loc, drone.currentLoc)
	if d < 0.5 {
		return 
	}

	drone.flying2target = true 
	go drone.flyToTarget()
}

func (drone *phyDrone) Log2File(status Drone) string{
	logfile := fmt.Sprintf("drone_log.txt")

	f, err := os.OpenFile(logfile, os.O_APPEND|os.O_RDWR|os.O_CREATE, 0666)

	var ret string = ""

	if err == nil {
		timestamp := float64(makeTimestamp()-drone.time_start) / 1000.0

		signal, _ := exec.Command("sh", "-c", "iwconfig wlan0 | grep Link").Output()

		ret = fmt.Sprintf("%.3f |%.8f %.8f %.8f |%.3f %.3f %.3f |%.3f %.3f |%.3f|%s\n",
			timestamp, status.GPS[0], status.GPS[1], status.GPS[2],
			status.XYZ[0], status.XYZ[1], status.XYZ[2],
			status.BatteryVoltage, status.Battery,
			status.Yaw, signal)
		
		ret = strings.Replace(ret, "\n", "", -1)
		ret += "\n"

		f.WriteString(ret)

		f.Close()

	}

	return ret 
}

// write to flight logs 
func (drone *phyDrone) loop() {
	var msg DroneToHub

	var takeoff bool=false

	bias := [3]float64{0.0, 0.0, 0.0}

	for {
		
		t_start := makeTimestamp()

		msg.DroneID = drone.droneID

		if takeoff == true {
			status := drone.wrapper.GetStatus()

			msg.LocXYZ[0] = status.XYZ[0] + bias[0]
			msg.LocXYZ[1] = status.XYZ[1] + bias[1]
			msg.LocXYZ[2] = status.XYZ[2] + bias[2]// - float64(drone.droneID) * 2.0
			msg.Battery = float64(status.Battery)

			drone.currentLoc[0] = status.XYZ[0]
			drone.currentLoc[1] = status.XYZ[1]
			drone.currentLoc[2] = status.XYZ[2]

			msg.RawInfo = drone.Log2File(status)
		}

		tpost := makeTimestamp()
		ret := Post(drone.url, msg)
		fmt.Println("Post time: ", makeTimestamp() - tpost)


		msg.TransmissionTime = float64(makeTimestamp() - tpost) / 1000.0

		
		msg.ActionTime = 0.0 

		if ret != nil {

			if takeoff == false {
				drone.wrapper.Takeoff()
				takeoff = true

				drone.updateTarget([3]float64{0.0,0.0,5.0}, nil)

				time.Sleep(time.Duration(int(2000)) * time.Millisecond)
			}

			fmt.Println(ret)

			if len(ret.Actions) == 0 {
				continue 
			}

			bias = ret.Local2GlobalBias
			fmt.Println(bias)

			if strings.HasPrefix(ret.Actions[0], "flyto") {
				items := strings.Split(ret.Actions[0], "_")

				x, _ := strconv.ParseFloat(items[1], 64)
				y, _ := strconv.ParseFloat(items[2], 64)
				z, _ := strconv.ParseFloat(items[3], 64)

				x = x - bias[0]
				y = y - bias[1]
				z = z - bias[2]

				//z = z + float64(drone.droneID) * 2.0

				//fmt.Println("flyto", x, y, z)

				var atc *atc_solution = nil 

				new_items := strings.Split(ret.Actions[0], "atc_")
				if len(new_items) > 1 {
					atc_str := new_items[1]

					items := strings.Split(atc_str, "_")

					atc = new(atc_solution)
					atc.solution_id, _ = strconv.Atoi(items[0])

					atc.wp1[0], _ = strconv.ParseFloat(items[2], 64)
					atc.wp1[1], _ = strconv.ParseFloat(items[3], 64)
					atc.wp1[2], _ = strconv.ParseFloat(items[4], 64)

					atc.wp2[0], _ = strconv.ParseFloat(items[6], 64)
					atc.wp2[1], _ = strconv.ParseFloat(items[7], 64)
					atc.wp2[2], _ = strconv.ParseFloat(items[8], 64)

					atc.hp[0], _ = strconv.ParseFloat(items[10], 64)
					atc.hp[1], _ = strconv.ParseFloat(items[11], 64)
					atc.hp[2], _ = strconv.ParseFloat(items[12], 64)

					for i:=0; i<3; i++ {
						atc.wp1[i] = atc.wp1[i] - bias[i]
						atc.wp2[i] = atc.wp2[i] - bias[i]
						atc.hp[i] = atc.hp[i] - bias[i]
							
					}

				}


				drone.updateTarget([3]float64{x,y,z}, atc)


			} else if strings.HasPrefix(ret.Actions[0], "standby") {


				drone.updateTarget(drone.currentLoc, nil)
			}

			msg.PayLoadDatas = []string{}
			msg.PayLoadUIDs = []int{}

			if len(ret.Actions) > 1 {
				taction := makeTimestamp()

				for ind, act := range ret.Actions[1:] {
					fmt.Println("action", act)

					
					if act == "measure_wifi_signal" {

						if drone.wrapper.mode == MODE_SIMULATOR  {
							//d := distanceXYZ(drone.currentLoc, [3]float64{15.0, -10.0, 0.0})

							d := 1000.0 

							d1 := distanceXYZ(drone.currentLoc, [3]float64{30.0, 0, 0.0})
							d2 := distanceXYZ(drone.currentLoc, [3]float64{50.0, 40, 0.0})
							d3 := distanceXYZ(drone.currentLoc, [3]float64{10.0, -40, 0.0})


							if d > d1 {
								d = d1 
							}

							if d > d2 {
								d = d2 
							}

							if d > d3 {
								d = d3
							}

							if d < 15.0 {
								d = 15.0 + (15.0 - d) * 1.0 
							}

							lookuptable := [20]float64{-24.1, -27.9, -31.3, -34.6, -36.4,
													   -42.3, -43.1, -48.8, -50.2, -54, 
													   -54.6, -59.1, -57.0, -56.4, -58.2,
													   -59.0, -60.0, -61.6, -64.2, -66.2}



							ind := int(d/5.0)
							if ind >= 19 {
								ind = 18
							} 

							alpha := (d - float64(ind) * 5.0) / 5.0 

							signal := int((lookuptable[ind+1] - lookuptable[ind]) * alpha + lookuptable[ind] + rand.Float64() * 6.0 - 3.0)

							//msg.PayLoadDatas = append(msg.PayLoadDatas, msg.RawInfo)
							msg.PayLoadDatas = append(msg.PayLoadDatas, msg.RawInfo + fmt.Sprintf("         Link Quality=70/70  Signal level=%d dBm ", signal) )
						} else {
							msg.PayLoadDatas = append(msg.PayLoadDatas, msg.RawInfo)
						}

						

						msg.PayLoadUIDs = append(msg.PayLoadUIDs, ret.ActionUIDs[ind])

					} else if act == "scan_wifi_signal" {
						signal, _ := exec.Command("sh", "-c", "sudo iwlist wlan0 scan | grep \"Signal\\|ESSID\"").Output()
						msg.PayLoadDatas = append(msg.PayLoadDatas, string(signal))
						msg.PayLoadUIDs = append(msg.PayLoadUIDs, ret.ActionUIDs[ind])
					
					} else if act == "turn_on_infrared" {
						
						_ = exec.Command("curl", "http://127.0.0.1:8003").Run()
						
						msg.PayLoadDatas = append(msg.PayLoadDatas, "ack")
						msg.PayLoadUIDs = append(msg.PayLoadUIDs, ret.ActionUIDs[ind])

					} else if act == "turn_off_infrared" {

						_ = exec.Command("curl", "http://127.0.0.1:8004").Run()
						
						msg.PayLoadDatas = append(msg.PayLoadDatas, "ack")
						msg.PayLoadUIDs = append(msg.PayLoadUIDs, ret.ActionUIDs[ind])

					} else if act == "take_photo" || act == "take_photo_fast" {
						//c := exec.Command("raspistill", "-n", "-w", "640", "-h", "480", "-o", "tmpimg.jpg", "--timeout", "3000")
						
						//tt1 := makeTimestamp()

						cameraurl := "http://127.0.0.1:8002"
						if act == "take_photo_fast" {
							cameraurl = "http://127.0.0.1:8001"
						}
						c := exec.Command("curl", cameraurl)
						err := c.Run()


						//fmt.Println("Debug take photo time marker 1", makeTimestamp()-tt1 )
						
						if err != nil {
							fmt.Println(err)
							log.Printf("[Drone endpoint] Failed to run raspistill")
							
							msg.PayLoadDatas = append(msg.PayLoadDatas, "action failed [Drone endpoint] Failed to run raspistill")
							msg.PayLoadUIDs = append(msg.PayLoadUIDs, ret.ActionUIDs[ind])
						} else {

							//
							imgFile, err := os.Open("tmpimg.jpg")
							if err != nil {
								log.Printf("[Drone endpoint] Failed to take photo")

								msg.PayLoadDatas = append(msg.PayLoadDatas, "action failed [Drone endpoint] Failed to take photo")
								msg.PayLoadUIDs = append(msg.PayLoadUIDs, ret.ActionUIDs[ind])
						
							} else {


								fInfo, _ := imgFile.Stat()
								var size int64 = fInfo.Size()
								buf := make([]byte, size)

								// read file content into buffer
								fReader := bufio.NewReader(imgFile)
								fReader.Read(buf)

								//fmt.Println("Debug take photo time marker 2", makeTimestamp()-tt1 )
						

								image_str := base64.StdEncoding.EncodeToString(buf)

								//fmt.Println("Debug take photo time marker 3", makeTimestamp()-tt1 )
						
								//fmt.Println(image_str)

								imgFile.Close()

								msg.PayLoadDatas = append(msg.PayLoadDatas, msg.RawInfo + "|" + image_str)
								msg.PayLoadUIDs = append(msg.PayLoadUIDs, ret.ActionUIDs[ind])
						
							}
						}

					} else if strings.HasPrefix(act, "set_yaw") {

						items := strings.Split(act, "_")

						drone.yaw, _ = strconv.ParseFloat(items[2], 64)

						fmt.Println("Set yaw to ", drone.yaw, "in effect after next flyto")

						msg.PayLoadDatas = append(msg.PayLoadDatas, "action ack")
						msg.PayLoadUIDs = append(msg.PayLoadUIDs, ret.ActionUIDs[ind])
					} else {
						msg.PayLoadDatas = append(msg.PayLoadDatas, "action ack")
						msg.PayLoadUIDs = append(msg.PayLoadUIDs, ret.ActionUIDs[ind])
					}

				}
				msg.ActionTime = float64(makeTimestamp() - taction)/1000.0 
			}
		}

		
		t_now := makeTimestamp()

		if t_start + 100 > t_now {
			time.Sleep(time.Duration(int(t_start+100 - t_now)) * time.Millisecond)
		}
	}
}


func main() {
	drone := new(phyDrone) 

	if len(os.Args) > 2 {

		wrapper := MakeDJIWrapper(MODE_SIMULATOR)
		drone.wrapper = wrapper

		if os.Args[2] == "onboard_sim" {
			drone.url = "http://192.168.0.1:7900"
		} else if os.Args[2] == "local" {
			drone.url = "http://localhost:7900"
		} else {
			drone.url = "http://192.168.1.21:7900"
		}

	} else {
		wrapper := MakeDJIWrapper(MODE_ON_BOARD_SDK)
		drone.wrapper = wrapper
		drone.url = "http://192.168.0.1:7900"
	}
	//

	
	
	drone.flying2target = false 
	drone.droneID, _ = strconv.Atoi(os.Args[1])
	
	drone.yaw = 0.0 
	drone.time_start = makeTimestamp()

	drone.loop()
}