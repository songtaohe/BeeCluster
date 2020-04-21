package main 

/*
 * This file implements the DTG replay. 
 * 
 */

import (
	"fmt"
	"sync"
	"encoding/json"
	"io/ioutil"
	"net/http"
	"bytes"
	"time"
	
)


type Replay struct{
	framework *Framework 
	dag *DAG

	global_counter map[string]int 
	global_counter_mutex sync.Mutex 

	global_dag_lock sync.Mutex

	appid string 
	sessionid int 
}

func MakeReplay(dagfilename string, appid string, f *Framework) *Replay {
	r := new(Replay)
	r.dag = LoadDag(dagfilename)[0]

	r.framework = f 

	//_,_ = http.Get("http://localhost:8007/control?cmd=loaddag")


	r.global_counter = make(map[string]int)
	r.appid = appid 

	return r 
}

func (r *Replay) Replay() {
	r.newSession()

	done := make(chan int, 1)
	cancel := make(chan int, 1)

	//fmt.Println(r.dag.Root, len(r.dag.Name2Node))

	for k, v := range r.dag.Name2Node {
		if k == "root" {
			//fmt.Println(k)
			var handle *int = new(int)
			*handle = -1 
			r.ExecuteTask(v, done, cancel, handle)

			//fmt.Println("root returned")
		}
	}
	

	duration := r._getDuration()

	fmt.Println("duration: ", duration)
	d1 := []byte(fmt.Sprintf("%f\n", duration))
    _ = ioutil.WriteFile("duration.txt", d1, 0644)

    r.closeSession()

}


func (r *Replay) IncreaseAndFectchCounter(name string) int{
	var ret int = -1 

	r.global_counter_mutex.Lock() 

	if c, ok := r.global_counter[name]; ok {
		r.global_counter[name] = c + 1
		ret = c + 1
	} else {
		r.global_counter[name] = 1 
		ret = 1
	}

	r.global_counter_mutex.Unlock()

	return ret 
}


func (r *Replay) FindTasksWithinRange(name string, loc [3]float64, radius float64) []string {

	r.global_dag_lock.Lock() 
	defer r.global_dag_lock.Unlock() 

	var result []string 

	for k, v := range r.dag.Name2Node {
		if v.PureName == name || (v.NodeType == NEW_TASK && name == ""){
			if distance(loc, v.Location) < radius {
				result = append(result, k)
			} 
		}
	}

	return result 



}

func (r *Replay) ExecuteTask(node *DAGNode, done chan int, cancel chan int, drone_handler *int) {
	//fmt.Println(node.Name, node.PureName, node.NodeType)

	if node.NodeType == ROOT  {
		local_done := make(chan int, len(node.Children))
		local_cancel := make(chan int, len(node.Children))

		//fmt.Println(len(node.Children))
		for _, child := range node.Children {
			go r.ExecuteTask(child, local_done, local_cancel, drone_handler)
		}

		for i :=0; i< len(node.Children); i ++ {
			_ = <- local_done 
			//fmt.Println(i, len(node.Children))
		}

	} 

	if node.NodeType == NEW_TASK {
		if *drone_handler != -1 {
			r._release_drone(*drone_handler)
			*drone_handler = -1
		}

		var dependencies []string 

		for _, dep := range node.Parents {
			dependencies = append(dependencies, dep.Name)
		}

		r._addDependency(node.ScopeName, node.Name,  dependencies, node.MetaInfo)

		local_done := make(chan int, len(node.Children))
		local_cancel := make(chan int, len(node.Children))
		for _, child := range node.Children {
			go r.ExecuteTask(child, local_done, local_cancel, drone_handler)
		}

		counter := 0 

		for {
			select {
			case _ =<- local_done:
				counter += 1
				//fmt.Println("counter", counter)
				if counter == len(node.Children) {
					//fmt.Println(node.Name, "return")
					done <- 1
					return 
					
				}
			case _ =<- cancel:
				for i :=0; i< len(node.Children); i ++ {
					local_cancel <- 1 
				}
				done <- 1

				return 
			}
		}
	}

	if node.NodeType == CODEBLOCK {
		// do the action in code blcok 

		// acquire a drone at location  node.Location
		// sleep for time node.Duration 
		var dependencies []string 

		for _, dep := range node.Parents {
			dependencies = append(dependencies, dep.Name)
		}

		//fmt.Println("CodeBlcok location:", node.Location, " duration: ", node.Duration, "cancelName:", node.CancelName, len(node.Children))

		// first add dependency 

		r._addDependency(node.ScopeName, node.Name,  dependencies, node.MetaInfo)


		// let's check the prediction!

	
		//time.Sleep(time.Duration(1000 * 30) * time.Millisecond)


		// drone_handler = r._acquire_drone(node.ScopeName, node.Name, node.Location)
		// r._release_drone(drone_handler)

		// drone_handler = -1

		var new_drone_handler *int = new(int)


		if *drone_handler == -1 {

			*new_drone_handler  = r._acquire_drone(node.ScopeName, node.Name, node.Location)
			if *new_drone_handler == -1 {
				// cancelled ...
				//fmt.Println("-1 drone handler")
				done <- 1
				return 
			}

		} else {
			new_drone_handler = drone_handler
		}

		r.VirtualSleep(3.0)


		//time.Sleep(time.Duration(100) * time.Millisecond)
		// TODO
		
		preds :=r.dag.match(r.dag.Name2Node[node.Name], []*DAG{r.dag})

		for _, vnode := range preds.Nodes {
			if vnode.CancelOtherTask {
				//fmt.Println("CancelOtherTaskMask",vnode.CancelOtherTaskMask) // TODO this is empty???

				cancel_tasks := r.FindTasksWithinRange(vnode.CancelOtherTaskMask, node.Location, vnode.CancelOtherTaskRadius)

				r._cancelRequest(node.ScopeName, cancel_tasks, []string{node.Name})

			}
		}
		


		if len(node.Children) > 0 {
			local_done := make(chan int, len(node.Children))
			local_cancel := make(chan int, len(node.Children))
			for _, child := range node.Children {
				r.ExecuteTask(child, local_done, local_cancel, new_drone_handler)
			}


			counter := 0 

			for {
				select {
				case _ =<- local_done:
					counter += 1
					//fmt.Println("counter", counter)
					if counter == len(node.Children) {
						//fmt.Println(node.Name, "return")
						done <- 1
						return 
						
					}
				case _ =<- cancel:
					for i :=0; i< len(node.Children); i ++ {
						local_cancel <- 1 
					}
					done <- 1
					return 
				}
			}
	
		} else {
			if *new_drone_handler != -1 {
				r._release_drone(*new_drone_handler)
				*new_drone_handler = -1
			}
		}

	}

	if node.NodeType == CANCEL_TASK {
		//fmt.Println("cancel",node.Name, node.PureName, node.NodeType)
		if *drone_handler != -1 {
			r._release_drone(*drone_handler)
			*drone_handler = -1
		}

		var dependencies []string 

		for _, dep := range node.Parents {
			dependencies = append(dependencies, dep.Name)
		}

		// cancel happened in prediction 
		//r._cancelRequest(node.ScopeName, []string{node.CancelName}, dependencies)

	} 

	if node.NodeType == SYNC {

		if *drone_handler != -1 {
			r._release_drone(*drone_handler)
			*drone_handler = -1
		}


		// increace the counter
		// the last thread who reached the len(Children), generate new threads
		ret := r.IncreaseAndFectchCounter(node.Name)

		if ret == len(node.Children) {
			local_done := make(chan int, len(node.Children))
			local_cancel := make(chan int, len(node.Children))
			for _, child := range node.Children {
				go r.ExecuteTask(child, local_done, local_cancel,drone_handler)
			}

			for i :=0; i< len(node.Children); i ++ {
				_  = <- local_done 
			}				
		} else {
			// do nothing but return 
		}
	}

	done <- 1
}
func (r *Replay) newSession() {
	var cmd frameworkCMD 

	cmd.CMD = "newSession"
	cmd.AppID = r.appid 

	sendBytes, _ := json.Marshal(cmd)

	for {
		resp, err := http.Post("http://localhost:8008", "application/json", bytes.NewBuffer(sendBytes))
		if err != nil {
			time.Sleep(time.Duration(1) * time.Millisecond)
		} else {
			ret_bytes, _ := ioutil.ReadAll(resp.Body)
				
			if err := json.Unmarshal(ret_bytes, &(r.sessionid)); err != nil {
				//fmt.Println(string(buf))
				panic(err)
			}
			resp.Body.Close()
			break 
		}
	}
}

func (r *Replay) closeSession() {

	var cmd frameworkCMD 

	cmd.CMD = "stopSession"
	cmd.AppID = r.appid 
	cmd.SessionID = r.sessionid

	sendBytes, _ := json.Marshal(cmd)

	for {
		resp, err := http.Post("http://localhost:8008", "application/json", bytes.NewBuffer(sendBytes))
		if err != nil {
			time.Sleep(time.Duration(1) * time.Millisecond)
		} else {
			_, _ = ioutil.ReadAll(resp.Body)
			resp.Body.Close()
			break 
		}
	}
}


func (r *Replay) _acquire_drone(scopename string, nodename string, loc [3]float64) int {
	// new Request + getDrone 
	var cmd frameworkCMD 

	cmd.CMD = "newRequest"
	cmd.X = loc[0]
	cmd.Y = loc[1]
	cmd.Z = loc[2]
	cmd.ScopeName = scopename
	cmd.NodeName = nodename 
	cmd.AppID = r.appid 
	cmd.SessionID = r.sessionid


	sendBytes, _ := json.Marshal(cmd)

	var reqid int 

	for {
		resp, err := http.Post("http://localhost:8008", "application/json", bytes.NewBuffer(sendBytes))
		if err != nil {
			time.Sleep(time.Duration(1) * time.Millisecond)
		} else {

			ret_bytes, _ := ioutil.ReadAll(resp.Body)
				
			if err := json.Unmarshal(ret_bytes, &reqid); err != nil {
				//fmt.Println(string(buf))
				panic(err)
			}
			resp.Body.Close()

			break
		}

	}


	cmd.CMD = "getDrone"
	cmd.ID = reqid 
	cmd.AppID = r.appid 
	cmd.SessionID = r.sessionid
	

	sendBytes, _ = json.Marshal(cmd)
	var drone_handler int 

	for {
		resp, err := http.Post("http://localhost:8008", "application/json", bytes.NewBuffer(sendBytes))
		if err != nil {
			time.Sleep(time.Duration(1) * time.Millisecond)
		} else{
			ret_bytes, _ := ioutil.ReadAll(resp.Body)
	
			if err := json.Unmarshal(ret_bytes, &drone_handler); err != nil {
				//fmt.Println(string(buf))
				panic(err)
			}
			resp.Body.Close()
			break
		}
	}

	
	return drone_handler
}


func (r *Replay) _release_drone(drone_handler int){
	var cmd frameworkCMD 

	cmd.CMD = "releaseDrone"
	cmd.ID = drone_handler
	cmd.AppID = r.appid 
	cmd.SessionID = r.sessionid
	

	sendBytes, _ := json.Marshal(cmd)

	resp, err := http.Post("http://localhost:8008", "application/json", bytes.NewBuffer(sendBytes))
	if err != nil {
		panic(err)
	}

	_, _ = ioutil.ReadAll(resp.Body)
	resp.Body.Close()

}

func (r *Replay) _addDependency(scopename string, nodename string, dependencies []string, MetaInfo string ) {
	

	var cmd frameworkCMD 

	cmd.CMD = "addDependency"
	cmd.NodeName = nodename
	cmd.ScopeName = scopename 
	cmd.Dependencies = dependencies
	cmd.MetaInfo = MetaInfo
	cmd.AppID = r.appid 
	cmd.SessionID = r.sessionid
	

	sendBytes, _ := json.Marshal(cmd)

	for {
		resp, err := http.Post("http://localhost:8008", "application/json", bytes.NewBuffer(sendBytes))
		if err != nil {
			time.Sleep(time.Duration(1) * time.Millisecond)
		} else {
			_, _ = ioutil.ReadAll(resp.Body)
			resp.Body.Close()

			break 
		}
	}

	

}

func (r *Replay) _cancelRequest(scopename string, names []string, dependencies []string) {
	for _, name := range names {
		r._addDependency(scopename, "cancel_" + name,  dependencies, "\"nul")
	}

	for _, name := range names {
		var cmd frameworkCMD 

		cmd.CMD = "cancelRequest"
		cmd.ScopeName = name 
		cmd.AppID = r.appid 
		cmd.SessionID = r.sessionid
	

		sendBytes, _ := json.Marshal(cmd)

		for {
			resp, err := http.Post("http://localhost:8008", "application/json", bytes.NewBuffer(sendBytes))
			if err != nil {
				time.Sleep(time.Duration(1) * time.Millisecond)
			} else {
				_, _ = ioutil.ReadAll(resp.Body)
				resp.Body.Close()

				break 
			}
		}

	}

}

func (r *Replay) _getDuration() float64 {
	var cmd droneCMD 

	cmd.CMD = "GetDuration"
	cmd.ID = 0 
	cmd.AppID = r.appid 
	cmd.SessionID = r.sessionid
	

	sendBytes, _ := json.Marshal(cmd)

	var duration float64

	for {
		resp, err := http.Post("http://localhost:8009", "application/json", bytes.NewBuffer(sendBytes))
		if err != nil {
			time.Sleep(time.Duration(1) * time.Millisecond)
		} else {
			ret_bytes, _ := ioutil.ReadAll(resp.Body)
	
			if err := json.Unmarshal(ret_bytes, &duration); err != nil {
				//fmt.Println(string(buf))
				panic(err)
			}
			resp.Body.Close()
			break
		}
	}

	return duration 
}

func (r *Replay) VirtualSleep(delay float64)  {
	var cmd droneCMD 

	cmd.CMD = fmt.Sprintf("VirtualSleep_%.3f_", delay)
	cmd.ID = 0 
	cmd.AppID = r.appid 
	cmd.SessionID = r.sessionid
	

	sendBytes, _ := json.Marshal(cmd)

	for {
		resp, err := http.Post("http://localhost:8009", "application/json", bytes.NewBuffer(sendBytes))
		if err != nil {
			time.Sleep(time.Duration(1) * time.Millisecond)
		} else {
			_,_ = ioutil.ReadAll(resp.Body)
			resp.Body.Close()
			break
		}
	}

}
































