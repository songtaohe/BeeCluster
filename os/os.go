package main 

/*
 * This file is the entry of the BeeCluster 
 * 
 */

import (
	"fmt"
	"os"
)

var p = fmt.Println


func main() {
	if len(os.Args) >= 2 {
		//p("BeeCluster Replay Mode")
		LogLevel = -1 // no logs 
		f := MakeFramework("os/configs/framework/framework_config_replay.json", false) 
		// [1] dag to be replay
		// [2] app name (load historical DTG)
		r := MakeReplay(os.Args[1], os.Args[2], f)

		r.Replay()

		//fmt.Println(r)

		
	} else {
		p("BeeCluster OS")
		_ = MakeFramework("os/configs/framework/framework_config.json", true) 
	
	}
}

