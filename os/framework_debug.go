package main 


/*
 * This file implements some helper function for debug
 * 
 */

import (
	"fmt"
	"time"
)



//debuglist := make(map[int]bool)

var debuglist map[int]bool


func (fra *Framework) DebugPrint() {
	debuglist = make(map[int]bool)
	for {
		time.Sleep(time.Duration(5000000) * time.Microsecond)
		continue

		fra.mutex.Lock()

		fmt.Println("--------------- Debug Print ------------------")
		for reqid, req := range fra.Requests {
			if req.phase < 2 {
				fmt.Println("active request", reqid, req.phase, req)
			}
		}

		fmt.Println("request waiting list", debuglist)
		for k,_ := range debuglist {
			fmt.Println("probably bad request", fra.Requests[k].phase, fra.Requests[k])
		}

		fra.mutex.Unlock()
	}
}


func (fra *Framework) DebugAddToWaitingList(reqid int) {
	debuglist[reqid] = true

}

func (fra *Framework) DebugRemoveFromWaitingList(reqid int) {

	delete(debuglist, reqid)
}