package main 

/*
 * This file implements the collision avoidance and conflicts resolution protocol.
 * 
 */

import (
	"sync"
	"math"
	"fmt"
)


var SAFE_DISTANCE = 1.5

const (
	ATC_HOLD = 0
	ATC_PROCEED = 1 
	ATC_PROCEED_WITH_CONTROL_POINTS = 2 
	ATC_HOLD_AT_POSITION = 3 	
)


type ATCTicket struct {
	TicketNumber 	int
	Target			[3]float64 
	Orig 			[3]float64
	Solution 		int

	ControlPoint1 	[3]float64
	ControlPoint2 	[3]float64

	HoldPoint 		[3]float64

	Cancel 			bool 

	droneid 	int 
}



type ATC struct {
	
	tickets 	[]*ATCTicket  
	ticketcounter	int 

	mutex 		sync.Mutex

	dronelocations [][3]float64
}

func MakeATC(nDrone int) *ATC {
	atc := new(ATC)

	atc.dronelocations = make([][3]float64, nDrone)
	for i:=0; i<nDrone; i++ {
		atc.dronelocations[i] = [3]float64{0.0, 0.0, -10.0 } // LOL
	}
	atc.ticketcounter = 0 


	//atc.test()



	return atc 
}



func GenerateTicketSolutionFlyOver(ticket *ATCTicket) {
	ticket.Solution = ATC_PROCEED_WITH_CONTROL_POINTS

	ticket.ControlPoint1 = ticket.Orig
	ticket.ControlPoint1[2] += SAFE_DISTANCE * 2.5

	ticket.ControlPoint2 = ticket.Target
	ticket.ControlPoint2[2] += SAFE_DISTANCE * 2.5
	
}	

func GenerateTicketSolutionFlyOnLeft(ticket *ATCTicket) bool {
	ticket.Solution = ATC_PROCEED_WITH_CONTROL_POINTS
	
	dx := ticket.Target[0] - ticket.Orig[0]
	dy := ticket.Target[1] - ticket.Orig[1]
	dz := ticket.Target[2] - ticket.Orig[2]

	d := math.Sqrt(dx*dx + dy*dy + 0.0001)

	if d < SAFE_DISTANCE * 5.0 {
		return false 
	}

	vx := dx/d * SAFE_DISTANCE * 2.5
	vy := dy/d * SAFE_DISTANCE * 2.5

	d = math.Sqrt(dx*dx + dy*dy + dz*dz)
	vz := dz/d * SAFE_DISTANCE * 2.5

	ticket.ControlPoint1[0] = ticket.Orig[0] + vx - vy
	ticket.ControlPoint1[1] = ticket.Orig[1] + vx + vy
	ticket.ControlPoint1[2] = ticket.Orig[2] + vz 


	ticket.ControlPoint2[0] = ticket.Target[0] - vx - vy
	ticket.ControlPoint2[1] = ticket.Target[1] + vx - vy
	ticket.ControlPoint2[2] = ticket.Target[2] - vz 

	return true 
}

func GenerateTicketSolutionFlyOnRight(ticket *ATCTicket) bool {
	ticket.Solution = ATC_PROCEED_WITH_CONTROL_POINTS
	
	dx := ticket.Target[0] - ticket.Orig[0]
	dy := ticket.Target[1] - ticket.Orig[1]
	dz := ticket.Target[2] - ticket.Orig[2]

	d := math.Sqrt(dx*dx + dy*dy + 0.0001)

	if d < SAFE_DISTANCE * 5.0 {
		return false 
	}

	vx := dx/d * SAFE_DISTANCE * 2.5
	vy := dy/d * SAFE_DISTANCE * 2.5

	d = math.Sqrt(dx*dx + dy*dy + dz*dz)
	vz := dz/d * SAFE_DISTANCE * 2.5

	ticket.ControlPoint1[0] = ticket.Orig[0] + vx + vy
	ticket.ControlPoint1[1] = ticket.Orig[1] + vx - vy
	ticket.ControlPoint1[2] = ticket.Orig[2] + vz 


	ticket.ControlPoint2[0] = ticket.Target[0] - vx + vy
	ticket.ControlPoint2[1] = ticket.Target[1] + vx + vy
	ticket.ControlPoint2[2] = ticket.Target[2] - vz 

	return true 
}

func (atc *ATC) Ticket2Segments(ticket1 *ATCTicket) ([][3]float64, [][3]float64){
	var segments1p1 [][3]float64
	var segments1p2 [][3]float64

	switch ticket1.Solution {
	case ATC_HOLD:
		segments1p1 = append(segments1p1, atc.dronelocations[ticket1.droneid])
		segments1p2 = append(segments1p2, atc.dronelocations[ticket1.droneid])

	case ATC_HOLD_AT_POSITION:
		// segments1p1 = append(segments1p1, ticket1.Orig)
		// segments1p2 = append(segments1p2, ticket1.HoldPoint)

		segments1p1 = append(segments1p1, atc.dronelocations[ticket1.droneid])
		segments1p2 = append(segments1p2, ticket1.Target)



	case ATC_PROCEED_WITH_CONTROL_POINTS:

		d1 := distance(atc.dronelocations[ticket1.droneid], ticket1.ControlPoint2)
		d2 := distance(ticket1.ControlPoint1, ticket1.ControlPoint2)


		if d2 < d1 {
			segments1p1 = append(segments1p1, atc.dronelocations[ticket1.droneid])
			segments1p2 = append(segments1p2, ticket1.ControlPoint1)

			segments1p1 = append(segments1p1, ticket1.ControlPoint1)
			segments1p2 = append(segments1p2, ticket1.ControlPoint2)

			segments1p1 = append(segments1p1, ticket1.ControlPoint2)
			segments1p2 = append(segments1p2, ticket1.Target)

		} else {

			d1 := distance(atc.dronelocations[ticket1.droneid], ticket1.Target)
			d2 := distance(ticket1.ControlPoint2, ticket1.Target)

			if d2 < d1 {
				segments1p1 = append(segments1p1, atc.dronelocations[ticket1.droneid])
				segments1p2 = append(segments1p2, ticket1.ControlPoint2)

				segments1p1 = append(segments1p1, ticket1.ControlPoint2)
				segments1p2 = append(segments1p2, ticket1.Target)
			} else {

				segments1p1 = append(segments1p1, atc.dronelocations[ticket1.droneid])
				segments1p2 = append(segments1p2, ticket1.Target)
			}
		}
	
	case ATC_PROCEED:
		segments1p1 = append(segments1p1, atc.dronelocations[ticket1.droneid])
		segments1p2 = append(segments1p2, ticket1.Target)
	}


	return segments1p1, segments1p2
}

func CheckConflict(ticket1 *ATCTicket, ticket2 *ATCTicket, atc *ATC) bool {
	
	segments1p1, segments1p2 := atc.Ticket2Segments(ticket1)
	segments2p1, segments2p2 := atc.Ticket2Segments(ticket2)


	for ind1, _ := range segments1p1 {
		for ind2, _ := range segments2p1 {
			if segmentDistance(segments1p1[ind1], segments1p2[ind1], segments2p1[ind2], segments2p2[ind2]) < SAFE_DISTANCE { 
				return true
			}
		}
	} 

	return false 
}


func (atc *ATC) getTicket(from [3]float64, to [3]float64, did int) *ATCTicket {
	
	atc.mutex.Lock()

	var ticket ATCTicket
	ticket.TicketNumber = atc.ticketcounter
	atc.ticketcounter += 1

	ticket.Target = to 
	ticket.Orig   = from 
	ticket.Cancel = false 
	ticket.Solution = ATC_PROCEED
	ticket.droneid = did 


	for _, t := range atc.tickets {
		if t.droneid == did {
			t.Cancel = true // remove old tickets 
		}
	}

	// Remove cancelled tickets 
	cc := 0
	for _, t := range atc.tickets {
		if t.Cancel == true {
			cc += 1
		} else {
			break
		}
	}

	if cc == len(atc.tickets) {
		atc.tickets = []*ATCTicket{}
	} else {
		atc.tickets = atc.tickets[cc:len(atc.tickets)]
	}




	atc.tickets = append(atc.tickets, &ticket)

	// find solution
	var ok bool 

	if atc.checkConflict(&ticket) == true {
		ok = GenerateTicketSolutionFlyOnLeft(&ticket)
		
		if atc.checkConflict(&ticket) == true || ok == false{
			ok = GenerateTicketSolutionFlyOnRight(&ticket)

			if atc.checkConflict(&ticket) == true || ok == false{
				GenerateTicketSolutionFlyOver(&ticket)

				if atc.checkConflict(&ticket) == true {
					atc.GenerateTicketSolutionHold(&ticket)
				}
			}
		}	
	}

	atc.mutex.Unlock()


	return &ticket 
}


func (atc *ATC) updateTicket(ticket *ATCTicket) {
	atc.mutex.Lock()

	if ticket.Solution == ATC_HOLD_AT_POSITION {

		ticket.Solution = ATC_PROCEED

		var ok bool 

		if atc.checkConflict(ticket) == true {
			ok = GenerateTicketSolutionFlyOnLeft(ticket)
			
			if atc.checkConflict(ticket) == true || ok == false{
				ok = GenerateTicketSolutionFlyOnRight(ticket)

				if atc.checkConflict(ticket) == true || ok == false {
					GenerateTicketSolutionFlyOver(ticket)

					if atc.checkConflict(ticket) == true {
						atc.GenerateTicketSolutionHold(ticket)


					}
				}
			}	
		}
	}

	atc.mutex.Unlock()
}

func (atc *ATC) cancelTicket(ticket *ATCTicket) {
	atc.mutex.Lock()

	ticket.Cancel = true 

	atc.mutex.Unlock()
}


func (atc *ATC) GenerateTicketSolutionHold(ticket *ATCTicket) {
	a_l := 0.0 
	a_r := 1.0 

	ticket.Solution = ATC_HOLD_AT_POSITION

	for {
		a_mid := (a_l + a_r) / 2.0

		var loc [3]float64

		loc[0] = ticket.Orig[0] * (1.0-a_mid) + ticket.Target[0] * a_mid
		loc[1] = ticket.Orig[1] * (1.0-a_mid) + ticket.Target[1] * a_mid
		loc[2] = ticket.Orig[2] * (1.0-a_mid) + ticket.Target[2] * a_mid

		ticket.HoldPoint = loc 

		if atc.checkConflict(ticket) == false {
			a_l = a_mid
		} else {
			a_r = a_mid
		}

		if a_r - a_l < 0.01 {
			ticket.HoldPoint[0] = ticket.Orig[0] * (1.0-a_l) + ticket.Target[0] * a_l
			ticket.HoldPoint[1] = ticket.Orig[1] * (1.0-a_l) + ticket.Target[1] * a_l
			ticket.HoldPoint[2] = ticket.Orig[2] * (1.0-a_l) + ticket.Target[2] * a_l

			break	
		}
	}

}

func (atc *ATC) checkConflict(ticket *ATCTicket) bool{
	//fmt.Println("checkConflict", atc.tickets)

	if len(atc.tickets) == 1 {
		
	} else {
		for _, t := range atc.tickets {
			if t.Cancel == true {
				continue 
			}

			if t.TicketNumber == ticket.TicketNumber {
				break
			}

			if CheckConflict(t, ticket, atc) {
				return true
			}
		}
	}

	segmentsp1, segmentsp2 := atc.Ticket2Segments(ticket)
	
	for did, dloc := range atc.dronelocations {
		if did == ticket.droneid {
			continue
		}

		for ind, _ := range segmentsp1 {
			if point2segmentDistance(segmentsp1[ind], segmentsp2[ind], dloc) < SAFE_DISTANCE {
				return true
			}
		}
	}

	return false 
}


func (atc *ATC) setDroneLocations(droneid int, loc [3]float64) {
	atc.mutex.Lock()
	defer atc.mutex.Unlock()

	atc.dronelocations[droneid] = loc

}

func (atc *ATC) dumpQueue() {
	atc.mutex.Lock()
	defer atc.mutex.Unlock()


	cc := 0 
	for _, t:= range atc.tickets {
		if t.Cancel == false {
			fmt.Printf("[ATC-%d] Drone %d: _atc_%d_wp1_%.6f_%.6f_%.6f_wp2_%.6f_%.6f_%.6f_hp_%.6f_%.6f_%.6f \n",cc, t.droneid, t.Solution, t.ControlPoint1[0], t.ControlPoint1[1], t.ControlPoint1[2], t.ControlPoint2[0], t.ControlPoint2[1], t.ControlPoint2[2], t.HoldPoint[0], t.HoldPoint[1], t.HoldPoint[2])
			cc += 1
		}
	}

	for _, t:= range atc.tickets {
		if t.Cancel == false {
			for _, t2:= range atc.tickets {
				if t2.Cancel == false && t.TicketNumber != t2.TicketNumber{
					conflict := CheckConflict(t,t2,atc)
					if conflict == true {
						fmt.Println("[ATC] !!!!!!!!!!!!!!!!!!!!!!!! Conflict !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

					}
					
				}
			}
		}
	}



}

func (atc *ATC) test() {

	// the following code is a unit test. 
	
	fmt.Println(segmentDistance([3]float64{0.0, 0.0, 5.0}, [3]float64{0.0, 20.0, 5.0}, [3]float64{-10.0, 10.0, 5.0}, [3]float64{10.0, 10.0, 5.0}))

	t1 := atc.getTicket([3]float64{0.0, 0.0, 5.0}, [3]float64{0.0, 20.0, 5.0}, 0)
	t := t1
	fmt.Printf("T1: _atc_%d_wp1_%.6f_%.6f_%.6f_wp2_%.6f_%.6f_%.6f_hp_%.6f_%.6f_%.6f \n", t.Solution, t.ControlPoint1[0], t.ControlPoint1[1], t.ControlPoint1[2], t.ControlPoint2[0], t.ControlPoint2[1], t.ControlPoint2[2], t.HoldPoint[0], t.HoldPoint[1], t.HoldPoint[2])
				
	t2 := atc.getTicket([3]float64{-10.0, 10.0, 5.0}, [3]float64{10.0, 10.0, 5.0}, 1)
	t = t2
	fmt.Printf("T2: _atc_%d_wp1_%.6f_%.6f_%.6f_wp2_%.6f_%.6f_%.6f_hp_%.6f_%.6f_%.6f \n", t.Solution, t.ControlPoint1[0], t.ControlPoint1[1], t.ControlPoint1[2], t.ControlPoint2[0], t.ControlPoint2[1], t.ControlPoint2[2], t.HoldPoint[0], t.HoldPoint[1], t.HoldPoint[2])
	
	t3 := atc.getTicket([3]float64{-10.0, 0.0, 5.0}, [3]float64{10.0, 20.0, 5.0}, 2)
	t = t3
	fmt.Printf("T3: _atc_%d_wp1_%.6f_%.6f_%.6f_wp2_%.6f_%.6f_%.6f_hp_%.6f_%.6f_%.6f \n", t.Solution, t.ControlPoint1[0], t.ControlPoint1[1], t.ControlPoint1[2], t.ControlPoint2[0], t.ControlPoint2[1], t.ControlPoint2[2], t.HoldPoint[0], t.HoldPoint[1], t.HoldPoint[2])
	
	atc.cancelTicket(t2)

	fmt.Println("cancel T2")

	atc.updateTicket(t3)
	t = t3
	fmt.Printf("T3: _atc_%d_wp1_%.6f_%.6f_%.6f_wp2_%.6f_%.6f_%.6f_hp_%.6f_%.6f_%.6f \n", t.Solution, t.ControlPoint1[0], t.ControlPoint1[1], t.ControlPoint1[2], t.ControlPoint2[0], t.ControlPoint2[1], t.ControlPoint2[2], t.HoldPoint[0], t.HoldPoint[1], t.HoldPoint[2])
	
	atc.cancelTicket(t3)
	atc.cancelTicket(t1)

}







