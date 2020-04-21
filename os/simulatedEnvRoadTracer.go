package main 

/*
 * Simulator Environment
 * 
 */

 
import (
	"github.com/llgcode/draw2d/draw2dimg"
	"image"
	"image/color"
	//"fmt"
)

type roadtracer_env struct {
	node map[int][2]float64
	adj map[int][]int

	loc2nodeid map[[2]int]int

	node_n int 

	sensed map[int]bool

	mapscale float64

}

func MakeRoadTracerEnv() *roadtracer_env {
	env := new(roadtracer_env)

	env.node_n = 0 
	env.node = make(map[int][2]float64)
	env.adj = make(map[int][]int)
	env.loc2nodeid = make(map[[2]int]int)
	env.sensed = make(map[int]bool)
	env.mapscale = 1.0 

	env.createTowns()

	return env
}


func (env *roadtracer_env) addedge(loc1 [2]float64, loc2 [2]float64) {
	iloc1 := [2]int{int(loc1[0]), int(loc1[1])}
	iloc2 := [2]int{int(loc2[0]), int(loc2[1])}


	var nid1 int 
	var nid2 int 

	if id, ok := env.loc2nodeid[iloc1]; ok {
		nid1 = id 
	} else {
		env.node[env.node_n] = loc1 
		nid1 = env.node_n
		env.adj[nid1] = []int{}
		env.sensed[nid1] = false
		env.loc2nodeid[iloc1] = nid1 
		env.node_n += 1
	}


	if id, ok := env.loc2nodeid[iloc2]; ok {
		nid2 = id 
	} else {
		env.node[env.node_n] = loc2 
		nid2 = env.node_n
		env.adj[nid2] = []int{}
		env.sensed[nid2] = false
		env.loc2nodeid[iloc2] = nid2 
		env.node_n += 1
	}


	var flag bool=false
	for _, neighbor := range env.adj[nid1] {
		if neighbor == nid2 {
			flag = true
			break
		}
	} 

	if flag == false {
		env.adj[nid1] = append(env.adj[nid1], nid2)
	}


	flag = false 
	for _, neighbor := range env.adj[nid2] {
		if neighbor == nid1 {
			flag = true
			break
		}
	} 

	if flag == false {
		env.adj[nid2] = append(env.adj[nid2], nid1)
	}

}




func (env *roadtracer_env) createBlock2x(n int, m int, start [2]float64, interval float64) {
	for i:=0; i<n; i++ {
		if i % 2 == 0{
		for j:=1; j<m; j++ {
			env.addedge([2]float64{start[0] + float64(i) * interval, start[1] + float64(j-1) * interval}, 
					[2]float64{start[0] + float64(i) * interval, start[1] + float64(j) * interval})
		}
		}
	}

	for i:=1; i<n; i++ {
		for j:=0; j<m; j++ {
			if j % 2 == 0 {
			env.addedge([2]float64{start[0] + float64(i-1) * interval, start[1] + float64(j) * interval}, 
					[2]float64{start[0] + float64(i) * interval, start[1] + float64(j) * interval})
			}
		}
	}

}

func (env *roadtracer_env) createBlock(n int, m int, start [2]float64, interval float64) {
	for i:=0; i<n; i++ {
		for j:=1; j<m; j++ {
			env.addedge([2]float64{start[0] + float64(i) * interval, start[1] + float64(j-1) * interval}, 
					[2]float64{start[0] + float64(i) * interval, start[1] + float64(j) * interval})
		}
	}

	for i:=1; i<n; i++ {
		for j:=0; j<m; j++ {
			env.addedge([2]float64{start[0] + float64(i-1) * interval, start[1] + float64(j) * interval}, 
					[2]float64{start[0] + float64(i) * interval, start[1] + float64(j) * interval})
		}
	}

}


func (env *roadtracer_env) createLongRoad(loc1 [2]float64,  loc2 [2]float64, interval float64) {
	dist := distance2(loc1,loc2)

	kk := int(dist/interval) + 1 


	for i:=0; i<kk; i ++ {
		alpha1 := float64(i)/float64(kk)
		alpha2 := float64(i+1)/float64(kk)

		newloc1 := [2]float64{0,0}
		newloc1[0] = loc1[0] * (1.0-alpha1) + loc2[0] * alpha1
		newloc1[1] = loc1[1] * (1.0-alpha1) + loc2[1] * alpha1

		newloc2 := [2]float64{0,0}
		newloc2[0] = loc1[0] * (1.0-alpha2) + loc2[0] * alpha2
		newloc2[1] = loc1[1] * (1.0-alpha2) + loc2[1] * alpha2


		env.addedge(newloc1, newloc2)

	}

}


func (env *roadtracer_env) createTowns() {

	// // towns 
	env.createBlock2x(5,5,[2]float64{-300,-300},50.0)
	env.createBlock2x(5,5,[2]float64{150,-300},50.0)
	env.createBlock2x(5,5,[2]float64{-300,150},50.0)
	env.createBlock2x(5,5,[2]float64{150,150},50.0)


	// links between towns 
	env.createLongRoad([2]float64{-200,-100},[2]float64{-200,150}, 50.0)

	env.createLongRoad([2]float64{-100,-200},[2]float64{150,-200}, 50.0)

	env.createLongRoad([2]float64{250,-100},[2]float64{250,150}, 50.0)

	env.createLongRoad([2]float64{-100,250},[2]float64{150,250}, 50.0)

	// env.createBlock(14,14,[2]float64{-300,-300},50.0)



}	



func (env *roadtracer_env) Sense(loc [3]float64, time float64, action string) interface{} {


	if action == "explore" {
		iloc := [2]int{int(loc[0]), int(loc[1])}

		ret := [][2]float64{}

		if nid, ok := env.loc2nodeid[iloc]; ok {

			env.sensed[nid] = true 

			for _, id := range env.adj[nid] {

				ret = append(ret, env.node[id])
			}
		}

		return ret 
	}


	if action == "coverage" {
		
		var ct int = 0 
		var cc int = 0 

		for _,v := range env.sensed {
			ct += 1
			if v {
				cc += 1
			}
		}

		return float64(cc) / float64(ct)
	}



	return nil 
}


func (env *roadtracer_env) UpdateBackground(time float64, name string)	{
	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})

	gc := draw2dimg.NewGraphicContext(img)

	gc.SetStrokeColor(color.RGBA{0x88, 0x88, 0x88, 0xff})
	gc.SetLineWidth(3)

	for k,v := range env.adj {
		loc1 := env.node[k]
		//fmt.Println(k,v)

		if env.sensed[k] == false {
			continue
		}

		for _, nid := range v {
			if env.sensed[nid] == false {
				continue
			}

			loc2 := env.node[nid]

			_x1 := (loc1[0]/env.mapscale + 400.0)
			_y1 := (loc1[1]/env.mapscale + 400.0)

			_x2 := (loc2[0]/env.mapscale + 400.0)
			_y2 := (loc2[1]/env.mapscale + 400.0)

			
			gc.MoveTo(_x1,_y1)
			gc.LineTo(_x2,_y2)
			gc.Close()
			gc.FillStroke()
		}
	}


	for k,v := range env.sensed {
		draw := false
		if v {
			gc.SetStrokeColor(color.RGBA{0x00, 0xff, 0x00, 0xff})
			draw = true 
		} else {
			gc.SetStrokeColor(color.RGBA{0xff, 0x00, 0x00, 0xff})

			for _, neighbor := range env.adj[k] {
				if env.sensed[neighbor] == true {
					gc.SetStrokeColor(color.RGBA{0x00, 0x00, 0xff, 0xff})
					draw = true
				}

			}

		}
		
		if draw {

			loc := env.node[k]

			x := loc[0]/env.mapscale + 400.0 
			y := loc[1]/env.mapscale + 400.0

			gc.MoveTo(x-3,y-3)
			gc.LineTo(x+3,y-3)
			gc.LineTo(x+3,y+3)
			gc.LineTo(x-3,y+3)
			gc.LineTo(x-3,y-3)
			gc.Close()
			gc.FillStroke()

		}
			

	}


	draw2dimg.SaveToPngFile(name, img)

}

