package main 

/*
 * Simulator Environment
 * 
 */

import (
	"github.com/llgcode/draw2d/draw2dimg"
	"image"
	"image/color"
	"image/png"
	"math/rand"
	"math"
	"os"
	//"fmt"
)


type gaussian_process_env struct {
	gaussian	[][5]float64
	sensed_location [][3]float64
	maxvalue float64
	minvalue float64

	firstrun bool 
}

func MakeGaussianProcessEnv(n int) *gaussian_process_env {
	env := new(gaussian_process_env)

	for i :=0; i< n; i++ {

		var x float64
		var y float64
		var m float64
		var s float64
		
		x = rand.Float64()*600.0 - 300.0 //- 350
		y = rand.Float64()*600.0 - 300.0 //- 350

		//x = rand.Float64()*400.0 - 200.0 
		//y = rand.Float64()*400.0 - 200.0 

		
		m = (rand.Float64()*2.0 - 1.0) * 0.8 // * (1.0/math.Sqrt(float64(n)))
		
		s = rand.Float64()*100.0 + 50.0
		
		//m = 0.0 

		env.gaussian = append(env.gaussian, [5]float64{x,y,m,s,s})
	}


	env.maxvalue = 0.0
	env.minvalue = 0.0
	env.firstrun = true 

	return env
}

func (env *gaussian_process_env) UpdateBackground(time float64, name string) {
	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})
	n := len(env.gaussian)


	// flag := false


	// if env.firstrun {
	// 	env.firstrun = false 
	// 	flag = true
	// }

	for x:=0; x<800; x++ {
		for y:=0; y<800; y++ {
			v := 0.0 

			for i:=0; i<n; i++ {
				_x := float64(x-400)
				_y := float64(y-400)

				v = v + env.gaussian[i][2] * math.Exp(-( (_x - env.gaussian[i][0]) * (_x - env.gaussian[i][0]) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - env.gaussian[i][1]) * (_y - env.gaussian[i][1]) / 2 / env.gaussian[i][4] / env.gaussian[i][4]    ))
			}

			c := color.RGBA{255, 255, 255, 255}


			if v > 0.0 {

				iv := int(v*192)
				if iv > 511 {
					iv = 511
				}

				//c := color.RGBA{iv, 0, (255-iv)/2, 255}

				c = color.RGBA{255 - uint8(iv/9), 255 - uint8(iv/2), 255 - uint8(iv/2), 255}
			} else {
				iv := int(-v*192)
				if iv > 511 {
					iv = 511
				}


				//c := color.RGBA{iv, 0, (255-iv)/2, 255}

				c = color.RGBA{255 - uint8(iv/2), 255 - uint8(iv/2), 255 - uint8(iv/9), 255}


			}

			img.Set(x,y,c)
		}
	}

	for x:=1; x<799; x++ {
		for y:=1; y<799; y++ {
			v := 0.0 

			for i:=0; i<n; i++ {
				_x := float64(x-400)
				_y := float64(y-400)

				v = v + env.gaussian[i][2] * math.Exp(-( (_x - env.gaussian[i][0]) * (_x - env.gaussian[i][0]) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - env.gaussian[i][1]) * (_y - env.gaussian[i][1]) / 2 / env.gaussian[i][4] / env.gaussian[i][4]    ))
			}


			v0 := v 

			v = 0.0 

			for i:=0; i<n; i++ {
				_x := float64(x+1-400)
				_y := float64(y-400)

				v = v + env.gaussian[i][2] * math.Exp(-( (_x - env.gaussian[i][0]) * (_x - env.gaussian[i][0]) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - env.gaussian[i][1]) * (_y - env.gaussian[i][1]) / 2 / env.gaussian[i][4] / env.gaussian[i][4]    ))
			}

			v1:=v

			v = 0.0 

			for i:=0; i<n; i++ {
				_x := float64(x-1-400)
				_y := float64(y-400)

				v = v + env.gaussian[i][2] * math.Exp(-( (_x - env.gaussian[i][0]) * (_x - env.gaussian[i][0]) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - env.gaussian[i][1]) * (_y - env.gaussian[i][1]) / 2 / env.gaussian[i][4] / env.gaussian[i][4]    ))
			}

			v2:=v

			v = 0.0 

			for i:=0; i<n; i++ {
				_x := float64(x-400)
				_y := float64(y+1-400)

				v = v + env.gaussian[i][2] * math.Exp(-( (_x - env.gaussian[i][0]) * (_x - env.gaussian[i][0]) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - env.gaussian[i][1]) * (_y - env.gaussian[i][1]) / 2 / env.gaussian[i][4] / env.gaussian[i][4]    ))
			}

			v3:=v

			v = 0.0 

			for i:=0; i<n; i++ {
				_x := float64(x-400)
				_y := float64(y-1-400)

				v = v + env.gaussian[i][2] * math.Exp(-( (_x - env.gaussian[i][0]) * (_x - env.gaussian[i][0]) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - env.gaussian[i][1]) * (_y - env.gaussian[i][1]) / 2 / env.gaussian[i][4] / env.gaussian[i][4]    ))
			}

			v4:=v

			vs := [5]float64{v0,v1,v2,v3,v4}
			vmax:= -1.5 
			vmin:= 1.5 


			for jj:=0; jj<5; jj ++ {

				if vs[jj]>vmax{
					vmax = vs[jj]
				}
				if vs[jj] < vmin {
					vmin = vs[jj]
				}
			}


			for tr := -1.0; tr < 1.0; tr += 0.1 {
				if vmax >= tr && vmin < tr {
					c := color.RGBA{128,128,128,255}
					img.Set(x,y,c)
				}
			}
		}
	}


	gc := draw2dimg.NewGraphicContext(img)

	gc.SetStrokeColor(color.RGBA{0x00, 0xff, 0x00, 0xff})
	gc.SetLineWidth(3)


	for _, loc := range env.sensed_location {
		x := loc[0] + 400.0 
		y := loc[1] + 400.0

		gc.MoveTo(x-3,y-3)
		gc.LineTo(x+3,y-3)
		gc.LineTo(x+3,y+3)
		gc.LineTo(x-3,y+3)
		gc.LineTo(x-3,y-3)
		gc.Close()
		gc.FillStroke()
	}



	f, _ := os.Create(name)
	png.Encode(f, img)	
}

func (env *gaussian_process_env) Sense(loc [3]float64, time float64, action string) interface{} {

	v := 0.0 

	for i:=0; i<len(env.gaussian); i++ {
		_x := loc[0]
		_y := loc[1]

		v = v + env.gaussian[i][2] * math.Exp(-( (_x - env.gaussian[i][0]) * (_x - env.gaussian[i][0]) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - env.gaussian[i][1]) * (_y - env.gaussian[i][1]) / 2 / env.gaussian[i][4] / env.gaussian[i][4]    ))
	}

	env.sensed_location = append(env.sensed_location, [3]float64{loc[0],loc[1],loc[2]})

	return v
}




