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
	"fmt"
)

// static gaussians, one major gaussian plus other samll gaussians

type static_pollution_env struct {
	gaussian	[][5]float64

	sensed_location [][3]float64
}

func MakeStaticPollutionEnv(n int) *static_pollution_env {
	env := new(static_pollution_env)

	for i :=0; i< n; i++ {

		var x float64
		var y float64
		var m float64
		var s float64
		
		x = rand.Float64()*400.0 - 200.0 - 350
		y = rand.Float64()*400.0 - 200.0 - 350

		//x = rand.Float64()*400.0 - 200.0 
		//y = rand.Float64()*400.0 - 200.0 

		if i == 0 {
			x = -350
			y = -350
			m = 1.0
			s = rand.Float64()*50.0 + 50.0
		} else {
			m = rand.Float64()*0.02+0.02
			s = rand.Float64()*200.0 + 200.0
		}
	
		env.gaussian = append(env.gaussian, [5]float64{x,y,m,s,s})
	}


	return env
}

func (env *static_pollution_env) UpdateBackground(time float64, name string) {
	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})
	n := len(env.gaussian)

	for x:=0; x<800; x++ {
		for y:=0; y<800; y++ {
			v := 0.0 

			for i:=0; i<n; i++ {
				_x := float64(x-400)
				_y := float64(y-400)

				v = v + env.gaussian[i][2] * math.Exp(-( (_x - env.gaussian[i][0]) * (_x - env.gaussian[i][0]) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - env.gaussian[i][1]) * (_y - env.gaussian[i][1]) / 2 / env.gaussian[i][4] / env.gaussian[i][4]    ))
			}

			iv := uint8(v*192)

			//c := color.RGBA{iv, 0, (255-iv)/2, 255}

			c := color.RGBA{255 - iv/9, 255 - iv/2, 255 - iv/2, 255}


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
			vmax:= 0.0 
			vmin:= 1.5 


			for jj:=0; jj<5; jj ++ {

				if vs[jj]>vmax{
					vmax = vs[jj]
				}
				if vs[jj] < vmin {
					vmin = vs[jj]
				}
			}


			for tr := 0.1; tr < 1.0; tr += 0.1 {
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

func (env *static_pollution_env) Sense(loc [3]float64, time float64, action string) interface{} {

	v := 0.0 

	for i:=0; i<len(env.gaussian); i++ {
		_x := loc[0]
		_y := loc[1]

		v = v + env.gaussian[i][2] * math.Exp(-( (_x - env.gaussian[i][0]) * (_x - env.gaussian[i][0]) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - env.gaussian[i][1]) * (_y - env.gaussian[i][1]) / 2 / env.gaussian[i][4] / env.gaussian[i][4]    ))
	}

	env.sensed_location = append(env.sensed_location, [3]float64{loc[0],loc[1],loc[2]})

	return v
}




// dynamic gaussians, one major gaussian plus other samll gaussians


type dynamic_pollution_env struct {
	gaussian	[][5]float64

	dist	float64
	phase	float64
	speed	float64

}

func MakeDynamicPollutionEnv(n int) *dynamic_pollution_env {
	env := new(dynamic_pollution_env)

	for i :=0; i< n; i++ {

		var x float64
		var y float64
		var m float64
		var s float64
		
		x = rand.Float64()*400.0 - 200.0
		y = rand.Float64()*400.0 - 200.0

		if i == 0 {
			m = 1.0
			s = rand.Float64()*50.0 + 50.0

			env.dist = math.Sqrt(x*x + y*y)
			env.phase = rand.Float64()*3.1415926
			env.speed = 0.0005


		} else {
			m = rand.Float64()*0.02+0.02
			s = rand.Float64()*200.0 + 200.0
		}
	
		env.gaussian = append(env.gaussian, [5]float64{x,y,m,s,s})
	}

	return env
}


func (env *dynamic_pollution_env) UpdateBackground(time float64, name string) {

	env.gaussian[0][0] = math.Cos(time * env.speed + env.phase) * env.dist
	env.gaussian[0][1] = math.Sin(time * env.speed + env.phase) * env.dist

	n := len(env.gaussian)
	

	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})

	for x:=0; x<800; x++ {
		for y:=0; y<800; y++ {
			v := 0.0 

			for i:=0; i<n; i++ {
				_x := float64(x-400)
				_y := float64(y-400)

				v = v + env.gaussian[i][2] * math.Exp(-( (_x - env.gaussian[i][0]) * (_x - env.gaussian[i][0]) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - env.gaussian[i][1]) * (_y - env.gaussian[i][1]) / 2 / env.gaussian[i][4] / env.gaussian[i][4]    ))
			}

			iv := uint8(v*192)

			//c := color.RGBA{iv, 0, (255-iv)/2, 255}

			c := color.RGBA{255 - iv/9, 255 - iv/2, 255 - iv/2, 255}


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
			vmax:= 0.0 
			vmin:= 1.5 


			for jj:=0; jj<5; jj ++ {

				if vs[jj]>vmax{
					vmax = vs[jj]
				}
				if vs[jj] < vmin {
					vmin = vs[jj]
				}
			}


			for tr := 0.1; tr < 1.0; tr += 0.1 {
				if vmax >= tr && vmin < tr {
					c := color.RGBA{128,128,128,255}
					img.Set(x,y,c)
				}
			}
		}
	}

	f, _ := os.Create(name)
	png.Encode(f, img)	
}


func (env *dynamic_pollution_env) Sense(loc [3]float64, time float64, action string) interface{} {


	v := 0.0 

	for i:=0; i<len(env.gaussian); i++ {
		_x := loc[0]
		_y := loc[1]


		g_x := env.gaussian[i][0]
		g_y := env.gaussian[i][1]

		if i == 0 {
			g_x = math.Cos(time * env.speed + env.phase) * env.dist
			g_y = math.Sin(time * env.speed + env.phase) * env.dist

			fmt.Println("sense", g_x, g_y, time)
		}

		v = v + env.gaussian[i][2] * math.Exp(-( (_x - g_x) * (_x - g_x) / 2 / env.gaussian[i][3] / env.gaussian[i][3] + (_y - g_y) * (_y - g_y) / 2 / env.gaussian[i][4] / env.gaussian[i][4] ))
	}

	return v
}

































