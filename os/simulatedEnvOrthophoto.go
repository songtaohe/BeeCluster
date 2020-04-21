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
	//"math/rand"
	//"math"
	"os"
	//"fmt"
)

// cell size = 50 meters
// 16 * 16 
type orthophoto_env struct {
	grid 		[][]int
	cover		[][]int 
	vis			[][]int

	cellsize	float64
	size 		int 
}

func MakeOrthophotoEnv(size int, threshold float64) *orthophoto_env{
	env := new(orthophoto_env)
	env.cellsize = 800.0 / float64(size) 
	env.size = size 
	env.grid = make([][]int, size)
	env.cover = make([][]int, size)
	env.vis = make([][]int, size)

	gpenv := MakeGaussianProcessEnv(20)

	for i:=0; i<size; i++ {
		env.grid[i] = make([]int, size)
		env.cover[i] = make([]int, size)
		env.vis[i] = make([]int, size)

		for j:=0; j<size; j++ {

			v := gpenv.Sense([3]float64{float64(i)*env.cellsize - 400.0, float64(j)*env.cellsize - 400.0, 0.0}, 0.0, "none").(float64)
			if v > threshold {
				env.grid[i][j] = 1
			} else {
				env.grid[i][j] = 0
			}

			env.cover[i][j] = 0 
			env.vis[i][j] = 0

		}
	}

	
	return env 
}

func (env *orthophoto_env) UpdateBackground(time float64, name string) {
	img := image.NewRGBA(image.Rectangle{image.Point{0,0}, image.Point{800,800}})
	gc := draw2dimg.NewGraphicContext(img)

	
	gc.SetLineWidth(3)

	for i:=0; i<env.size; i++ {
		for j:=0; j<env.size; j++ {
			x1:= float64(i)*env.cellsize
			y1:= float64(j)*env.cellsize 

			if env.grid[i][j] == 1 {
				gc.SetFillColor(color.RGBA{0x88, 0x88, 0x00, 0xff})
			} else {
				gc.SetFillColor(color.RGBA{0x00, 0x00, 0x88, 0xff})
			}

			gc.SetStrokeColor(color.RGBA{0x00, 0x00, 0x00, 0x00})


			if env.vis[i][j] == 1 {
				gc.SetStrokeColor(color.RGBA{0x88, 0x00, 0x00, 0xff})
			}

			if env.cover[i][j] == 1 {
				gc.SetStrokeColor(color.RGBA{0xff, 0x00, 0x00, 0xff})
			}


			gc.MoveTo(x1,y1)
			gc.LineTo(x1+env.cellsize,y1)
			gc.LineTo(x1+env.cellsize,y1+env.cellsize)
			gc.LineTo(x1,y1+env.cellsize)
			gc.LineTo(x1,y1)
			gc.Close()
			gc.FillStroke()

			


		}
	}

	f, _ := os.Create(name)
	png.Encode(f, img)	

}


func (env *orthophoto_env) Sense(loc [3]float64, time float64, action string) interface{} {

	ii := int((loc[0] + 400.0) / env.cellsize)
	jj := int((loc[1] + 400.0) / env.cellsize)

	if ii < 0 || jj < 0 || ii>= env.size || jj >= env.size {

	} else {
		env.cover[ii][jj] = 1 
	}

	if action == "sense" {
		//ret := make(map[[3]float64]int)
		var ret [][3]float64
		for i:= ii-2; i<ii+3;i ++ {
			for j:= jj-2; j<jj+3; j++ {
				if i < 0 || j<0 || i>=env.size || j>= env.size {
					//ret[[3]float64{(float64(i)+0.5) * env.cellsize, (float64(j)+0.5) * env.cellsize, 0.0}] = 0
					
				} else {
					//ret[[3]float64{(float64(i)+0.5) * env.cellsize, (float64(j)+0.5) * env.cellsize, 0.0}] = env.grid[i][j]
					ret = append(ret, [3]float64{(float64(i)+0.5) * env.cellsize, (float64(j)+0.5) * env.cellsize, 0.0})
					env.vis[i][j] = 1

				}


				 
			}
		}

		return ret 
	}

	

	return nil
}




