package main 

/*
 * Simulator Environment
 * 
 */
 
import (
	"math/rand"
)
type wifi_env struct {
	// without any states
}

func MakeWifiEnv() *wifi_env{
	env := new(wifi_env)

	return env 
}

func (env *wifi_env) UpdateBackground(time float64, name string) {
	// do nothing here ...
}

func (env *wifi_env) Sense(loc [3]float64, time float64, action string) interface{} {
	d := distanceRaw(loc, [3]float64{-10.0, 10.0, 0.0})

	if d < 10.0 {
		d = 10.0 + (10.0 - d) * 1.0 
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

	return signal
}