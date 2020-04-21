package main 

/*
 * This file implements an example proxy to route BeeCluster traffic
 * 
 */

import (
	"net/http"
	"io"
	"fmt"
)

func copyHeader(dst, src http.Header) {
    for k, vv := range src {
        for _, v := range vv {
            dst.Add(k, v)
        }
    }
}

func  DataPlaneHandler(w http.ResponseWriter, r *http.Request) {
	address := "192.168.1.21:7900"

	url := r.URL
    url.Host = address
    url.Scheme = "http"

    r.URL = url 

	resp, err := http.DefaultTransport.RoundTrip(r)
    if err != nil {
    	fmt.Println("httperror", err.Error(), http.StatusServiceUnavailable)
        return 
    }

    defer resp.Body.Close()

    copyHeader(w.Header(), resp.Header)
    w.WriteHeader(resp.StatusCode)
    io.Copy(w, resp.Body)

}



func main() {
  dataPlane := http.NewServeMux()
  dataPlane.HandleFunc("/", DataPlaneHandler)
  http.ListenAndServe(":7900", dataPlane)

}
