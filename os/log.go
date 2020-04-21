package main 

/*
 * This file implements logging functions
 * 
 */

import (
	"fmt"
	"strings"
	"sync"
	"os"
)
const (
	LogError = 0
	LogWarning = 1 
	LogInfo = 2
	LogDebug = 3 
)

var LogLevel = 3 
var logLock sync.Mutex
// const (
//         InfoColor    = "\033[1;34m%s\033[0m"
//         NoticeColor  = "\033[1;36m%s\033[0m"
//         WarningColor = "\033[1;33m%s\033[0m"
//         ErrorColor   = "\033[1;31m%s\033[0m"
//         DebugColor   = "\033[0;36m%s\033[0m"
// )

var LogName []string = []string{"\033[1;31m[Error]\033[0m", "\033[1;33m[Warning]\033[0m","\033[1;34m[Info]\033[0m","\033[1;36m[Debug]\033[0m"}

func PrintLog(logtype int, content string) {
	if logtype <= LogLevel {
		s := makeTimestampString()+": "+ LogName[logtype] +" "+ strings.TrimSuffix(content,"\n")
		fmt.Println(s)
		logLock.Lock()
		f, _ := os.OpenFile("log.txt", os.O_APPEND|os.O_RDWR|os.O_CREATE, 0666)
		f.WriteString(s + "\n")
		f.Close()
		logLock.Unlock()

	} 
}