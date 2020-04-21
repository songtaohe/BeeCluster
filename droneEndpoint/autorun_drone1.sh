sleep 5
cd /home/pi/Code/droneEndpoint
python cameraserver.py &
go run *.go 1
