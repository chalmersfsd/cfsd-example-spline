# cfsd-example-spline

## Compile

```
docker build -f Dockerfile.amd64 -t chalmersfsd/cfsd-example-spline-amd64:v0.0.1 .
```

## Run

Start the vehicle viewer and a suitable recording. The example assumes a image resolution of 1344x376, so use a recent recoding from the ZED camera:
```
docker run --rm --init --net=host --name=opendlv-vehicle-view -~/recordings:/opt/vehicle-view/recordings -v /var/run/docker.sock:/var/run/docker.sock -p 8081:8081 chalmersrevere/opendlv-vehicle-view-multi:v0.0.48
```

Then run the provided yml file to start the video decoder and the example itself:
```
docker-compose up
```
