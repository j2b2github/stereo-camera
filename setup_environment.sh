# So that docker can see the webcam, XWindows
echo "Setting environment variables for OpenCV application development"
xhost +local:docker
XSOCK=/tmp/.X11-unix
IMG_TAG=trt21.10_cv4.7.0:0.1

# Build the dockerfile for the engironment
if [ ! -z $(docker images -q $IMG_TAG) ]; then
	echo "Dockerfile has already been built"
else
	echo "Building docker image"
	docker build -f dockerfiles/Dockerfile --tag=$IMG_TAG .
fi

# Start the docker container

# stereo vision
echo "Starting docker container with stereo-vision"
docker run -it \
--gpus all \
-v `pwd`:/mnt \
--device /dev/video0 \
--device /dev/video1 \
--device /dev/video2 \
--device /dev/video3 \
--device /dev/video4 \
--device /dev/video5 \
-e DISPLAY=$DISPLAY \
-v $XSOCK:$XSOCK \
-e NO_AT_BRIDGE=1 \
-e QT_X11_NO_MITSHM=1 \
$IMG_TAG
