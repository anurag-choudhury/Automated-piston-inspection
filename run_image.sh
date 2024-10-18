#/bin/bash
xhost +local:docker
    docker run --rm -it --net=host --name piston_container --volume /tmp/.X11-unix:/tmp/.X11-unix --volume /home/anurag/Desktop/code/Piston_fsm:/fsm_piston --workdir /fsm_piston --env DISPLAY=$DISPLAY --env ROS_MASTER_URI=http://localhost:11311/ --privileged -v /dev/bus/usb:/dev/bus/usb --env ROS_HOSTNAME=192.168.0.101 --env ROS_IP=192.168.0.100 mr_fsm  /bin/bash 