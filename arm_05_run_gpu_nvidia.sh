xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="arm/lab05"
ROS_CONTAINER="ARM_05"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

XAUTH=/tmp/.docker.xauth
 if [ ! -f $XAUTH ]
 then
     xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
     if [ ! -z "$xauth_list" ]
     then
         echo $xauth_list | xauth -f $XAUTH nmerge -
     else
         touch $XAUTH
     fi
     chmod a+r $XAUTH
 fi

docker stop $ROS_CONTAINER || true && docker rm $ROS_CONTAINER || true

docker run -it \
    --gpus all \
    --runtime=nvidia \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --volume="$SCRIPT_DIR/src/search_interfaces:/arm_ws/src/search_interfaces" \
    --volume="$SCRIPT_DIR/src/search_robot:/arm_ws/src/search_robot" \
    --privileged \
    --network=host \
    --name="$ROS_CONTAINER" \
    $ROS_IMAGE \
    /bin/bash
