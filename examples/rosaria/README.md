## Running with Docker

First, at the root of this project(jason-ros) build jason-ros image:
```
$ docker build --tag jason-ros .
```

Then, at this folder build this example image:
```
$ docker build --tag jason-rosaria .
```

Create network:
```
$ docker network create ros_net
```

Then, run all the containers:

Container 1:
```
$ docker run -it --rm --net ros_net  --name master --env ROS_HOSTNAME=master --env ROS_MASTER_URI=http://master:11311 ros:melodic-ros-core roscore
```

Container 2:
```
$ xhost +local:root # for the lazy and reckless
```

```
$ docker run -it --rm --net ros_net --name mobilesim --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  --env ROS_HOSTNAME=mobilesim --env ROS_MASTER_URI=http://master:11311 jason-rosaria MobileSim --nomap -r p3dx   
```

Container 3:
```
sudo  docker run -it --rm --net ros_net --name rosaria --env ROS_HOSTNAME=rosaria --env ROS_MASTER_URI=http://master:11311 jason-rosaria rosrun rosaria RosAria _port:=mobilesim:8101
```

Container 4:
```
$ docker run -it --rm --net ros_net --name hwbridge --env ROS_HOSTNAME=hwbridge --env ROS_MASTER_URI=http://master:11311 jason-rosaria rosrun hw_bridge hw_bridge.py
```

Container 5:
```
$ docker run -it --rm --net ros_net --name jason --env ROS_HOSTNAME=jason --env ROS_MASTER_URI=http://master:11311 jason-rosaria gradle
```
