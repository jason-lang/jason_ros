## Dependencies
[turtlesim](http://wiki.ros.org/turtlesim)

## Running Bare Metal

Start roscore:
```
$ roscore
```

Start turtlesim:

```
$ rosrun turtlesim turtlesim_node
```

Start HwBridge, must be in this folder:
```
$ rosrun hw_bridge hw_bridge.py
```

Note: Remember to run ```catkin_make``` and to source hw_bridge

Start jason agent:
```
$ cd turtlesim_agent
$ gradle
```

## Running with Docker

First, at the root of this project(jason-ros) build jason-ros image:
```
$ docker build --tag jason-ros .
```

Then, at this folder build this example image:
```
$ docker build --tag jason-turtle .
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

````
$ docker run -it --rm --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --name turtlesim --net ros_net --env ROS_HOSTNAME=turtlesim --env ROS_MASTER_URI=http://master:11311 jason-turtle rosrun turtlesim turtlesim_node
````

Container 3:
```
$ docker run -it --rm --net ros_net --name hwbridge --env ROS_HOSTNAME=hwbridge --env ROS_MASTER_URI=http://master:11311 jason-turtle rosrun hw_bridge hw_bridge.py
```

Container 4:
```
$ docker run -it --rm --net ros_net --name jason --env ROS_HOSTNAME=jason --env ROS_MASTER_URI=http://master:11311 jason-turtle gradle
```

If you want to inspect what is happening run an extra container and then use rostopic etc:
```
$ docker run -it --rm --net ros_net --name echo --env ROS_HOSTNAME=echo --env ROS_MASTER_URI=http://master:11311 jason-turtle
```
