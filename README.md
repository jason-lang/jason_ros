# jason-ros
## Description
This repository is being used to develop a possible integration between Jason and ROS.

This is being done by customizing the agent architecture, AgArch class, to include ROS functionalities, using [rosjava](http://wiki.ros.org/rosjava). Also, an intermediary ros node, called HwBridge, is being used to serve as bridge between Jason and Hardware nodes.

Basically, the Jason agent publishes the action it wants to perform into the topic '/jason/actions' and it subscribes to the topic '/jason/actions_status'  to receive the status of the action sent and to the topic '/jason/percepts'/  to receive new perceptions.

In other hand, HwBridge node subscribes to '/jason/actions'  and using the information contained in the [action_manifest](https://github.com/Rezenders/jason-ros/blob/master/jason_ws/src/hw_bridge/src/actions_manifest) it translates the action received and sends it to the right topic/service, when the action is completed it publishes its status into '/jason/actions_status'  topic. Also, it publishes perceptions into '/jason/percepts'  topic according to what is defined in [perception_manifest](https://github.com/Rezenders/jason-ros/blob/master/jason_ws/src/hw_bridge/src/perceptions_manifest).

Docker images containing this repo and needed environment can be found at [dockerhub](https://cloud.docker.com/u/rezenders/repository/docker/rezenders/jason-ros).

armv7 version at branch [armv7](https://github.com/Rezenders/jason-ros/tree/armv7) (out of date)

## Usage


### Running Bare Metal

#### Dependencies
ros - this project was tested with ros melodic but it should work with different versions

java 

gradle

#### Running
Clone this repo:
```
$ git clone https://github.com/Rezenders/jason-ros.git
```

Initialize roscore:
```
$ roscore
```

HwBridge node:
```
$ cd jason_ws/src/hw_bridge/src
$ ./hw_bridge.py
```

Jason node:
```
$ cd rosjava_agents
$ gradle
```

### Running with Docker 

Build image
```
$ docker build --tag jason-ros .
```
Create network:
```
$ docker network create ros_net
```

Container 1:

```
$ docker run -it --rm --net ros_net  --name master --env ROS_HOSTNAME=master --env ROS_MASTER_URI=http://master:11311 ros:melodic-ros-core roscore
```

Container 2:
```
$ docker run -it --rm  --net ros_net  --name agent --env ROS_HOSTNAME=agent --env ROS_MASTER_URI=http://master:11311 jason-ros   
```
Inside container at the agent folder:
```
$ gradle
```

Container 3:
```
$ docker run -it --rm  --net ros_net  --name echo --env ROS_HOSTNAME=echo --env ROS_MASTER_URI=http://master:11311 jason-ros
```

Then, at /jason_ws/src/hw_bridge/src folder:
```
$ ./hw_bridge.py
```
## Customization

In order to use this project, one must modify the [action_manifest](https://github.com/Rezenders/jason-ros/blob/master/jason_ws/src/hw_bridge/src/actions_manifest) and [perception_manifest](https://github.com/Rezenders/jason-ros/blob/master/jason_ws/src/hw_bridge/src/perceptions_manifest) to include the information about the actions being performed and the perceptions of interest. Also, one must include [RosArch](https://github.com/Rezenders/jason-ros/blob/master/rosjava_agents/src/java/RosArch.java), [RosJasonNode](https://github.com/Rezenders/jason-ros/blob/master/rosjava_agents/src/java/RosJasonNode.java) into the jason src/java/ directory, and [jason_msgs.jar](https://github.com/Rezenders/jason-ros/blob/master/rosjava_agents/lib/jason_msgs.jar) into the jason lib/ directory (this will be improved by generating one .jar that already contains all 3 dependencies).

### Manifests Info
action_manifest:
```
[teste]
method = topic 
name = /hw/teste
msg_type = String
dependencies = std_msgs.msg
params_name = data
```
method - topic or service

name - name of the topic or service 

msg_type - type of the message e.g. String, Bool, Int32

dependencies - python module which contains the message type e.g. std_msgs.msg, mavros_msgs.msg

params_name - name of the parameter being sent

params_type - type of the parameter e.g. bool, str, int

For more examples of actions you can take a look at this [action_manifest](https://github.com/Rezenders/MAS-UAV/blob/master/MiddleNode/actions_manifest)

perception_manifest:
```
[state]
name = /hw/teste2
msg_type = String
dependencies = std_msgs.msg
args = data
buf = add
```
name - name of the topic or service 

msg_type - type of the message e.g. String, Bool, Int32

dependencies - python module which contains the message type e.g. std_msgs.msg, mavros_msgs.msg

args - fields that you want to perceive

buf (belief update function) - if the perception should be added or updated in the belief base

This perception would be added into the agent belief base as state(data)

For more examples of perceptions you can take a look at this [perceptions_manifest](https://github.com/Rezenders/MAS-UAV/blob/master/MiddleNode/perceptions_manifest)

## Examples
A functional example using docker can be found at the repo [MAS-UAV](https://github.com/Rezenders/MAS-UAV)
