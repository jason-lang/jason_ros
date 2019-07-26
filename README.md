# jason-ros
## Description
This repository is being used to develop a possible integration between Jason and ROS.

This is being done by customizing the agent architecture, AgArch class, to include ROS functionalities, using [rosjava](http://wiki.ros.org/rosjava). Also, an intermediary ros node, called HwBridge, is being used to serve as bridge between Jason and Hardware nodes.

Basically, the Jason agent publishes the action it wants to perform into the topic '/jason/actions' and it subscribes to the topic '/jason/actions_status'  to receive the status of the action sent and to the topic '/jason/percepts'/  to receive new perceptions.

In other hand, HwBridge node subscribes to '/jason/actions'  and using the information contained in the [action_manifest](https://github.com/Rezenders/jason-ros/blob/master/jason_ws/src/hw_bridge/src/actions_manifest) it translates the action received and sends it to the right topic/service, when the action is completed it publishes its status into '/jason/actions_status'  topic. Also, it publishes perceptions into '/jason/percepts'  topic according to what is defined in [perception_manifest](https://github.com/Rezenders/jason-ros/blob/master/jason_ws/src/hw_bridge/src/perceptions_manifest).

Docker images containing this repo and needed environment can be found at [dockerhub](https://cloud.docker.com/u/rezenders/repository/docker/rezenders/jason-ros).

armv7 version at branch [armv7](https://github.com/Rezenders/jason-ros/tree/armv7) (out of date)

## Usage

### mas2j
Use jasonros.RosArch

```
MAS rosbridge_agents {

    infrastructure: Centralised

    agents: sample_agent agentArchClass jasonros.RosArch;
}
```

### Gradle
Include jason, jasonros and jasonros_msgs to gradle:

```
repositories {
   mavenCentral()
   jcenter()

   maven { url "https://raw.github.com/rosjava/rosjava_mvn_repo/master" }
   maven { url "https://raw.github.com/jason-lang/mvn-repo/master" }
   maven { url "http://jacamo.sourceforge.net/maven2" }
}

dependencies {
   compile group: 'org.jason-lang',     name: 'jason' ,   version: '2.4-SNAPSHOT'
   compile group: 'org.jason-lang',     name: 'jasonros' ,   version: '1.0'
   compile group: 'org.jason-lang',     name: 'jasonros_msgs' ,   version: '1.0'
}
```

### Edit Manifests

In order to use this project, one must modify the [action_manifest](https://github.com/Rezenders/jason-ros/blob/master/jason_ws/src/hw_bridge/src/actions_manifest) and [perception_manifest](https://github.com/Rezenders/jason-ros/blob/master/jason_ws/src/hw_bridge/src/perceptions_manifest) to include the information about the actions being performed and the perceptions of interest.

action_manifest:
```
[teste]
method = topic
name = /hw/teste
msg_type = String
dependencies = std_msgs.msg
params_name = data
params_type = str
latch = True
```
method - topic or service

name - name of the topic or service

msg_type - type of the message e.g. String, Bool, Int32

dependencies - python module which contains the message type e.g. std_msgs.msg, mavros_msgs.msg

params_name - name of the parameter being sent

params_type - type of the parameter e.g. bool, str, int. If not inclued the default type is str

latch - if the action should be latched e.g. true, True, yes, 1, False, false. If not included the default is true

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

buf (belief update function) - if the perception should be added or updated in the belief base. If not inclued the default option is update

This perception would be added into the agent belief base as state(data)

For more examples of perceptions you can take a look at this [perceptions_manifest](https://github.com/Rezenders/MAS-UAV/blob/master/MiddleNode/perceptions_manifest)

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

Before running HwBridge node jason_ws must be built and sourced:
```
$ cd jason_ws
$ catkin_make
$ source devel/setup.bash
```

If you don't want to source everytime you open a new bash, do something like:
```
echo "source ~/jason-ros/jason_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Note: Change "~/jason-ros/jason_ws/devel/setup.bash" for the right path in your computer

HwBridge node:
```
$ cd jason_ws/src/hw_bridge/src
$ ./hw_bridge.py
```

In order to inspect what is being exchanged between the nodes you can use rostopic echo/info to inspect the topics /jason/actions, /jason/actions_status, /jason/percepts, /hw/teste or /hw/teste2:
```
$ rostopic echo /jason/actions      #or any other topic
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
$ docker run -it --rm  --net ros_net  --name hwbridge --env ROS_HOSTNAME=hwbridge --env ROS_MASTER_URI=http://master:11311 jason-ros
```

Then, at /jason_ws/src/hw_bridge/src folder:
```
$ ./hw_bridge.py
```

Container 3:

In order to inspect what is being exchanged between the nodes you can use rostopic echo/info to inspect the topics /jason/actions, /jason/actions_status, /jason/percepts, /hw/teste or /hw/teste2:
```
$ docker run -it --rm  --net ros_net  --name echo --env ROS_HOSTNAME=echo --env ROS_MASTER_URI=http://master:11311 jason-ros
```
Then:
```
$ rostopic echo /jason/actions      #or any other topic
```

Container 4:
```
$ docker run -it --rm  --net ros_net  --name agent --env ROS_HOSTNAME=agent --env ROS_MASTER_URI=http://master:11311 jason-ros   
```
Inside container at the agent folder:
```
$ gradle
```
## Developers Note
In order to generate jason_msgs.jar run:

```
$ docker build -t jason-msg . -f MsgDockerfile
```

```
$ docker run -it --rm -v ${PWD}/rosjava_agents/lib/:/artifacts jason-msg cp /jason_ws/build/jason_msgs/java/jason_msgs/build/libs/jason_msgs-0.0.0.jar /artifacts/jason_msgs.jar
```
## Examples
A functional example using docker can be found at the repo [MAS-UAV](https://github.com/Rezenders/MAS-UAV)
For more examples check the example folder
