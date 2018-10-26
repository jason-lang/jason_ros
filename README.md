# jason-ros
ROS package for jason

## Usage

Build image
```
$ docker build --tag jason-ros:melodic .
```
Create network:
```
$ docker network create ros_net
```

Container 1:

```
$ docker run -it --rm --net ros_net  --name master --env ROS_HOSTNAME=master --env ROS_MASTER_URI=http://master:11311 jason-ros:melodic roslaunch rosbridge_server rosbridge_websocket.launch address:=master
```

Container 2:
```
$ docker run -it --rm  --net ros_net  --name agent --env ROS_HOSTNAME=agent --env ROS_MASTER_URI=http://master:11311 jason-ros:melodic jason rosbridge_agents.mas2j   
```

Container 3:
```
$ docker run -it --rm  --net ros_net  --name echo --env ROS_HOSTNAME=echo --env ROS_MASTER_URI=http://master:11311 jason-ros:melodic
```

Then:
```
$ rostopic pub -1 /jason/percepts std_msgs/String teste
```
## Customization

In order to customize the agent application you can put your agent code inside rosbridge_agents folder or alter the following lines:

```
## Copy Jason files
COPY rosbridge_agents /rosbridge_agents
WORKDIR /rosbridge_agents

RUN ["/bin/bash","-c","mkdir -p lib && cp /java_rosbridge/target/java_rosbridge-2.0.2-jar-with-dependencies.jar lib/"]
```
When pulling the image from Dockerhub just modify the lines above and add to your Dockerfile

## More info
Dockerhub: https://hub.docker.com/r/rezenders/jason-ros/
