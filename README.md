# jason-ros
ROS package for jason

## Rosbridge

Build image
```
$ docker build --tag ros_bridge .
```
Create network:
```
$ docker network create ros_net
```

Container 1:

```
$ docker run -it --rm --net ros_net  --name master --env ROS_HOSTNAME=master --env ROS_MASTER_URI=http://master:11311 ros_bridge roslaunch rosbridge_server rosbridge_websocket.launch address:=master
```

Container 2:
```
$ docker run -it --rm  --net ros_net  --name agent --env ROS_HOSTNAME=agent --env ROS_MASTER_URI=http://master:11311 ros_bridge jason rosbridge_agents.mas2j   
```

Container 3:
```
$ docker run -it --rm  --net ros_net  --name echo --env ROS_HOSTNAME=echo --env ROS_MASTER_URI=http://master:11311 ros_bridge
```
Then:
```
$ rostopic pub -1 /jason/percepts std_msgs/String teste
```
