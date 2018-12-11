FROM ros:kinetic-ros-core

# Install packages
RUN apt-get update && apt-get install -y \
	vim \
	default-jdk \
	gradle  \
	maven \
	ros-kinetic-catkin \
	ros-kinetic-rospack \
	python-wstool \
	&& rm -rf /var/lib/apt/lists/

# Set java home
ENV JAVA_HOME=/usr/lib/jvm/default-java

# Download, install and configure Jason
WORKDIR /
RUN ["/bin/bash", "-c", "git clone https://github.com/jason-lang/jason.git"]
WORKDIR /jason
RUN ["/bin/bash", "-c", "gradle config"]
ENV JASON_HOME=/jason/build
ENV PATH=$JASON_HOME/scripts:$PATH


RUN apt-get update && apt-get install -y \
	ros-kinetic-world-canvas-msgs \
	ros-kinetic-concert-service-msgs \
	ros-kinetic-ar-track-alvar-msgs \
	ros-kinetic-gateway-msgs \
	ros-kinetic-rocon-device-msgs \
	ros-kinetic-rocon-app-manager-msgs \
	ros-kinetic-scheduler-msgs \
	ros-kinetic-rocon-tutorial-msgs \
	ros-kinetic-rocon-interaction-msgs \
	ros-kinetic-yocs-msgs \
	ros-kinetic-concert-msgs \
	ros-kinetic-move-base-msgs \
	ros-kinetic-tf2-msgs \
	&& rm -rf /var/lib/apt/lists/

# Download, install and configure rosjava
WORKDIR /
RUN ["/bin/bash","-c","mkdir -p ~/rosjava/src && wstool init -j4 ~/rosjava/src https://raw.githubusercontent.com/rosjava/rosjava/kinetic/rosjava.rosinstall && source /opt/ros/kinetic/setup.bash && cd ~/rosjava/ && rosdep update && rosdep install --from-paths src -i -y && catkin_make"]


CMD ["bash"]
