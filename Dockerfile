FROM ros:kinetic

# Install packages
RUN apt-get update && apt-get install -y \
	vim \
	git \
	default-jdk \
	gradle  \   
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

# Install rosjava
RUN apt update && \
	apt install -y  \
	ros-kinetic-rosjava && \
	rm -rf /var/lib/apt/lists/

# Setup catkin workspace
RUN ["/bin/bash","-c", "source /opt/ros/kinetic/setup.bash && \
                  mkdir -p /catkin_ws/src && \
                  cd /catkin_ws/src && \
                  catkin_init_workspace && \
                  cd /catkin_ws/ && \
                  catkin_make "]

WORKDIR /catkin_ws/src/

#Copy package files to catkin workspace
RUN ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && \ 
                        catkin_create_rosjava_pkg jason_agents"]



COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
