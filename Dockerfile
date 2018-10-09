FROM ros:melodic

RUN apt-get update && apt-get install -y \
	vim \
	git \
	default-jdk \
	gradle  \   
	&& rm -rf /var/lib/apt/lists/

ENV JAVA_HOME=/usr/lib/jvm/default-java

WORKDIR /
RUN ["/bin/bash", "-c", "git clone https://github.com/jason-lang/jason.git"]
WORKDIR /jason
RUN ["/bin/bash", "-c", "gradle config"]
ENV JASON_HOME=/jason/build
ENV PATH=$JASON_HOME/scripts:$PATH

# Setup catkin workspace
RUN ["/bin/bash","-c", "source /opt/ros/melodic/setup.bash && \
                  mkdir -p /catkin_ws/src && \
                  cd /catkin_ws/src && \
                  catkin_init_workspace && \
                  cd /catkin_ws/ && \
                  catkin_make "]

WORKDIR /catkin_ws/src/

#Copy package files to catkin workspace
#RUN catkin_create_pkg keyboard_driver
#COPY teleop_ws/src/keyboard_driver /catkin_ws/src/keyboard_driver

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
