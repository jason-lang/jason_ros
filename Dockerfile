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
RUN apt-get update && \
	mkdir -p /rosjava/src && \
	wstool init -j4 /rosjava/src https://raw.githubusercontent.com/rosjava/rosjava/kinetic/rosjava.rosinstall && \
	cd /rosjava && \
	rosdep update && \
	rosdep install --from-paths src -i -y  && \
	rm -rf /var/lib/apt/lists/

# Catkin_make rosjava
RUN ["/bin/bash","-c", "source /opt/ros/kinetic/setup.bash && \
                  cd /rosjava && \
                  catkin_make "]

# Setup catkin workspace
RUN ["/bin/bash","-c", "source /opt/ros/kinetic/setup.bash && \
                  mkdir -p /catkin_ws/src && \
                  cd /catkin_ws/src && \
                  catkin_init_workspace && \
                  cd /catkin_ws/ && \
                  catkin_make "]

WORKDIR /catkin_ws/src/

#Copy package files to catkin workspace
RUN ["/bin/bash", "-c", "source /rosjava/devel/setup.bash && \ 
                         catkin_create_rosjava_pkg jason_agents"]
#COPY teleop_ws/src/keyboard_driver /catkin_ws/src/keyboard_driver

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
