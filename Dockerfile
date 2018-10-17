FROM ros:kinetic

# Install packages
RUN apt-get update && apt-get install -y \
	vim \
	git \
	default-jdk \
	gradle  \
	maven \
	ros-kinetic-rosbridge-server \
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

# Download, install and configure java_rosbridge
WORKDIR /
RUN ["/bin/bash","-c","git clone https://github.com/h2r/java_rosbridge.git"]
WORKDIR java_rosbridge
RUN ["/bin/bash","-c","mvn compile && \
                       mvn package && \
                       mvn install"]
ENV CLASSPATH="/java_rosbridge/target/java_rosbridge-2.0.2-jar-with-dependencies.jar:${CLASSPATH}"

## Setup catkin workspace
RUN ["/bin/bash","-c", "source /opt/ros/kinetic/setup.bash && \
                  mkdir -p /catkin_ws/src && \
                  cd /catkin_ws/src && \
                  catkin_init_workspace && \
                  cd /catkin_ws/ && \
                  catkin_make "]

WORKDIR /catkin_ws/src/



COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
