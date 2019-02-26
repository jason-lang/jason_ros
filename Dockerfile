FROM rezenders/beaglebone-black-ubuntu-openjdk-ros

COPY qemu-arm-static /usr/bin

# Install packages
RUN apt-get update && apt-get install -y \
	git \
	gradle \
	maven \
	ros-melodic-rosbridge-server \
	&& rm -rf /var/lib/apt/lists/

# Download, install and configure Jason
WORKDIR /
RUN ["/bin/bash", "-c", "git clone https://github.com/jason-lang/jason.git"]
WORKDIR /jason
RUN ["/bin/bash", "-c", "gradle config --info"]
ENV JASON_HOME=/jason/build
ENV PATH=$JASON_HOME/scripts:$PATH

# Download, install and configure java_rosbridge
WORKDIR /
RUN ["/bin/bash","-c","git clone https://github.com/Rezenders/java_rosbridge.git"]
WORKDIR java_rosbridge
RUN ["/bin/bash","-c","mvn compile && \
                       mvn package && \
                       mvn install"]

## Copy Jason files
COPY rosbridge_agents /rosbridge_agents
WORKDIR /rosbridge_agents

RUN ["/bin/bash","-c","mkdir -p lib && cp /java_rosbridge/target/java_rosbridge-2.0.2-jar-with-dependencies.jar lib/"]

CMD ["bash"]
