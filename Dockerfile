FROM ros:melodic-ros-core

# Install packages
RUN apt-get update && apt-get install -y \
	vim \
	default-jdk \
	gradle  \
	python-pathlib \
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

COPY rosjava_agents/ /rosjava_agents/
WORKDIR /rosjava_agents
RUN ["/bin/bash","-c","gradle build"]

COPY jason_ws/ /jason_ws
RUN [ "/bin/bash","-c","source /opt/ros/melodic/setup.bash && \
                        cd /jason_ws && catkin_make"]

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
