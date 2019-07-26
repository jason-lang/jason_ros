FROM ros:melodic-ros-core

# Install packages
RUN apt-get update && apt-get install -y \
	vim \
	default-jdk \
	python-pathlib \
	wget \
	unzip \
	&& rm -rf /var/lib/apt/lists/

# Set java home
ENV JAVA_HOME=/usr/lib/jvm/default-java

# Install gradle
RUN ["/bin/bash", "-c", "wget https://services.gradle.org/distributions/gradle-5.5.1-bin.zip -P /tmp && \
                            unzip -d /opt/gradle /tmp/gradle-*.zip"]
ENV GRADLE_HOME=/opt/gradle/gradle-5.5.1
ENV PATH=${GRADLE_HOME}/bin:${PATH}

COPY jason_ws/ /jason_ws
RUN [ "/bin/bash","-c","source /opt/ros/melodic/setup.bash && \
        cd /jason_ws && catkin_make"]

COPY rosjava_agents/ /rosjava_agents/
WORKDIR /rosjava_agents
RUN ["/bin/bash","-c","gradle build"]

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
