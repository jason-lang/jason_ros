FROM ros:noetic-ros-core

# Install packages
RUN apt-get update && apt-get install -y \
	vim \
	wget \
	unzip \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  build-essential \
  openjdk-17-jdk \
	&& rm -rf /var/lib/apt/lists/

RUN rosdep init

# Set java home
ENV JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64

# Install gradle
RUN ["/bin/bash", "-c", "wget https://services.gradle.org/distributions/gradle-8.1-bin.zip -P /tmp && \
                            unzip -d /opt/gradle /tmp/gradle-*.zip"]
ENV GRADLE_HOME=/opt/gradle/gradle-8.1
ENV PATH=${GRADLE_HOME}/bin:${PATH}

RUN [ "/bin/bash","-c","source /opt/ros/noetic/setup.bash && \
        mkdir -p /jason_ros_ws/src && \
        cd /jason_ros_ws/src && catkin_init_workspace"]

COPY jason_ros/ /jason_ros_ws/src/jason_ros/jason_ros/
COPY jason_ros_msgs/ /jason_ros_ws/src/jason_ros/jason_ros_msgs/
COPY jason_ros_comm/ /jason_ros_ws/src/jason_ros/jason_ros_comm/

WORKDIR /jason_ros_ws/
RUN [ "/bin/bash","-c","source /opt/ros/noetic/setup.bash \
            && apt update && rosdep update \
            && rosdep install --from-paths src --ignore-src -r -y \
            && rm -rf /var/lib/apt/lists/"]

RUN [ "/bin/bash","-c","source /opt/ros/noetic/setup.bash && \
        cd /jason_ros_ws && catkin_make"]

COPY examples/ /jason_ros_ws/src/jason_ros/examples/

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
