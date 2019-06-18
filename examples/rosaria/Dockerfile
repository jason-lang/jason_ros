FROM jason-ros

WORKDIR /
RUN ["/bin/bash","-c", "git clone https://github.com/reedhedges/AriaCoda.git && cd AriaCoda && make"]

WORKDIR /jason_ws/src
RUN ["/bin/bash","-c","git clone https://github.com/amor-ros-pkg/rosaria.git"]

WORKDIR /jason_ws
RUN ["/bin/bash", "-c","apt-get update && rosdep install --from-paths src --ignore-src --rosdistro=melodic -r -y && rm -rf /var/lib/apt/lists"]

RUN ["/bin/bash","-c","source /opt/ros/melodic/setup.bash && \
        cd /jason_ws && catkin_make"]

COPY mobilesim.deb /mobilesim/
RUN ["/bin/bash","-c","dpkg -i /mobilesim/mobilesim.deb"]

COPY * /rosaria/
WORKDIR /rosaria
