FROM rezenders/jason-ros

RUN apt-get update && apt-get install -y \
    ros-noetic-turtlesim \
    && rm -rf /var/lib/apt/lists/

COPY * /turtlesim/
WORKDIR /turtlesim

CMD ["bash"]
