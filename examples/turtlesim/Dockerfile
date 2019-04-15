FROM jason-ros

RUN apt-get update && apt-get install -y \
    ros-melodic-turtlesim \
    && rm -rf /var/lib/apt/lists/

COPY * /turtlesim/
WORKDIR /turtlesim

CMD ["bash"]
