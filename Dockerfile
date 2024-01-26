FROM ros:foxy

ENV ROS_WS=/ros_ws

WORKDIR $ROS_WS

RUN mkdir -p $ROS_WS/src
RUN cd $ROS_WS/src && \
    git clone -b foxy https://github.com/ros2/ros2.git

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN cd $ROS_WS && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Source ROS2 workspace setup file
RUN echo "source $ROS_WS/install/setup.bash" >> /root/.bashrc

# VNC and ROS2 ports
EXPOSE 5900
EXPOSE 8888

# entry point to start VNC server and ROS2
CMD ["/ros_entrypoint.sh"]
