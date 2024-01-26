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




# ARG BASE_IMAGE=jupyter/minimal-notebook:python-3.11.5
FROM jupyter/base-notebook:latest

USER root

RUN adduser -D myuser
USER myuser

COPY . /app
COPY --from=builder /usr/local/lib/python3.9/site-packages /usr/local/lib/python3.9/site-packages
COPY --from=builder /usr/local/bin/gunicorn /usr/local/bin/gunicorn
COPY --from=builder /app .

RUN pip install --no-cache-dir -r requirements.txt

RUN apk add --no-cache \
    package1 \
    package2 \
    && rm -rf /var/cache/apk/*

ENV FLASK_APP=app.py \
    FLASK_RUN_HOST=0.0.0.0 \
    FLASK_RUN_PORT=5000

# port
EXPOSE 8888

CMD ["jupyter", "notebook", "--ip='*'", "--port=8888", "--no-browser", "--allow-root"]

COPY mqtt-server-flow.json

RUN pip cache purge 
