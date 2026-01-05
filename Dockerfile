FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-dev \
    python3-pip \
    python3-numpy \
    libglu1-mesa-dev \
    freeglut3-dev \
    libglew-dev \
    libxmu-dev \
    libxi-dev \
    libopenal-dev \
    libalut-dev \
    libvorbis-dev \
    libogg-dev \
    libassimp-dev \
    libglfw3-dev \
    libyaml-cpp-dev \
    libbullet-dev \
    ros-noetic-tf \
    ros-noetic-rospy \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-catkin \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
RUN mkdir -p /ws/catkin_ws/src

COPY . /ws/catkin_ws/src/raven2-ambf-crtk

RUN git clone https://github.com/WPI-AIM/ambf.git /opt/ambf \
    && git clone https://github.com/uw-biorobotics/raven2.git /ws/catkin_ws/src/raven2

RUN cd /opt/ambf \
    && mkdir -p build \
    && cd build \
    && cmake .. \
    && make -j"$(nproc)"

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /ws/catkin_ws && catkin_make"

RUN cp -r /ws/catkin_ws/src/raven2-ambf-crtk/raven_models/* \
    /opt/ambf/ambf_models/meshes/blender_afmb/raven_2/

ENV AMBF_ROOT=/opt/ambf
ENV PYTHONPATH=/opt/ambf/ambf_ros_modules/ambf_client/python:$PYTHONPATH

COPY docker/entrypoint.sh /ws/entrypoint.sh
RUN chmod +x /ws/entrypoint.sh

ENTRYPOINT ["/ws/entrypoint.sh"]
