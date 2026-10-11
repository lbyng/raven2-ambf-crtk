FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-dev \
    python3-numpy \
    libasound2-dev \
    libgl1-mesa-dev \
    xorg-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
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
    ros-noetic-roscpp \
    ros-noetic-std-msgs \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-catkin \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:0.13.0 /uv /uvx /usr/local/bin/

WORKDIR /ws
RUN mkdir -p /ws/catkin_ws/src

RUN git clone -b ambf-2.0 https://github.com/WPI-AIM/ambf.git /opt/ambf \
    && git clone https://github.com/uw-biorobotics/raven2.git /ws/catkin_ws/src/raven2

RUN ln -s /opt/ambf/ambf_ros_modules /ws/catkin_ws/src/ambf_ros_modules

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd /opt/ambf \
    && mkdir -p build \
    && cd build \
    && cmake .. \
    && make -j\"$(nproc)\""

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /ws/catkin_ws && catkin_make"

# Copied after the AMBF/catkin builds so code changes reuse their cached layers. The adapter is the
# catkin package in adapter/, so the workspace is built again (only it compiles).
COPY . /ws/catkin_ws/src/raven2-ambf-crtk
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /ws/catkin_ws && catkin_make"
RUN cd /ws/catkin_ws/src/raven2-ambf-crtk && uv sync --frozen

ENV AMBF_ROOT=/opt/ambf
ENV PYTHONPATH=/opt/ambf/ambf_ros_modules/ambf_client/python:$PYTHONPATH

RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc \
    && echo "source /ws/catkin_ws/devel/setup.bash" >> /etc/bash.bashrc

COPY docker/entrypoint.sh /ws/entrypoint.sh
RUN chmod +x /ws/entrypoint.sh

ENTRYPOINT ["/ws/entrypoint.sh"]
