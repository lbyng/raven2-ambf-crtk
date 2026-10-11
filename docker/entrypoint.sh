#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/ambf/build/devel/setup.bash
source /ws/catkin_ws/devel/setup.bash

export AMBF_ROOT=/opt/ambf
export PYTHONPATH=/opt/ambf/ambf_ros_modules/ambf_client/python:${PYTHONPATH}

ROSCORE_LOG=/tmp/roscore.log
AMBf_LOG=/tmp/ambf.log

roscore > "${ROSCORE_LOG}" 2>&1 &
sleep 2

/opt/ambf/bin/lin-x86_64/ambf_simulator -a \
  "/ws/catkin_ws/src/raven2-ambf-crtk/raven_models/raven_straight.yaml" \
  > "${AMBf_LOG}" 2>&1 &

sleep 2

cd /ws/catkin_ws/src/raven2-ambf-crtk
exec uv run --frozen raven2-ambf-crtk
