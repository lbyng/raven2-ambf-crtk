# RAVEN-II AMBF CRTK Framework

Runs the **RAVEN-II** surgical robot in the **AMBF** simulator behind a **CRTK API**, with a real-RAVEN-style `raven_state` topic. The adapter loop and its topics run at **500 Hz**; AMBF runs the joint controllers at 1 kHz.

> ⚠ Only the **left arm** (GOLD arm) is supported.

![Flow Chart](./fig/flow-chart.png)

| Topic | Type | Direction |
|---|---|---|
| `/arm1/measured_js` | `sensor_msgs/JointState` | published |
| `/arm1/measured_cp` | `geometry_msgs/TransformStamped` | published |
| `/arm1/servo_jr` | `sensor_msgs/JointState` (joint deltas in `position`) | subscribed |
| `/arm1/servo_cr` | `geometry_msgs/TransformStamped` (relative pose) | subscribed |
| `/ambf/raven_state` | `raven_2/raven_state` | published |

## Native setup (Ubuntu 20.04 + ROS Noetic)

**1. AMBF 2.0** — build dependencies are listed in the `apt-get install` step of the [Dockerfile](./Dockerfile).

```bash
source /opt/ros/noetic/setup.bash
git clone -b ambf-2.0 https://github.com/WPI-AIM/ambf.git ~/ambf
mkdir -p ~/ambf/build && cd ~/ambf/build
cmake .. && make -j$(nproc)
```

**2. Catkin workspace** with the [RAVEN-II package](https://github.com/uw-biorobotics/raven2) (for `raven_2` messages) and AMBF's ROS modules:

```bash
mkdir -p ~/ros_ambf_ws/src && cd ~/ros_ambf_ws/src
git clone https://github.com/uw-biorobotics/raven2.git
ln -s ~/ambf/ambf_ros_modules ambf_ros_modules
cd ~/ros_ambf_ws && catkin_make
echo "source ~/ros_ambf_ws/devel/setup.bash" >> ~/.bashrc
```

**3. This repository**, with its Python environment managed by [uv](https://docs.astral.sh/uv/):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
git clone https://github.com/lbyng/raven2-ambf-crtk.git ~/raven2-ambf-crtk
cd ~/raven2-ambf-crtk && uv sync
```

`uv sync` builds `.venv` on the system Python 3.8 that ROS Noetic uses. ROS packages (`rospy`, `tf`, `ambf_client`, ...) are not installed into it; they come from your sourced ROS workspace.

**4. (Optional) alias** for launching AMBF with this repo's RAVEN-II model (base-link parallel to the ground):

```bash
echo "alias ambf-raven2='~/ambf/bin/lin-x86_64/ambf_simulator -a ~/raven2-ambf-crtk/raven_models/raven_straight.yaml'" >> ~/.bashrc
```

## Run

Each in its own terminal:

```bash
roscore
ambf-raven2
cd ~/raven2-ambf-crtk && uv run main.py                 # once AMBF has loaded the model
cd ~/raven2-ambf-crtk && uv run keyboard_controller.py  # optional
```

The keyboard controller takes one key at a time; per-joint velocities are set at the top of `keyboard_controller.py`.

## Docker

Builds AMBF 2.0, the workspace and this repository, then runs `roscore`, the AMBF GUI and `main.py` in one container.

Host requirements: [Docker Engine with Compose v2](https://docs.docker.com/engine/install/ubuntu/), the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), and an X server.

```bash
cd ~/raven2-ambf-crtk
xhost +local:root
docker compose up --build
```

Keyboard controller, from a second terminal in the repo:

```bash
docker compose exec raven2-ambf bash -ic "cd /ws/catkin_ws/src/raven2-ambf-crtk && uv run --frozen keyboard_controller.py"
```

AMBF logs to `/tmp/ambf.log` inside the container. The container uses host networking, so the topics are also visible from the host.

## Contact

Questions and discussions are welcome via GitHub Issues or by email to the maintainers.
