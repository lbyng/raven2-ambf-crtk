# RAVEN-II AMBF CRTK Framework

Runs the **RAVEN-II** surgical robot in the **AMBF** simulator behind the **CRTK interface of the real RAVEN-II** (the `crtk` branch of [raven2](https://github.com/uw-biorobotics/raven2)), including its `/ravenstate` topic. The adapter is a C++ ROS node that runs at **1 kHz**, the control rate of the real RAVEN and of the AMBF joint controllers.

> ⚠ Only the **left arm** (GOLD arm: `arm1`, `grasp1`) is supported.

![Flow Chart](./fig/flow-chart.png)

| Topic | Type | Direction |
|---|---|---|
| `/arm1/measured_js`, `/arm1/setpoint_js` | `sensor_msgs/JointState` | published |
| `/arm1/measured_cp`, `/arm1/setpoint_cp` | `geometry_msgs/TransformStamped` | published |
| `/grasp1/measured_js` | `sensor_msgs/JointState` (grasp angle, jaw 1, jaw 2) | published |
| `/arm1/operating_state`, `/operating_state` | `crtk_msgs/operating_state` | published |
| `/ravenstate` | `raven_2/raven_state` | published |
| `/arm1/servo_jr`, `/arm1/servo_jp`, `/arm1/servo_jv` | `sensor_msgs/JointState` | subscribed |
| `/arm1/servo_cr`, `/arm1/servo_cp`, `/arm1/servo_cv` | `geometry_msgs/TransformStamped` | subscribed |
| `/grasp1/servo_jr`, `/grasp1/servo_jp`, `/grasp1/servo_jv` | `sensor_msgs/JointState` (grasp angle) | subscribed |
| `/arm1/state_command`, `/state_command` | `crtk_msgs/StringStamped` | subscribed |

As on the real RAVEN:
- **States.** At start-up the arm homes, then waits `PAUSED`. Send `resume` to enable it; `pause`, `disable`, `enable`, `home` and `unhome` also work, with `enable` standing in for the start button. Servo commands move the arm only while it is `ENABLED`.
- **Joints** take the real RAVEN's values: at home the insertion is 0.40 m and the tool roll 0. The limits are those of the RAVEN grasper tool.
- **Frames.** CRTK poses are in the CRTK base frame `arm1_base` (origin at the remote center of motion, z up), `/ravenstate` poses in frame 0. The tool frame has its origin on the grasper axis and x along the jaws.
- **Servo commands.** A joint moves at most 0.0025 rad (insertion: 0.05 mm) per 1 ms cycle, so clients stream joint commands at up to 1 kHz. `servo_cr` and `servo_cv` are increments in the base frame. Each cycle applies one command type, the first of jr, cr, jp, cp, jv, cv. A Cartesian target past the joint limits is saturated at them.

Known bugs of the real software are not reproduced: `setpoint_js` and `setpoint_cp` are the setpoint (in the base frame), `grasp1/servo_jp` takes radians, and Cartesian control resumes after joint commands from where they sent the arm, not from where it lags behind. The simulation homes along a 2 s path instead of finding the joint limits, holds the arm where the real RAVEN turns its motors off, limits the setpoint to 2.5 rad/s (0.5 m/s) after IK jumps, and does not model the motors yet (`/ravenstate` torques and currents are 0).

## Native setup (Ubuntu 20.04 + ROS Noetic)

**1. AMBF 2.0** — build dependencies are listed in the `apt-get install` step of the [Dockerfile](./Dockerfile).

```bash
source /opt/ros/noetic/setup.bash
git clone -b ambf-2.0 https://github.com/WPI-AIM/ambf.git ~/ambf
mkdir -p ~/ambf/build && cd ~/ambf/build
cmake .. && make -j$(nproc)
```

**2. This repository.** The adapter is the C++ ROS package in `adapter/`; the keyboard client is Python, with its environment managed by [uv](https://docs.astral.sh/uv/):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
git clone https://github.com/lbyng/raven2-ambf-crtk.git ~/raven2-ambf-crtk
cd ~/raven2-ambf-crtk && uv sync
```

`uv sync` builds `.venv` on the system Python 3.8 that ROS Noetic uses. ROS packages (`rospy`, `tf`, ...) are not installed into it; they come from your sourced ROS workspace.

**3. Catkin workspace** with the [RAVEN-II package](https://github.com/uw-biorobotics/raven2) (for `raven_2` messages), [crtk_msgs](https://github.com/collaborative-robotics/crtk_msgs) 1.0.0 (the version of the real RAVEN's CRTK interface), AMBF's ROS modules and the adapter:

```bash
mkdir -p ~/ros_ambf_ws/src && cd ~/ros_ambf_ws/src
git clone https://github.com/uw-biorobotics/raven2.git
git clone -b 1.0.0 https://github.com/collaborative-robotics/crtk_msgs.git
ln -s ~/ambf/ambf_ros_modules ambf_ros_modules
ln -s ~/raven2-ambf-crtk/adapter raven2_ambf_crtk_adapter
cd ~/ros_ambf_ws && catkin_make
echo "source ~/ros_ambf_ws/devel/setup.bash" >> ~/.bashrc
```

**4. (Optional) alias** for launching AMBF with this repo's RAVEN-II model (base-link parallel to the ground):

```bash
echo "alias ambf-raven2='~/ambf/bin/lin-x86_64/ambf_simulator -a ~/raven2-ambf-crtk/raven_models/raven_straight.yaml'" >> ~/.bashrc
```

## Run

Each in its own terminal:

```bash
roscore
ambf-raven2
rosrun raven2_ambf_crtk_adapter adapter          # the CRTK adapter (1 kHz; e.g. _rate:=500 to change)
cd ~/raven2-ambf-crtk && uv run raven-keyboard   # optional keyboard teleop
```

Without the keyboard controller, enable the arm once it has homed:

```bash
rostopic pub -1 /arm1/state_command crtk_msgs/StringStamped "string: 'resume'"
```

The keyboard controller enables the arm itself and takes one key at a time. Joint keys move a joint by the step set at the top of `raven2_ambf_crtk/keyboard/keyboard_controller.py`; Cartesian keys move the tool 10 mm or turn it 0.01 rad about the base axes.

## Docker

Builds AMBF 2.0, the workspace and this repository, then runs `roscore`, the AMBF GUI and the adapter in one container.

Host requirements: [Docker Engine with Compose v2](https://docs.docker.com/engine/install/ubuntu/), the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), and an X server.

```bash
cd ~/raven2-ambf-crtk
xhost +local:root
docker compose up --build
```

Keyboard controller, from a second terminal in the repo:

```bash
docker compose exec raven2-ambf bash -ic "cd /ws/catkin_ws/src/raven2-ambf-crtk && uv run --frozen raven-keyboard"
```

AMBF logs to `/tmp/ambf.log` inside the container. The container uses host networking, so the topics are also visible from the host.

## Project layout

```
adapter/          CRTK <-> AMBF adapter, a C++ ROS (catkin) package
├── include/      RAVEN-II definitions (joint limits, DH, motors) and the kinematics API
└── src/          kinematics.cpp (forward/inverse kinematics), adapter.cpp (node: CRTK interface, states, servo commands, ravenstate)
raven2_ambf_crtk/
└── keyboard/     keyboard teleop client (Python)
raven_models/     AMBF description of the RAVEN-II (straight base) and its meshes
docker/           container entrypoint (Dockerfile and docker-compose.yml are at the root)
```

## Contact

Questions and discussions are welcome via GitHub Issues or by email to the maintainers.
