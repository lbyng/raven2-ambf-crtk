# Docker Setup (AMBF + Raven2 + CRTK Adapter)

This setup builds AMBF from source, builds the Raven2 workspace, and runs the
AMBF GUI plus the adapter in a single container.

## Prereqs on host (Ubuntu 20.04)

- Docker Engine and Docker Compose v2
- NVIDIA Container Toolkit (for GPU + OpenGL)
- X11 running on host

Allow the container to use your X server:

```bash
xhost +local:root
```

## Build and run

From the repo root:

```bash
docker compose build
docker compose up
```

## Notes

- AMBF logs: `/tmp/ambf.log` inside the container.
- ROS master log: `/tmp/roscore.log` inside the container.
- If you see display errors, confirm `DISPLAY` is set on the host and
  `/tmp/.X11-unix` is mounted.
