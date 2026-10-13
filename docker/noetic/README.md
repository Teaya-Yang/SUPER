# SUPER in Docker (ROS1 Noetic on any Ubuntu host)

SUPER is tested on Ubuntu 20.04 + ROS1 Noetic. This directory builds that
environment in a Docker container so it runs on newer hosts (e.g. 22.04).

## Prerequisites (host)

- Docker Engine with the non-root (`docker` group) setup done.
- A graphical desktop session (X11 or XWayland) and `xauth`:
  `sudo apt install -y xauth`.
- Optional: `nvidia-container-toolkit` for GPU-accelerated RViz and the
  `marsim_render` LiDAR simulator. Without it, OpenGL uses Mesa software
  rendering (works, but slower).

## Setup

Run from the repository root. The workspace is created at `~/ws_super` with
`src/SUPER` symlinked to this repository, so edits here are built directly.

```bash
bash docker/noetic/super.sh init     # create ~/ws_super and the symlink
bash docker/noetic/super.sh image    # build the super-noetic:local image
bash docker/noetic/super.sh up       # create the persistent ws-super container
bash docker/noetic/super.sh build    # catkin_make inside the container
```

`up` must be run from a desktop terminal; it captures the X11 display and
decides between NVIDIA and software rendering (`SUPER_RENDERER=nvidia|software`
overrides). If you later install the NVIDIA toolkit, run
`super.sh rm && super.sh up` to recreate the container.

Do **not** run `scripts/select_ros_version.sh`; the committed package files are
already the ROS1 variants.

## Run the demos

From a desktop terminal:

```bash
bash docker/noetic/super.sh demo click        # press G in RViz, click a goal
bash docker/noetic/super.sh demo high_speed
bash docker/noetic/super.sh demo dense
```

Other commands:

```bash
bash docker/noetic/super.sh shell             # interactive shell in the container
bash docker/noetic/super.sh build             # rebuild after C++ changes
bash docker/noetic/super.sh exec rostopic list
bash docker/noetic/super.sh stop | start | rm
```

YAML/launch changes take effect on the next launch without rebuilding.
