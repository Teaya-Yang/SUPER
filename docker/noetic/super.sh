#!/usr/bin/env bash
# Host-side launcher for the SUPER ROS1 Noetic Docker setup.
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo=${SUPER_REPO:-$(cd -- "$script_dir/../.." && pwd)}
workspace=${SUPER_WS:-$HOME/ws_super}
container=${SUPER_CONTAINER:-ws-super}
image=${SUPER_IMAGE:-super-noetic:local}
action=${1:-help}
if (($#)); then shift; fi

xauth_dir=$workspace/.x11
xauth_file=$xauth_dir/Xauthority

need_ws() { [[ -d "$workspace/src" ]] || { echo 'Run init first.' >&2; exit 1; }; }

# Export only this display's cookie into a bind-mounted directory. FamilyWild
# allows the container's hostname to differ from the host. Never use xhost +.
refresh_xauth() {
    [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]] || {
        echo 'Run this command from a host desktop terminal with DISPLAY set (X11 or XWayland).' >&2
        exit 1
    }
    command -v xauth >/dev/null || { echo 'Install xauth on the host first.' >&2; exit 1; }
    local host_authority=${XAUTHORITY:-$HOME/.Xauthority}
    [[ -r "$host_authority" ]] || { echo "Cannot read X11 authority file: $host_authority" >&2; exit 1; }
    mkdir -p "$xauth_dir"
    chmod 700 "$xauth_dir"
    local tmp
    tmp=$(mktemp "$xauth_dir/.Xauthority.XXXXXX")
    xauth -f "$host_authority" nlist "$DISPLAY" \
        | sed 's/^..../ffff/' \
        | xauth -f "$tmp" nmerge -
    [[ -s "$tmp" ]] || { rm -f "$tmp"; echo "No X11 cookie found for display $DISPLAY." >&2; exit 1; }
    chmod 600 "$tmp"
    mv -f "$tmp" "$xauth_file"
}

detect_renderer() {
    local renderer=${SUPER_RENDERER:-auto}
    if [[ "$renderer" == auto ]]; then
        local runtimes
        runtimes=$(docker info --format '{{range $name, $_ := .Runtimes}}{{$name}} {{end}}')
        if [[ " $runtimes " == *' nvidia '* ]]; then renderer=nvidia; else renderer=software; fi
    fi
    echo "$renderer"
}

case "$action" in
    init)
        mkdir -p "$workspace/src" "$workspace/.ros" "$xauth_dir"
        chmod 700 "$xauth_dir"
        ln -sfn "$repo" "$workspace/src/SUPER"
        # fsm_node saves replan logs here on shutdown; the dir is not in git.
        mkdir -p "$repo/super_planner/log/replan_logs"
        echo "Workspace: $workspace (src/SUPER -> $repo)"
        ;;
    image)
        docker build --build-arg "LOCAL_UID=$(id -u)" --build-arg "LOCAL_GID=$(id -g)" \
            -t "$image" "$script_dir"
        ;;
    up)
        need_ws
        refresh_xauth
        renderer=$(detect_renderer)
        graphics_args=()
        case "$renderer" in
            nvidia) graphics_args=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility) ;;
            software) graphics_args=(-e LIBGL_ALWAYS_SOFTWARE=1) ;;
            *) echo 'SUPER_RENDERER must be auto, nvidia, or software.' >&2; exit 1 ;;
        esac
        echo "Creating container $container ($renderer rendering, display $DISPLAY)."
        docker run -d --name "$container" --network host --init --shm-size=512m \
            --mount "type=bind,src=$workspace,dst=$workspace" \
            --mount "type=bind,src=$repo,dst=$repo" \
            --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
            -e "SUPER_WS=$workspace" -e "ROS_HOME=$workspace/.ros" \
            -e "ROS_MASTER_URI=${SUPER_ROS_MASTER_URI:-http://127.0.0.1:11311}" \
            -e "ROS_IP=${SUPER_ROS_IP:-127.0.0.1}" \
            -e "DISPLAY=$DISPLAY" -e "XAUTHORITY=$xauth_file" -e QT_X11_NO_MITSHM=1 \
            "${graphics_args[@]}" \
            -w "$workspace" "$image" sleep infinity
        ;;
    build)
        # Eigen-heavy translation units need >1 GB each; keep parallelism modest.
        docker exec -e "SUPER_JOBS=${SUPER_JOBS:-4}" "$container" /usr/local/bin/super-entrypoint bash -c \
            'catkin_make -DBUILD_TYPE=Release -j"$SUPER_JOBS" "$@"' super-build "$@"
        ;;
    shell)
        refresh_xauth
        docker exec -it "$container" /usr/local/bin/super-entrypoint bash
        ;;
    demo)
        case "${1:-}" in
            high_speed) launch=benchmark_high_speed.launch ;;
            dense) launch=benchmark_dense.launch ;;
            click) launch=click_demo.launch ;;
            *) echo "Usage: bash $0 demo {high_speed|dense|click}" >&2; exit 1 ;;
        esac
        shift
        refresh_xauth
        docker exec -it "$container" /usr/local/bin/super-entrypoint \
            roslaunch mission_planner "$launch" "$@"
        ;;
    lab)
        # fsm_node only, configured for the lab stack (FAST_LIO topics, camera_init frame).
        # Run the Unity endpoint, simulator, FAST_LIO and quad_super_waypoint_control in ws-palio.
        [[ -n "${DISPLAY:-}" ]] && refresh_xauth || true
        docker exec -it "$container" /usr/local/bin/super-entrypoint \
            roslaunch super_planner lab_fsm.launch "$@"
        ;;
    exec)
        docker exec "$container" /usr/local/bin/super-entrypoint "$@"
        ;;
    start)
        docker start "$container"
        ;;
    stop)
        docker stop "$container"
        ;;
    rm)
        docker rm -f "$container"
        ;;
    *)
        echo "Usage: bash $0 {init|image|up|build [CATKIN_ARGS...]|shell|demo {high_speed|dense|click}|lab [ROSLAUNCH_ARGS...]|exec COMMAND...|start|stop|rm}"
        ;;
esac
