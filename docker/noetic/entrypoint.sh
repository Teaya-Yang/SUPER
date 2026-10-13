#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
if [[ -n "${SUPER_WS:-}" && -f "$SUPER_WS/devel/setup.bash" ]]; then
    source "$SUPER_WS/devel/setup.bash"
fi
exec "$@"
