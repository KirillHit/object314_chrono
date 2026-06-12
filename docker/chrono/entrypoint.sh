#!/usr/bin/env bash
set -e

if [ -f "${ROS_INSTALL_PREFIX}/setup.bash" ]; then
    . "${ROS_INSTALL_PREFIX}/setup.bash"
fi

if [ -n "${ROS_WORKSPACE_DIR}" ] && [ -f "${ROS_WORKSPACE_DIR}/install/setup.sh" ]; then
    . "${ROS_WORKSPACE_DIR}/install/setup.sh"
fi

exec "$@"
