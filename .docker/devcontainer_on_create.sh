#!/usr/bin/env bash
# .docker/devcontainer_on_create.sh
#
# Runs ONCE when the devcontainer is first created.
# Works for both the Humble and Jazzy variants — the active distro
# is read from $ROS_DISTRO which is set by the base image.

set -euo pipefail

DISTRO="${ROS_DISTRO:-humble}"

echo "═══════════════════════════════════════════════════════"
echo "  Rolling Bonxai devcontainer — first-time setup"
echo "  ROS distro: ${DISTRO}"
echo "═══════════════════════════════════════════════════════"

# ROS setup scripts reference unset variables (e.g. AMENT_TRACE_SETUP_FILES)
set +u
source "/opt/ros/${DISTRO}/setup.bash"
set -u

cd /ros2_ws

rosdep update && \
rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro "${DISTRO}" \
    -y \
    --skip-keys "libpcl-all-dev" \
    || true

colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --event-handlers console_cohesion+

# Symlink compile_commands.json to workspace root for clangd / VS Code
ln -sf /ros2_ws/build/compile_commands.json \
        /ros2_ws/compile_commands.json 2>/dev/null || true

echo "═══════════════════════════════════════════════════════"
echo "  Build complete!  Distro: ${DISTRO}"
echo "═══════════════════════════════════════════════════════"
