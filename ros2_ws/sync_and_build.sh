#!/usr/bin/env bash
# Syncs this repo's ros2_ws/src (the git-tracked source of truth) into a
# clean WSL2-native path and builds it there. Building must happen outside
# this repo because CMake/colcon cannot handle the literal parentheses in
# this folder's name ("(8) Simulated Robotic Arm...") -- a hard, unrelated-
# to-performance limitation in how Make/CMake generate and parse shell
# commands, not something fixable by choosing a faster filesystem.
#
# Run from a WSL2 Ubuntu terminal with ROS2 + the project venv sourced:
#   source /opt/ros/jazzy/setup.bash
#   source ~/.venvs/ros2_arm/bin/activate
#   bash ros2_ws/sync_and_build.sh

set -e

REPO_ROS2_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_WS="$HOME/ros2_ws"

mkdir -p "$BUILD_WS"
rsync -a --delete --exclude=build --exclude=install --exclude=log "$REPO_ROS2_WS/src/" "$BUILD_WS/src/"

cd "$BUILD_WS"
# colcon itself has a system-Python shebang; running it via the active
# venv's python3 is what makes the generated node executables' shebangs
# point at the venv (otherwise `ros2 run`/`ros2 launch` fail with
# "ModuleNotFoundError: No module named 'pybullet'").
python3 "$(which colcon)" build

echo ""
echo "Built at $BUILD_WS. Before running nodes, also source:"
echo "  source $BUILD_WS/install/setup.bash"
