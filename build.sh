#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
sudo apt install -y ninja-build
rosdep install -r --from-paths . --ignore-src -y
colcon build --symlink-install --mixin release compile-commands ninja
