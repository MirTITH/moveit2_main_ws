#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
rosdep install -r --from-paths . --ignore-src -y
colcon build --symlink-install --mixin release compile-commands
