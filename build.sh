#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
colcon build --symlink-install --mixin release compile-commands
