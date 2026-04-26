#!/bin/bash

set -e

# Check if ninja and ccache are installed. if not, install them.
pkg_to_install=""

if ! command -v ninja &> /dev/null; then
    pkg_to_install+=" ninja-build"
fi

if ! command -v ccache &> /dev/null; then
    pkg_to_install+=" ccache"
fi

if [ -n "$pkg_to_install" ]; then
    read -p "The following packages are required but not installed:$pkg_to_install. Do you want to install them? [Y/n] " answer
    if [[ "$answer" != "Y" && "$answer" != "y" && "$answer" != "" ]]; then
        echo "Aborting."
        exit 1
    fi
    sudo apt-get update
    sudo apt-get install -y $pkg_to_install
fi


source /opt/ros/humble/setup.bash
rosdep install -r --from-paths . --ignore-src -y

colcon build --symlink-install --mixin release compile-commands build-testing-off ninja ccache
