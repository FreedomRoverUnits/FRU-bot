#!/usr/bin/env bash
# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -e

ROSDISTRO="$(printenv ROS_DISTRO)"
WORKSPACE="$HOME/FRU_ws"

if [[ "$1" != "" ]]
    then
        WORKSPACE=$1
fi

echo "Installing in workspace ${WORKSPACE}"
echo

if [[ "$ROSDISTRO" == "" || "$ROSDISTRO" == "<unknown>" ]]
    then
        echo "No ROS2 distro detected"
        echo "Try running $ source /opt/ros/<ros_distro>/setup.bash and try again."
        exit 1
fi

echo
echo "INSTALLING NOW...."
echo

#### 1.1 Source your ROS2 distro and workspace
cd $HOME
mkdir -p $WORKSPACE/src
source /opt/ros/$ROS_DISTRO/setup.bash
cd $WORKSPACE
colcon build
source $WORKSPACE/install/setup.bash

#### 1.2 Download FRU-bot repository:
cd $WORKSPACE
git clone -b $ROS_DISTRO https://github.com/FreedomRoverUnits/FRU-bot src/FRU_bot

#### 1.3 Install FRU-bot package:
cd $WORKSPACE
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
colcon build
source $WORKSPACE/install/setup.bash

#### 1.4 Download docker image for micro-ros agent.
docker pull microros/micro-ros-agent:humble

echo
echo "INSTALLATION DONE."
echo
echo "Restart your robot computer now."
