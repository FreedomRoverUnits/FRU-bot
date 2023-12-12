## Manual installation of FRU-bot package.

### 1. Install micro-ROS and its dependencies if not using a docker file to run micro-ROS agent.

#### 1.1 Source your ROS2 distro and workspace
If it's your first time using ROS2 and haven't created your ROS2 workspace yet, you can check out [ROS2 Creating a Workspace](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html) tutorial.

    source /opt/ros/<your_ros_distro>/setup.bash
    cd <your_ws>
    colcon build
    source install/setup.bash

    
#### 1.2 Download and install micro-ROS:

    cd <your_ws>
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
    sudo apt install python3-vcstool build-essential
    sudo apt update && rosdep update
    rosdep install --from-path src --ignore-src -y
    colcon build
    source install/setup.bash

#### 1.3 Setup micro-ROS agent:

    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/setup.bash

* You can ignore `1 package had stderr output: microxrcedds_agent` after building your workspace. 

### 2. Download FRU-bot and its dependencies:

#### 2.1 Download linorobot2:

    source /opt/ros/<your_ros_distro>/setup.bash
    cd <your_ws>
    git clone https://github.com/FreedomRoverUnits/FRU-bot src/FRU_bot

#### 2.2 Install linorobot2 package:
    
    cd <your_ws>
    rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
    colcon build
    source install/setup.bash

* microxrcedds_agent dependency checks are skipped to prevent this [issue](https://github.com/micro-ROS/micro_ros_setup/issues/138) of finding its keys. This means that you have to always add `--skip-keys microxrcedds_agent` whenever you have to run `rosdep install` on the ROS2 workspace where you installed linorobot2.

### MISC Instructions

#### Pull micro-ROS Agent Docker Image:
    docker pull microros/micro-ros-agent:humble