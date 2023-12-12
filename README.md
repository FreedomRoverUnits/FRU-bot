# ROS2 Package for Freedom Rover Unit Host Development
![SLAM fig](docs/linorobot2.gif)
![Telecommunication fig](docs/communication_fig.png)
## Description
Fork of [linorobot2](https://github.com/linorobot/linorobot2). This repository holds FRU's modifications that are proprietary to the FRU-bot. The FRU-bot consists of an [lds-01](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/) 2D lidar, [MPU 6050](https://www.sparkfun.com/products/10937), [TT motors](https://www.adafruit.com/product/3777), dual L298N motor-driver, IR speed [sensor](https://docs.sunfounder.com/projects/ultimate-sensor-kit/en/latest/components_basic/18-component_speed.html), and ESP32-WROOM-DevKitC. Power is delivered via a custom pcb board consisting of 7.4V, 5V, and 3.3V headers for fascilitated attachment of hardware interfaces. The robot chassis was sourced from an external party on [printables](https://www.printables.com/en/model/355730-two-wheeled-robot-chassis); although, the final chassis height was lowered for improved kinematics. Agent is intended to run on a remote docker container communicating wirelessly with esp32, but manual installation instructions are [available](ROBOT_INSTALLATION.md).

![FRU-bot build fig](docs/Build_fig_w_back.png)

## Installation
1. Recommended installation via installation script.
   * Creates ros2 workspace in $HOME/FRU_ws by default.
   * Assumes docker in sudo usr group to pull micro-ROS agent docker image.
   ```
   source /opt/ros/<ros_distro>/setup.bash
   cd /tmp
   wget https://raw.githubusercontent.com/FreedomRoverUnits/FRU-bot/humble/install_FRU_bot.bash
   bash install_FRU_bot.bash <FRU_bot_ws>
   source ~/.bashrc
   ```

## Quickstart
1. Connect to robot wifi.
2. Start micro-ros agent via docker in another terminal.
   ```
   source /opt/ros/<your_ros_distro>/setup.bash
   sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
   ```

3. Start EKF node and joint state publisher, set rviz flag to true to visualize robot description.
   ```
   ros2 launch linorobot2_FRU_bringup bringup_default.launch.py rviz:=true
   ```
4. Control the robot manually via teleop twist commands.
   ```
   python3 teleop_twist_keyboard_FRU.py
   ```

## Creating a map. 

1. Run SLAM toolbox.
   ```
   ros2 launch linorobot2_navigation slam.launch.py rviz:=true
   ``` 
2. Move the robot to build the map, and save it.
   ```
   cd linorobot2/linorobot2_navigation/maps
   ros2 run nav2_map_server map_saver_cli -f <map_name> --ros-args -p save_map_timeout:=10000.
   ```

## Autonomous Navigation.
1. First change MAP_NAME in navigation.launch.py to the name of the map you created. Or change it dynamically.
   ```
   cd <robot_ws>
   colcon build

   ros2 launch linorobot2_navigation navigation.launch.py map:=<path_to_map_file>/<map_name>.yaml
   ```
2. Run Nav2 package.
   ```
   ros2 launch linorobot2_navigation navigation.launch.py
   ```

Optional parameter for loading maps:

* map - Path to newly created map <map_name.yaml>.
  
Optional parameters for simulation on host machine:

* sim - Set to true for simulated robots on the host machine. Default value is false.
* rviz - Set to true to visualize the robot in RVIZ. Default value is false.

navigation.launch.py will continue to throw this error - 'Timed out waiting for transform from base_link to map to become available, tf error: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist' until the robot's pose has been initialized.
- Refer to [Nav2](https://navigation.ros.org/tutorials/docs/navigation2_on_real_turtlebot3.html#initialize-the-location-of-turtlebot-3) tutorial for more info.
## Troubleshooting Guide (Directly from [linorobot2](https://github.com/linorobot/linorobot2))
#### 1. The changes I made on a file are not taking effect on the package configuration/robot's behavior.
- You need to build your workspace every time you modify a file:

   ```
   cd <ros2_ws>
   colcon build
   #continue what you're doing...
   ```

#### 2. [`slam_toolbox]: Message Filter dropping message: frame 'laser'`
- Try to up `transform_timeout` by 0.1 in linorobot2_navigation/config/slam.yaml until the warning is gone.


#### 3. `target_frame - frame does not exist`
- Check your <robot_type>.properties.urdf.xacro and ensure that there's no syntax errors or repeated decimal points.

#### 4. Weird microROS agent behavior after updating the Linux/ROS
- Don't forget to update the microROS agent as well after your updates. Just run:
    
   ```
   bash update_microros.bash
   ```
