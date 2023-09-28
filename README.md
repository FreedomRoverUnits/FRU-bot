# ROS2 Package for Freedom Rover Unit Host Development
![linorobot2](docs/linorobot2.gif)

## Description
Fork of [linorobot2](https://github.com/linorobot/linorobot2), a ROS2 package that facilitates development of 2/4 wheel-differential-drive and mecanum wheel drive robots built from accessible parts. This repository holds FRU's modifications that are proprietary to FRU's robot: [lds-01](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver) 2D lidar, MPU 6050, TT motors, dual motor driver interface, custom 3D printed [chassis](https://www.printables.com/en/model/355730-two-wheeled-robot-chassis/files), and ESP32-DevKitC-32UE. Intended to run on a remote docker container communicating wirelessly with esp32.   

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
