# YDLIDAR OS30A Depth Camera ROS2 Foxy Package
# **eYs3D ROS2 SDK**

The ROS SDK allows you to use eYs3D Depth Cameras with ROS2.

----------

## Support platforms

* Support x86_64 (64-bit)
* Tested on X86 PC with Ubuntu 20.04  
* Support ARM aarch64  (64-bit)
* Tested on ARM64 NVIDIA® Jetson Nano™ with Ubuntu 20.04
* These are the currently supported ROS2 Distributions:   
  - Foxy Fitzroy (Ubuntu 20.04 Focal)

----------

## Documentations

API reference and the guide documentations.

----------

## Installation of the package for ROS2 HUMBLE Instructions

To build the packages, follow these steps:

```shell
# Create a symlink for libdc1394.so, if you dont have this file, install opencv
sudo ln -sf /lib/x86_64-linux-gnu/libdc1394.so /usr/lib/libdc1394.so.22

# Change headers calls in tf2_geometry_msgs cpp file
sudo nano /opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp
# and change ".hpp" to ".h" in:
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"

# Go to your workspace/src folder
cd ~/ros2_ws/src

# Clone the repository
TODO: git clone [add repo link]

# Return to workspace folder
cd ~/ros2_ws

# Compile the package
colcon build --packages-select ydlidar_os30a

# Source your workspace
source ~/ros2_ws/install/setup.bash

```

## Usage Instructions
Start the eYs3D depth camera node, this will stream all camera sensors and publish on the appropriate ROS topics.  
From the root of your workspace (dev_ws), run the following command:  
- **using the launch file**  
   ```
   . install/setup.bash
   ros2 launch dm_preview apc_camera_launch.py
   ``` 
   ![ros2_preview_rviz2](https://user-images.githubusercontent.com/88474678/134872905-16ae6186-12c5-469f-8aa9-1f8f36122bc0.png)

- **without using the launch file**  
   ```
   . install/setup.bash
   ros2 run dm_preview apc_camera_node
   ```   
   In another terminal source the setup file and then run a C++ listener:
   ```  
   . install/setup.bash
   ros2 run dm_preview apc_camera_listener_node
   ``` 
   ![listener_node](https://user-images.githubusercontent.com/88474678/136147902-33a8b8b1-0c5d-4997-aaa4-ddf5c8e04977.png)
  
----------

## Launch file config instructions  
Set up your launch file by the following file.  
```
/dev_ws/src/dm_preview/launch/apc_camera_launch.py  
``` 
Parameters:
* multi_module
  - set [True]  
  if more then one module plug-in to host and please fill in the serial number of each module into the paramter dev_[mobule number]_serial_number.  
  for example: dev_1_serial_number = "803600000001" and dev_2_serial_number = "806200000002"  
  - set [False]  
  if only one module plug-in to host.  
* enable_color_stream
  - set [True]  
  SDK will output color stream.  
  - set [False]  
  SDK will not output color stream.  
* enable_depth_stream
  - set [True]  
  SDK will output depth stream.  
  - set [False]  
  SDK will not depth color stream.  
* enable_point_cloud_stream
  - set [True]  
  SDK will output point cloud stream.  
  - set [False]  
  SDK will not output point cloud stream.  
* enable_imu_stream
  - set [True]  
  SDK will output IMU stream.  
  - set [False]  
  SDK will not output IMU stream. 
* auto_config_camera_mode
  - set [0]  
  If set 0 you will manual set below parameters { framerate, color_width, color_height, color_stream_format, depth_width, depth_height, depth_data_type, depth_output_type, interleave_mode}.
  - set [ >0 integer number]  
  Please set the video mode according to the PIF of each module.   
* framerate
  - set [5 ~ 60]  
  Please set the frame rate according to the PIF of each module.  
* color_width
  - set [1280 | 2560]  
  Please set the color width according to the PIF of each module. 
* color_height
  - set [720 | 960]  
  Please set the color height according to the PIF of each module.  
* color_stream_format  
  - set [0]   
  Output color YUYV format.   
  - set [2]  
  Output color MJPG format.   
* depth_width
  - set [640 | 1280]  
  Please set the depth width according to the PIF of each module.  
* depth_height
  - set [360 | 720]  
  Please set the depth height according to the PIF of each module.  
* depth_data_type
  - set [integer number]  
  Please refer to /dev_ws/src/dm_preview/eYs3D_wrapper/include/[eSPDI_def.h] file.  
* depth_output_type  
  - set [0]  
  Output raw color stream for depth.  
  - set [1]  
  Output gray color stream for depth.  
  - set [2]  
  Output colorful stream for depth.  
* interleave_mode  
  - set [True]  
  Enable interleave mode if module and video mode support.  
  - set [False]  
  Disable interleave mode.  
* depth_maximun_mm  
  - set [integer number]  
  Depth range (mm).  
* state_ae  
  - set [True]   
  Enable auto exposure.   
  - set [False]  
  Disable auto exposure.  
* state_awb  
  - set [True]  
  Enable auto white balance.  
  - set [False]  
  Disable auto white balance.  
* state_extendIR  
  - set [True]  
  "ir_intensity" range is [0 ~ 15].
  - set [False]  
  "ir_intensity" range is [0 ~ 6].
* exposure_time_step  
  - set [-13 ~ 3]  
  Set exposure time. ("state_ae" must be set [false]).
* white_balance_temperature
  - set [2800 ~ 6500]
  Set white balance temperature. ("state_awb" must be set [false]).
* ir_intensity  
  - set [0 ~ 6]  
  set IR intensity.  
* left_color_topic  
  - set [string]  
  Change topic name.  
* depth_topic  
  - set [string]  
  Change topic name.  
* points_topic  
  - set [string]  
  Change topic name.  
* imu_topic  
  - set [string]  
  Change topic name.  
----------  

## Dynamic Change Params  
To change a parameter’s value at runtime.  
* rqt  
   From the root of your workspace (dev_ws), run the following command:   
   ```
   rqt
   ```
   Select Plugins/Configuration/Dynamic Reconfigure  
   ![rqt-1](https://user-images.githubusercontent.com/88474678/151469761-b0f5f5cc-9342-47c6-8113-47ee5251cb5e.png)  
   ![rqt-2](https://user-images.githubusercontent.com/88474678/151469832-1f85cfcf-7124-4ee3-ad9d-07af4cf46ed6.png)  

* console  
   From the root of your workspace (dev_ws), run the following command:  
   `ros2 param set <node_name> <parameter_name> <value>`
     
   - Change video mode [mode (refer to PIF)]:   
   ```
   ros2 param set /apc/dm_preview auto_config_camera_mode 2
   ```  
   - Adjust IR level [0-6]:   
   ```
   ros2 param set /apc/dm_preview ir_intensity 2
   ``` 
   - Trun On/Off Auto Exposure [true/false]:  
   ```
   ros2 param set /apc/dm_preview state_ae true
   ``` 
   - Trun On/Off Auto White Balance [true/false]:  
   ```
   ros2 param set /apc/dm_preview state_awb true
   ```   
   - Trun On/Off Extand IR range [true/false]:
   ```
   ros2 param set /apc/dm_preview state_extendIR true
   ```
   - Adjust Exposure Time steps [-13-3]:
   ```
   ros2 param set /apc/dm_preview exposure_time_step 2
   ```
   - Adjust White Balance Temperature [2800-6500]:
   ```
   ros2 param set /apc/dm_preview white_balance_temperature 3500
   ```
  ![sett](https://user-images.githubusercontent.com/88474678/164411597-00c03d56-e24d-4b81-afc6-f9be674ca5a8.png)

----------  
## Quality of Service (QoS) settings 
  The base QoS profile currently includes settings for the following policies:  
  Users can change the default value by themselves in the following files :  
  ```
  /dev_ws/src/dm_preview/config/common.yaml
  ```
  - History

    - Keep last: only store up to N samples, configurable via the queue depth option.
    - Keep all: store all samples, subject to the configured resource limits of the underlying middleware.

  - Depth

    - Queue size: only honored if the “history” policy was set to “keep last”.

  - Reliability

    - Best effort: attempt to deliver samples, but may lose them if the network is not robust.
    - Reliable: guarantee that samples are delivered, may retry multiple times.

  - Durability

    - Transient local: the publisher becomes responsible for persisting samples for “late-joining” subscriptions.
    - Volatile: no attempt is made to persist samples.

----------  
## Launch two ROS2 Application  
   If there are two modules and each module needs to open a ROS2 node, please set it as follows:
   * Set serial number of 1st camera module in the launch file for the master ROS2, for example set  
     "dev_serial_number": "8036D9AF800115",  
     ![image](https://user-images.githubusercontent.com/88474678/160577757-34cc3bd9-9b44-4a71-abb8-b9d222583ae7.png) 
   * Set serial number of 2ed camera module and modify topic name in the launch file for the slave ROS2, for example set   
     "dev_serial_number": "80625APA00022",  
     "left_color_topic": "dm8062/left/image_color",  
     "right_color_topic": "dm8062/right/image_color",  
     "depth_topic": "dm8062/depth/image_raw",  
     "points_topic": "dm8062/points/data_raw",  
     "imu_topic": "dm8062/imu/data_raw",  
     "imu_processed_topic": "dm8062/imu/data_raw_processed"  
     ![image](https://user-images.githubusercontent.com/88474678/160577461-b1834ece-8597-4bde-93f6-e3298897f6e4.png)  
     ![image](https://user-images.githubusercontent.com/88474678/160577578-901ece9b-7802-45a3-b1e8-53688e4053f5.png)  
   * After launch rviz, please select correct topic for slave module  
   ![2022-03-29 16-34-25 的螢幕擷圖](https://user-images.githubusercontent.com/88474678/160578602-d4f752c4-84d3-4a08-bdde-1ffc9bbfa8e8.png)  

----------   
## Published Topics  
   The published topics differ according to the device and parameters. After running the above command,  
   the following list of topics will be available (This is a partial list. For full one type ros2 topic list):  
      
    /apc/depth/camera_info  
    /apc/depth/image_raw  
    /apc/left/camera_info  
    /apc/left/image_color  
    /apc/points/data_raw  

----------  
## ROS2 command list  
  Command-line tools for ROS2 system  
  - `ros2 node list`  will show you the names of all running nodes.  
 
  - `ros2 node info <node_name>` access more information about node.  

  - `ros2 topic list` will return a list of all the topics currently active in the system.  

  - `ros2 topic info <topic name>` will return the information of the topic.  

  - `ros2 topic echo <topic name>` to see the data being published on a topic.  

  - `ros2 param list` to see the parameters belonging to your nodes.  

  - `ros2 param get <node_name> <parameter_name>` to display the type and current value of a parameter.   

  - `ros2 param set <node_name> <parameter_name> <value>` to change a parameter’s value at runtime.   

  - `ros2 param dump <node_name>` You can “dump” all of a node’s current parameter values into a file to save them for later.  

  - `ros2 param load <node_name> <parameter_file>` you can load parameters from a file to a currently running node.  

  - `ros2 topic hz <topic name>` display the publishing rate of a topic. The rate reported is by default the average rate over the entire time rostopic has been running. 

  - `ros2 daemon status` output the status of the daemon.  

  - `ros2 daemon stop`  stop the daemon if it is running.  
 
  - `ros2 daemon start`  start the daemon if it isn't running.


---------- 
## Troubleshoot  
 - Cannot find IMU for 8062 module:  
   From the root of your workspace (dev_ws), you can use the below command to gain permission.
   ```
   ./src/dm_preview/scripts/permissions.sh
   ``` 
----------
## License

This project is licensed under the [Apache License, Version 2.0](/LICENSE). Copyright 2020 eYs3D Microelectronics, Co., Ltd.
