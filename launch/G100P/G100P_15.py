#!/usr/bin/env python3

import launch
import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('dm_preview'), 'rviz', 'apc.rviz')
    base_frame = 'dm_base_frame'
    camera_model_1 = ''
    camera_model_2 = ''
    dev_1_serial_number = ""
    dev_2_serial_number = ""

    ####################################################################
    # multi_module
    # use
    # True : if set to ture, please have to enter the serial number of module. 
    # False: no need to set additional parameters.
    multi_module = False

    if multi_module == True :
        camera_model_1 = 'dm8036'
        dev_1_serial_number = "8036D9AF800115"
        camera_model_2 = 'dm8062'
        dev_2_serial_number = "80625APA00022"
    else :
        camera_model_1 = 'dm_preview'
    ####################################################################
    # eYs3D Configurations to be loaded by APC Node

    qos_config_dir = os.path.join(get_package_share_directory('dm_preview'), 'config', 'common.yaml')

    ld = LaunchDescription()

    DM_device_node = Node(
        package="dm_preview",
        executable="apc_camera_node",
        name= camera_model_1,
        output="screen",
        namespace= "apc",
        emulate_tty=True,
        parameters=[{
            "multi_module": multi_module,

            # SDK will not output video stream if set "False"
            "enable_color_stream": True,
            "enable_depth_stream": False,
            "enable_point_cloud_stream": False,
            "enable_imu_stream": False,

            # auto_config_camera_mode
            # use:
            # -set camera mode (refer to PIF)
            # -set camera config by manual if set 0
            "auto_config_camera_mode": 0,

            # Manual setting START [
            "framerate": 60, #also 30,15,10
            "color_width": 2560,
            "color_height": 720,

            # color_stream_format
            # use:
            # -stream_yuyv = 0
            # -stream_mjpg = 2
            "color_stream_format": 0,

            "depth_width": 0,
            "depth_height": 0,

            # depth_data_type
            # for more detail setting values, pls refer to eSPDI_def.h
            # use:
            # -type_color_only = 0
            # -type_8_bits = 1
            # -type_14_bits = 2
            # -type_11_bits = 4
            # -type_rectify_color_only = 5
            # -type_8_bits_raw = 6
            # -type_14_bits_raw = 7
            # -type_11_bits_raw = 9
            # -type_8_bits_interleave_mode = 17
            # -type_14_bits_interleave_mode = 18
            # -type_11_bits_interleave_mode = 20
            # -type_8_bits_scale_down = 33
            # -type_14_bits_scale_down = 34
            # -type_11_bits_scale_down = 36
            # -type_14_bits_scale_down_interleave_mode = 50
            # -type_11_bits_scale_down_interleave_mode = 52
            "depth_data_type": 5,

            # interleave mode
            "interleave_mode": False,

            # depth_output_type
            # use:
            # -depth_output_raw = 0
            # -depth_output_gray = 1
            # -depth_output_colorful = 2
            "depth_output_type": 1,
            #] Manual setting END

            "depth_maximum_mm": 1000,
            "state_ae": True,
            "state_awb": True,
            "state_extendIR": False,
            "exposure_time_step": -5,
            "white_balance_temperature": 3000,
            "ir_intensity": 6,
            "dev_serial_number": dev_1_serial_number,
            "kernel_name": "",

            "left_color_frame": "left_color_frame",
            "right_color_frame": "right_color_frame",
            "depth_frame": "depth_frame",
            "points_frame": "points_frame",
            "imu_frame": "imu_frame",
            "imu_frame_processed": "imu_frame_processed",

            "left_color_topic": "left/image_color",
            "right_color_topic": "right/image_color",
            "depth_topic": "depth/image_raw",
            "points_topic": "points/data_raw",
            "imu_topic": "imu/data_raw",
            "imu_processed_topic": "imu/data_raw_processed"},
            qos_config_dir
        ]
    )

    DM_device_1_node = Node(
        package="dm_preview",
        executable="apc_camera_node",
        name=camera_model_2,
        output="screen",
        namespace="apc",
        emulate_tty=True,
        parameters=[{
            "multi_module": multi_module,

            # SDK will not output video stream if set "False"
            "enable_color_stream": True,
            "enable_depth_stream": True,
            "enable_point_cloud_stream": False,
            "enable_imu_stream": False,

            # auto_config_camera_mode
            # use:
            # -set camera mode (refer to PIF)
            # -set camera config by manual if set 0
            "auto_config_camera_mode": 2,
            
            # Manual setting START [
            "framerate": 60,
            "color_width": 1280,
            "color_height": 720,

            # color_stream_format
            # use:
            # -stream_yuyv = 0
            # -stream_mjpg = 2
            "color_stream_format": 0,

            "depth_width": 1280,
            "depth_height": 720,

            # depth_data_type
            # for more detail setting values, pls refer to eSPDI_def.h
            # use:
            # -type_color_only = 0
            # -type_8_bits = 1
            # -type_14_bits = 2
            # -type_11_bits = 4
            # -type_8_bits_raw = 6
            # -type_14_bits_raw = 7
            # -type_11_bits_raw = 9
            # -type_8_bits_interleave_mode = 17
            # -type_14_bits_interleave_mode = 18
            # -type_11_bits_interleave_mode = 20
            # -type_8_bits_scale_down = 33
            # -type_14_bits_scale_down = 34
            # -type_11_bits_scale_down = 36
            "depth_data_type": 4,

            # interleave mode
            "interleave_mode": False,

            # depth_output_type
            # use:
            # -depth_output_raw = 0
            # -depth_output_gray = 1
            # -depth_output_colorful = 2
            "depth_output_type": 2,
            #] Manual setting END
            
            "depth_maximum_mm": 1000,
            "state_ae": True,
            "state_awb": True,
            "state_extendIR": False,
            "exposure_time_step": -5,
            "white_balance_temperature": 3000,
            "ir_intensity": 3,
            #80625APA00022
            "dev_serial_number": dev_2_serial_number,
            "kernel_name": "",

            "left_color_frame": "left_color_frame",
            "right_color_frame": "right_color_frame",
            "depth_frame": "depth_frame",
            "points_frame": "points_frame",
            "imu_frame": "imu_frame",
            "imu_frame_processed": "imu_frame_processed",

            "left_color_topic": camera_model_2+"/left/image_color",
            "right_color_topic": camera_model_2+"/right/image_color",
            "depth_topic": camera_model_2+"/depth/image_raw",
            "points_topic": camera_model_2+"/points/data_raw",
            "imu_topic": camera_model_2+"/imu/data_raw",
            "imu_processed_topic": camera_model_2+"/imu/data_raw_processed"},
            qos_config_dir
        ]
    )
    COLOR_TF_node = Node(
        package='tf2_ros', executable='static_transform_publisher', name="camera_left_color_broadcaster", output='screen',
            arguments=["0", "1", "0", "0", "0", "0", "dm_base_frame", "left_color_frame"]
    )
    DEPTH_TF_node = Node(
        package='tf2_ros', executable='static_transform_publisher', name="camera_depth_broadcaster", output='screen',
            arguments=["0", "1", "0", "0", "0", "0", "dm_base_frame", "depth_frame"]
    )
    PC_TF_node = Node(
        package='tf2_ros', executable='static_transform_publisher', name="camera_point_cloud_broadcaster", output='screen',
            arguments=["0", "0", "0", "-1.5707963267948966", "0", "-1.5707963267948966", "dm_base_frame", "points_frame"]
    )
    IMU_TF_node = Node(
        package='tf2_ros', executable='static_transform_publisher', name="camera_imu_broadcaster", output='screen',
            arguments=["0", "0", "0", "0", "0", "0", "dm_base_frame", "imu_frame"]
    )
    RVIZ_node = Node(
        package='rviz2', executable='rviz2', name="rviz2", output='screen', arguments=['-d', rviz_config_dir]
    )

    if multi_module == False :
        ld.add_action(DM_device_node)
    else :
        ld.add_action(DM_device_node)
        ld.add_action(DM_device_1_node)

    ld.add_action(COLOR_TF_node)
    ld.add_action(DEPTH_TF_node)
    ld.add_action(PC_TF_node)
    ld.add_action(IMU_TF_node)
    ld.add_action(RVIZ_node)

    return ld
