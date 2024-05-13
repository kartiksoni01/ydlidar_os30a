# YDLIDAR OS30A Depth Camera ROS Package

[Link to datasheet](https://www.ydlidar.com/products/view/23.html)

This ROS package is designed for interfacing with the Ydlidar OS-30A Depth Camera, providing configuration options and functionality for seamless integration into ROS (Robot Operating System) environments.

## Launching the Ydlidar OS-30A Depth Camera

To launch the Ydlidar OS-30A Depth Camera ROS package, execute the following command:

```bash
roslaunch bimu_rgbd_camera bimu_camera.launch
```

## Configuration Parameters Explanation

### File Variable Explanation

- **file_path**: Obtains the path of the current package, utilized for opening the `.cfg` file in the `HD-DM-Linux-SDK-5.0.1.16/cfg/DM_Quality_Cfg/` directory.

### Mode Selection

- **BlockingMode**: Selects the Block mode.

### Color Image Parameters

- **colorFormatNum**: Determines the transmission method for color images.
- **colorWidthNum**: Sets the width of the RGB image.
- **colorHeightNum**: Sets the height of the RGB image.
- **colorActualFpsNum**: Retrieves the frame rate of the RGB image.

### Depth Image Parameters

- **depthWidthNum**: Sets the width of the depth map.
- **depthHeightNum**: Sets the height of the depth map.
- **depthActualFpsNum**: Determines the transmission method for the depth map.
- **depthDataTypeNum**: Specifies the data transmission format for depth images.
- **buffSizeNum**: Sets the number of buffer digits.

### Publishing Options

- **colorImg**: Controls the publishing of RGB messages (0 for not publishing).
- **depthImg**: Controls the publishing of depth images (0 for not publishing).
- **pointCloudImg**: Controls the publishing of point cloud messages (0 for not publishing).
- **camera_info**: Controls the publishing of cameraInfo messages (0 for not publishing).

### Exposure and IR Settings

- **AE**: Selects the exposure mode (3 for automatic exposure, 1 for manual exposure).
- **irMaxValue**: Sets the maximum number of infrared (IR) values (0~15).
- **currentIRValue**: Sets the current IR number.

### Frame Rate Selection

- **frameSelection**: Sets the publishing frame rate (1 for 30 frames, 2 for 15 frames).

### Device Information

- **devInfoPID**: PID parameter of the camera.
- **devInfoVID**: Camera VID parameter.
- **devIndex**: Camera serial number.

## Usage

Ensure that ROS is properly configured and the necessary dependencies are installed. Adjust the configuration parameters as needed in the launch file (`bimu_camera.launch`) to suit your application requirements.

For further assistance or inquiries, please refer to the official Ydlidar documentation or contact support.

---
