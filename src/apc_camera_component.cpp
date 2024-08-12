#include "apc_camera_component.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace dmpreview
{

ApcCamera::ApcCamera(const rclcpp::NodeOptions& options)
    : Node("apc_node", options)
    , mQos(1)
{
    RCLCPP_INFO(get_logger(), "********************************");
    RCLCPP_INFO(get_logger(), "            APC Camera          ");
    RCLCPP_INFO(get_logger(), "********************************");
    RCLCPP_INFO(get_logger(), "node name: %s", get_name());
    RCLCPP_INFO(get_logger(), "ROS2 version : %s", EYS3D_ROS2_VERSION);
    RCLCPP_INFO(get_logger(), "wrapper version : %s", WRAPPER_VERSION);
    RCLCPP_INFO(get_logger(), "git branch : %s / commit id : %s", GIT_BRANCH, GIT_HASH);
    RCLCPP_INFO(get_logger(), "build time : %s", BUILD_TIMESTAMP);
    RCLCPP_INFO(get_logger(), "********************************");

    #ifdef _TEST
    publisher_ = this->create_publisher<std_msgs::msg::String>("dmpreviewcolor", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&ApcCamera::timer_callback, this));
    #endif //_TEST

    //set default params
    setDefaultParams();

    //get params from launch file
    getLanuchParams();

    //open device
    openDevice();

    //+Calibration info
    bool in_ok;
    StreamMode mStreamMode = getStreamModeIndex();
    auto&& in = getCameraIntrinsics(mStreamMode, &in_ok);
    if (in_ok) {
        RCLCPP_INFO(get_logger(), "Camera info is created, start preview ...");
    } else {
        RCLCPP_INFO(get_logger(), "Camera info is null, use default parameters.");
    }
    left_info_ptr = createCameraInfo(in.left);
    right_info_ptr = createCameraInfo(in.right);
    depth_info_ptr = createCameraInfo(in.left);
    //-Calibration info

    // Dynamic parameters callback
    set_on_parameters_set_callback(std::bind(&ApcCamera::paramChange_callback, this, _1));
}

ApcCamera::~ApcCamera() {
    RCLCPP_INFO(this->get_logger(), "Destroying Apc-node");
    deleteDevice();
}

#ifdef _TEST
void ApcCamera::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "ROS2 FOXY implement by eys3d sw! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}
#endif //_TEST

void ApcCamera::setDefaultParams() {

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    rcl_interfaces::msg::IntegerRange range;

    //is multi module
    descriptor.read_only = true;
    this->declare_parameter<bool>("multi_module", false, descriptor);
    //enable/disable stream
    this->declare_parameter<bool>("enable_color_stream", true, descriptor);
    this->declare_parameter<bool>("enable_depth_stream", true, descriptor);
    this->declare_parameter<bool>("enable_point_cloud_stream", false, descriptor);
    this->declare_parameter<bool>("enable_imu_stream", false, descriptor);

    //for camera params
    this->declare_parameter<int>("framerate", 30, descriptor);
    this->declare_parameter<int>("color_width", 1280, descriptor);
    this->declare_parameter<int>("color_height", 720, descriptor);
    this->declare_parameter<int>("color_stream_format", 0, descriptor);
    this->declare_parameter<int>("depth_width", 1280, descriptor);
    this->declare_parameter<int>("depth_height", 720, descriptor);
    this->declare_parameter<int>("depth_data_type", 4, descriptor);
    this->declare_parameter<bool>("interleave_mode", false, descriptor);
    this->declare_parameter<int>("depth_output_type", 2, descriptor);
    this->declare_parameter<int>("depth_maximum_mm", 1000, descriptor);
    this->declare_parameter<bool>("state_ae", true);
    this->declare_parameter<bool>("state_awb", true);
    this->declare_parameter<bool>("state_extendIR", false);


    this->declare_parameter<std::string>("dev_serial_number", "00000001", descriptor);
    this->declare_parameter<std::string>("kernel_name", "", descriptor);

    //for frame id
    this->declare_parameter<std::string>("left_color_frame", "left_color_frame", descriptor);
    this->declare_parameter<std::string>("right_color_frame", "right_color_frame", descriptor);
    this->declare_parameter<std::string>("depth_frame", "depth_frame", descriptor);
    this->declare_parameter<std::string>("points_frame", "points_frame", descriptor);
    this->declare_parameter<std::string>("imu_frame", "imu_frame", descriptor);
    this->declare_parameter<std::string>("imu_frame_processed", "imu_frame_processed", descriptor);

    // auto config camera mode
    range.set__from_value(0).set__to_value(50).set__step(1);
    descriptor.integer_range= {range};
    descriptor.description = "dynamic change video mode";
    descriptor.read_only = false;
    this->declare_parameter<int>("auto_config_camera_mode", 1, descriptor);

    range.set__from_value(0).set__to_value(96).set__step(1);
    descriptor.integer_range= {range};
    descriptor.description = "dynamic change IR intensity";
    this->declare_parameter<int>("ir_intensity", 3, descriptor);

    range.set__from_value(-13).set__to_value(3).set__step(1);
    descriptor.integer_range= {range};
    descriptor.description = "dynamic change exposure time step";
    this->declare_parameter<int>("exposure_time_step", -5, descriptor);

    range.set__from_value(2800).set__to_value(6500).set__step(1);
    descriptor.integer_range= {range};
    descriptor.description = "dynamic change white balance temperature";
    this->declare_parameter<int>("white_balance_temperature", 3000, descriptor);

    //for publish string
    this->declare_parameter<std::string>("left_color_topic", "/apc/left/image_color");
    this->declare_parameter<std::string>("right_color_topic", "/apc/right/image_color");
    this->declare_parameter<std::string>("depth_topic", "/apc/depth/image_raw");
    this->declare_parameter<std::string>("points_topic", "/apc/points/data_raw");
    this->declare_parameter<std::string>("imu_topic", "/apc/imu/data_raw");
    this->declare_parameter<std::string>("imu_processed_topic", "/apc/imu/data_raw_processed");

    //for QoS
    this->declare_parameter<int>("qos_history", RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    this->declare_parameter<int>("qos_depth", 5);
    this->declare_parameter<int>("qos_reliability", RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    this->declare_parameter<int>("qos_durability", RMW_QOS_POLICY_DURABILITY_VOLATILE);

}
void ApcCamera::getLanuchParams() {
    RCLCPP_INFO(get_logger(), "load lanuch params ...");
    //+get params from launch
    this->get_parameter("multi_module", params_.multi_module_);

    this->get_parameter("enable_color_stream", params_.enable_color_stream_);
    this->get_parameter("enable_depth_stream", params_.enable_depth_stream_);
    this->get_parameter("enable_point_cloud_stream", params_.enable_point_cloud_stream_);
    this->get_parameter("enable_imu_stream", params_.enable_imu_stream_);

    this->get_parameter("auto_config_camera_mode", params_.auto_config_camera_mode_);

    this->get_parameter("framerate", params_.framerate_);
    this->get_parameter("color_width", params_.color_width_);
    this->get_parameter("color_height", params_.color_height_);
    this->get_parameter("color_stream_format", params_.color_stream_format_);

    this->get_parameter("depth_width", params_.depth_width_);
    this->get_parameter("depth_height", params_.depth_height_);
    this->get_parameter("depth_data_type", params_.depth_data_type_);
    this->get_parameter("interleave_mode", params_.interleave_mode_);

    int depth_output_type;
    this->get_parameter("depth_output_type", depth_output_type);
    params_.depth_output_type_ = (DepthOutputType) depth_output_type;

    this->get_parameter("depth_maximum_mm", params_.z_maximum_mm_);
    this->get_parameter("state_ae", params_.auto_exposure_);
    this->get_parameter("state_awb", params_.auto_white_balance_);
    this->get_parameter("exposure_time_step", params_.exposure_time_step_);
    this->get_parameter("white_balance_temperature", params_.white_balance_temperature_);
    this->get_parameter("state_extendIR", params_.extend_ir_);
    this->get_parameter("ir_intensity", params_.ir_intensity_);
    this->get_parameter("dev_serial_number", params_.serial_number_);
    this->get_parameter("kernel_name", params_.kernel_name_);

    this->get_parameter("left_color_frame", left_color_frame_id);
    this->get_parameter("right_color_frame", right_color_frame_id);
    this->get_parameter("depth_frame", depth_frame_id);
    this->get_parameter("points_frame", points_frame_id);
    this->get_parameter("imu_frame", imu_frame_id);
    this->get_parameter("imu_frame_processed", imu_frame_processed_id);

    this->get_parameter("left_color_topic", left_color_topic);
    this->get_parameter("right_color_topic", right_color_topic);
    this->get_parameter("depth_topic", depth_topic);
    this->get_parameter("points_topic", points_topic);
    this->get_parameter("imu_topic", imu_topic);
    this->get_parameter("imu_processed_topic", imu_processed_topic);
    //-get params from launch

    //+get QoS porfile
    rclcpp::Parameter paramVal;
    if (this->get_parameter("qos_history", paramVal))
    {
      params_.qos_hist_ = paramVal.as_int() == 1 ? RMW_QOS_POLICY_HISTORY_KEEP_LAST : RMW_QOS_POLICY_HISTORY_KEEP_ALL;
      mQos.history(params_.qos_hist_);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", "qos_history");
    }
    RCLCPP_INFO(get_logger(), " * QoS History: %s", qos2str(params_.qos_hist_).c_str());

    if (this->get_parameter("qos_depth", paramVal))
    {
      params_.qos_depth_ = paramVal.as_int();
      mQos.keep_last(params_.qos_depth_);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", "qos_depth");
    }
    RCLCPP_INFO(get_logger(), " * QoS History depth: %d", params_.qos_depth_);

    if (get_parameter("qos_reliability", paramVal))
    {
      params_.qos_reliability_ =
          paramVal.as_int() == 1 ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      mQos.reliability(params_.qos_reliability_);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", "qos_reliability");
    }
    RCLCPP_INFO(get_logger(), " * QoS Reliability: %s", qos2str(params_.qos_reliability_).c_str());

    if (get_parameter("qos_durability", paramVal))
    {
      params_.qos_durability_ =
          paramVal.as_int() == 1 ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL : RMW_QOS_POLICY_DURABILITY_VOLATILE;
      mQos.durability(params_.qos_durability_);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "The parameter '%s' is not available, using the default value", "qos_durability");
    }
    RCLCPP_INFO(get_logger(), " * QoS Durability: %s", qos2str(params_.qos_durability_).c_str());
    //-get QoS porfile

    //+publishers
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

    print_qos(mQos);

    pub_left_color = image_transport::create_camera_publisher(this, left_color_topic, mQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << pub_left_color.getTopic());

    pub_depth = image_transport::create_camera_publisher(this, depth_topic, mQos.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << pub_depth.getTopic());

    pub_points = create_publisher<sensor_msgs::msg::PointCloud2>(points_topic, mQos/*1*/);
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << pub_points->get_topic_name());

    rclcpp::QoS mPoseQos(1);
    pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>(imu_topic, mPoseQos);
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << pub_pose->get_topic_name());
    //-publishers

    RCLCPP_INFO(get_logger(), "...load lanuch params done");
}

void ApcCamera::getModeConfig(int mode) {
    RCLCPP_INFO(get_logger(), "input camera mode : %d", mode);
    moduleModeConfig_ = {
        0 , 1280 ,720 , 30 ,1280 ,720, 4, false
    };

    if (!device_) {
        RCLCPP_INFO(this->get_logger(), "getModeConfig(), can not get device instance");
        exit(-1);
    }
    auto currentUSBType = device_->getUsbPortType();
    unsigned short pid = device_->getCameraDeviceInfo().devInfo.wPID;
    modeConfigOptions = device_->getModeConfigOptions(currentUSBType, pid);

    for (auto& modeItem : modeConfigOptions->GetModes()) {
        if (modeItem.iMode == mode && APC_OK == modeConfigOptions->SelectCurrentIndex(mode)) {
            RCLCPP_INFO(get_logger(), "Found input camera mode : %d", mode);
            break;
        }
    }
    RCLCPP_INFO(get_logger(), "select camera mode : %d", modeConfigOptions->GetCurrentIndex());
    modeConfig = modeConfigOptions->GetCurrentModeInfo();
    RCLCPP_INFO(get_logger(), "iMode: %d, iUSB_Type: %d, iInterLeaveModeFPS: %d, bRectifyMode: %d\n",
             modeConfig.iMode, modeConfig.iUSB_Type, modeConfig.iInterLeaveModeFPS,
             modeConfig.bRectifyMode);

    RCLCPP_INFO(get_logger(), "eDecodeType_L: %d, L_Resolution.Width: %d ,L_Resolution.Height: %d, "
                      "D_Resolution.Width: %d, D_Resolution.Height: %d, vecDepthType: %d, vecColorFps: %d, vecDepthFps: %d\n",
                      modeConfig.eDecodeType_L, modeConfig.L_Resolution.Width, modeConfig.L_Resolution.Height,
                      modeConfig.D_Resolution.Width, modeConfig.D_Resolution.Height,
                      modeConfig.vecDepthType.empty() ? 0 : modeConfig.vecDepthType.at(0),
                      modeConfig.vecColorFps.empty() ? 0 : modeConfig.vecColorFps.at(0),
                      modeConfig.vecDepthFps.empty() ? 0 : modeConfig.vecDepthFps.at(0));


    if (modeConfig.vecDepthType.empty()) {
        depth_raw_data_type = libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_OFF_RAW;
    } else {
        switch(modeConfig.vecDepthType.at(0)){
            case 8:
                depth_raw_data_type = modeConfig.bRectifyMode ?
                                            libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS:
                                            libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_8_BITS_RAW;
                break;
            case 11:
                depth_raw_data_type = modeConfig.bRectifyMode ?
                                            libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS:
                                            libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_11_BITS_RAW;
                break;
            case 14:
                depth_raw_data_type = modeConfig.bRectifyMode ?
                                            libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS:
                                            libeYs3D::video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_14_BITS_RAW;
                break;
        }
    }

    if ((device_->getCameraDeviceInfo().devInfo.wPID == 0x120 || device_->getCameraDeviceInfo().devInfo.wPID == 0x137) 
            && modeConfig.D_Resolution.Height == 360) {
        depth_raw_data_type = static_cast<libeYs3D::video::DEPTH_RAW_DATA_TYPE>(depth_raw_data_type + APC_DEPTH_DATA_SCALE_DOWN_MODE_OFFSET);
        RCLCPP_INFO(get_logger(), "set scale down mode");
    }

    if (modeConfig.iInterLeaveModeFPS > 0) {
        depth_raw_data_type = static_cast<libeYs3D::video::DEPTH_RAW_DATA_TYPE>(depth_raw_data_type + APC_DEPTH_DATA_INTERLEAVE_MODE_OFFSET);
        moduleModeConfig_.interLeaveMode = true;
        RCLCPP_INFO(get_logger(), "this mode supports interleave mode");
    } else {
        moduleModeConfig_.interLeaveMode = false;
    }

    moduleModeConfig_.videoMode = depth_raw_data_type;
    moduleModeConfig_.colorFormat = modeConfig.eDecodeType_L == ModeConfig::MODE_CONFIG::YUYV ?
            libeYs3D::video::COLOR_RAW_DATA_YUY2 : libeYs3D::video::COLOR_RAW_DATA_MJPG;
    moduleModeConfig_.colorWidth = modeConfig.L_Resolution.Width;
    moduleModeConfig_.colorHeight = modeConfig.L_Resolution.Height;
    moduleModeConfig_.fps = !modeConfig.vecColorFps.empty() ? modeConfig.vecColorFps.at(0) : modeConfig.vecDepthFps.at(0);
    moduleModeConfig_.depthWidth = modeConfig.D_Resolution.Width;
    moduleModeConfig_.depthHeight = modeConfig.D_Resolution.Height;

    RCLCPP_INFO(get_logger(), "mode config: {%s, %d, %d, %d fps, %d, %d, %d, %s} depth_raw_data_type=%d\n", 
            (moduleModeConfig_.colorFormat == ModeConfig::MODE_CONFIG::YUYV) ? "YUYV":"MJPG", 
            moduleModeConfig_.colorWidth, moduleModeConfig_.colorHeight, moduleModeConfig_.fps,
            moduleModeConfig_.depthWidth, moduleModeConfig_.depthHeight, moduleModeConfig_.videoMode,
            moduleModeConfig_.interLeaveMode ? "True" : "False", moduleModeConfig_.videoMode);
}

void ApcCamera::openDevice() {

    if ( eYs3DSystem_ != nullptr) {
        eYs3DSystem_.reset();
        eYs3DSystem_ = nullptr;
    }

    RCLCPP_INFO(get_logger(), "get eYs3DSystem instance");
    eYs3DSystem_ = std::make_shared<libeYs3D::EYS3DSystem>(libeYs3D::EYS3DSystem::COLOR_BYTE_ORDER::COLOR_RGB24);//libeYs3D::EYS3DSystem::getEYS3DSystem();

    if (!eYs3DSystem_) {
        RCLCPP_INFO(this->get_logger(), "can not get eYs3DSystem instance");
        exit(-1);
    }

    int index = 0;
    std::shared_ptr<libeYs3D::devices::CameraDevice> device;
    if (params_.multi_module_) { 
        RCLCPP_INFO(get_logger(), "launch multi modules");
        if (!params_.serial_number_.empty() || !params_.kernel_name_.empty()) {
            index = -1;

            for (int i = 0; i < eYs3DSystem_->getCameraDeviceCount() ; ++i) {
                device = eYs3DSystem_->getCameraDevice(i);

                if (!device) continue;

                if (!params_.serial_number_.empty()) {
                    if (strcmp(params_.serial_number_.c_str(),
                        device->getCameraDeviceInfo().serialNumber)) {
                        continue;
                    }
                }else {
                    if (strcmp(params_.kernel_name_.c_str(),
                        device->getCameraDeviceInfo().busInfo)) {
                        continue;
                    }
                }

                index = i;
                break;
            }
        }
    } else {
        RCLCPP_INFO(get_logger(), "launch single module");
        index = -1;
        for (int i = 0; i < eYs3DSystem_->getCameraDeviceCount() ; ++i) {
            device = eYs3DSystem_->getCameraDevice(i);

            if (!device) continue;

            if (!params_.serial_number_.empty()) {
                RCLCPP_INFO(this->get_logger(), "serial number: %s", params_.serial_number_.c_str());
                if (strcmp(params_.serial_number_.c_str(),
                    device->getCameraDeviceInfo().serialNumber)) {
                    continue;
                }
            }

            index = i;
            break;
        }
    }

    if(device != nullptr) {
        device.reset();
        device = nullptr;
    }

    RCLCPP_INFO(this->get_logger(), "find device index: '%d'", index);
    if (device_ == nullptr) {
        RCLCPP_INFO(this->get_logger(), "get device instance ...");
        device_ = eYs3DSystem_->getCameraDevice(index);
    }

    if (!device_) {
        RCLCPP_INFO(this->get_logger(), "can not get device instance");
        exit(-1);
    }
    RCLCPP_INFO(this->get_logger(), "... get device instance done \n");

    //get camera mode config from database
    if (params_.auto_config_camera_mode_ > 0) {
        getModeConfig(params_.auto_config_camera_mode_);

        params_.framerate_ = moduleModeConfig_.fps;
        params_.color_stream_format_ = moduleModeConfig_.colorFormat ;
        params_.color_width_ = moduleModeConfig_.colorWidth;
        params_.color_height_ = moduleModeConfig_.colorHeight;
        params_.depth_width_ = moduleModeConfig_.depthWidth;
        params_.depth_height_ = moduleModeConfig_.depthHeight;
        params_.depth_data_type_ = moduleModeConfig_.videoMode;
        params_.interleave_mode_ = moduleModeConfig_.interLeaveMode;
    }

    if (params_.enable_point_cloud_stream_) {
        point_msg.reset();
        point_msg = nullptr;
        point_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        point_msg->width = params_.depth_width_;
        point_msg->height = params_.depth_height_;
        point_msg->is_dense = true;
        point_msg->header.frame_id = points_frame_id;

        point_modifier.reset();
        point_modifier = nullptr;
        sensor_msgs::PointCloud2Modifier point_modifier(*(point_msg.get()));    
        /*point_modifier.setPointCloud2Fields(
            4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, 
                "y", 1, sensor_msgs::msg::PointField::FLOAT32, 
                "z", 1, sensor_msgs::msg::PointField::FLOAT32, 
                "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

        point_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");*/
        
        point_modifier.setPointCloud2Fields(
            3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, 
                "y", 1, sensor_msgs::msg::PointField::FLOAT32, 
                "z", 1, sensor_msgs::msg::PointField::FLOAT32);

        point_modifier.setPointCloud2FieldsByString(1, "xyz");
    }

    libeYs3D::devices::IRProperty property = device_->getIRProperty();
    property.setIRValue(params_.ir_intensity_);
    property.enableExtendIR(params_.extend_ir_);
    device_->setIRProperty(property);

    device_->setCameraDevicePropertyValue(
        libeYs3D::devices::CameraDeviceProperties::AUTO_EXPOSURE,
        params_.auto_exposure_);

    device_->setCameraDevicePropertyValue(
        libeYs3D::devices::CameraDeviceProperties::AUTO_WHITE_BLANCE,
        params_.auto_white_balance_);

    if (!params_.auto_exposure_){
        device_->setCameraDevicePropertyValue(
            libeYs3D::devices::CameraDeviceProperties::EXPOSURE_TIME,
            params_.exposure_time_step_);
    }

    if (!params_.auto_white_balance_){
        device_->setCameraDevicePropertyValue(
            libeYs3D::devices::CameraDeviceProperties::WHITE_BLANCE_TEMPERATURE,
            params_.white_balance_temperature_);
    }

    mColorStreamCallback = std::bind(&ApcCamera::color_image_callback, this, std::placeholders::_1);
    mDepthStreamCallback = std::bind(&ApcCamera::depth_image_callback, this, std::placeholders::_1);
    mPCStreamCallback = std::bind(&ApcCamera::pc_frame_callback, this, std::placeholders::_1);
    mIMUStreamCallback = std::bind(&ApcCamera::imu_data_callback, this, std::placeholders::_1);

    int ret = device_->initStream(
        (libeYs3D::video::COLOR_RAW_DATA_TYPE)params_.color_stream_format_,
        params_.color_width_, params_.color_height_, params_.framerate_,
        (libeYs3D::video::DEPTH_RAW_DATA_TYPE)params_.depth_data_type_,
        params_.depth_width_, params_.depth_height_,
        DEPTH_IMG_COLORFUL_TRANSFER, IMAGE_SN_SYNC, 0,
        params_.enable_color_stream_ ? mColorStreamCallback : nullptr,
        params_.enable_depth_stream_ ? mDepthStreamCallback : nullptr,
        params_.enable_point_cloud_stream_ ? mPCStreamCallback : nullptr,
        params_.enable_imu_stream_ ? mIMUStreamCallback : nullptr);

    //enable/disable insterleave mode if device support
    bool isSupportInterLeaveMode = device_->isInterleaveModeSupported();
    RCLCPP_INFO(get_logger(), "isInterleaveModeSupported : %s", isSupportInterLeaveMode ? "true":"false");
    RCLCPP_INFO(get_logger(), "set Interleave Mode : %s",params_.interleave_mode_ ? "true" : "False");
    if (isSupportInterLeaveMode) {
        ret = device_->enableInterleaveMode(params_.interleave_mode_);
        if ( ret == APC_OK ) {
            RCLCPP_INFO(get_logger(), "is InterLeave Mode enabled: %s",device_->isInterleaveModeEnabled() ? "true":"false");
        } else {
            RCLCPP_INFO(get_logger(), "enable Interleave Mode failure reason: %d",ret);
        }
    }
    //start stream
    device_->enableStream();

    //+imu info
    if (device_) {
        libeYs3D::devices::IMUDevice::IMUDeviceInfo sInfo;
        sInfo = device_->getIMUDeviceInfo();
        RCLCPP_INFO(this->get_logger(), "IMU-Status: %s , IMU-type: %s\n",
            sInfo.status ? "enable" : "disable", (sInfo.nType == 1 ) ? "9-AXIS" : ((sInfo.nType == 0) ? "6-AXIS" : "unknown"));
    }
    //-imu info
    uint16_t z_near, z_far;
    device_->getDepthOfField(&z_near, &z_far);
    device_->setDepthOfField(z_near, params_.z_maximum_mm_);
}

void ApcCamera::dynamicChangeMode(int mode) {
    device_->pauseStream();
    deleteDevice();

    //get params from launch file
    getLanuchParams();

    params_.auto_config_camera_mode_ = mode;

    //open device
    openDevice();
}

void ApcCamera::closeDevice() {
    RCLCPP_INFO(get_logger(), "closeDevice()");
    if(device_) {
        device_->closeStream();
        device_->enableInterleaveMode(false);
    }
}

void ApcCamera::deleteDevice() {
    RCLCPP_INFO(get_logger(), "deleteDevice()");
    closeDevice();
    device_.reset();
    device_ = nullptr;
    //release all instance
    if (eYs3DSystem_ != nullptr) {
        eYs3DSystem_.reset();
        eYs3DSystem_ = nullptr;
    }
}

bool ApcCamera::color_image_callback(const libeYs3D::video::Frame *frame) {
    //if (0 == count_subscribers(pub_left_color.getTopic())) return true;

#ifdef _USE_CV
    std_msgs::msg::Header header;
    //header.stamp = rclcpp::Clock().now();
    header.stamp = frameTime2Ros(frame->tsUs);
    header.frame_id = left_color_frame_id;

    cv::Mat mat =
        cv::Mat(frame->height, frame->width, CV_8UC3, (void *)frame->rgbVec.data());
    auto &&color_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mat).toImageMsg();
#else //_USE_CV
    cv::Mat mat =
        cv::Mat(frame->height, frame->width, CV_8UC3, (void *)frame->rgbVec.data());

    std::shared_ptr<sensor_msgs::msg::Image> color_msg;
    color_msg = ConvertFrameToMessage(mat, sensor_msgs::image_encodings::BGR8);
    color_msg->header.stamp = rclcpp::Clock().now();
    color_msg->header.frame_id = left_color_frame_id; 
#endif //_USE_CV

    if(left_info_ptr)
    {
        left_info_ptr->header.stamp = color_msg->header.stamp;
        left_info_ptr->header.frame_id = left_color_frame_id;
    }
    pub_left_color.publish(color_msg, left_info_ptr);

    return true;
}

bool ApcCamera::depth_image_callback(const libeYs3D::video::Frame *frame) {
    //if (0 == rclcpp::Node::count_subscribers(pub_depth.getTopic())) return true;

#ifdef _USE_CV
    std_msgs::msg::Header header;
    //header.stamp = rclcpp::Clock().now();
    header.stamp = frameTime2Ros(frame->tsUs);
    header.frame_id = depth_frame_id;

    cv::Mat mat =
        cv::Mat(frame->height, frame->width, CV_8UC3, (void *)frame->rgbVec.data());
    auto &&depth_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, mat).toImageMsg();

#else //_USE_CV
    std::shared_ptr<sensor_msgs::msg::Image> depth_msg;
    cv::Mat mat;

    switch (params_.depth_output_type_){
        case DepthOutputType::Depth_Output_Raw: {
            mat = cv::Mat(frame->height, frame->width, CV_16UC1,
                        (void *)frame->dataVec.data());
            depth_msg = ConvertFrameToMessage(mat, sensor_msgs::image_encodings::MONO16);
            break;
        }
        case DepthOutputType::Depth_Output_Gray: {
            mat = cv::Mat(frame->height, frame->width, CV_16UC1,
                        (void *)frame->zdDepthVec.data());
            depth_msg = ConvertFrameToMessage(mat, sensor_msgs::image_encodings::TYPE_16UC1);
            break;
        }
        case DepthOutputType::Depth_Output_Colorful: {
            mat = cv::Mat(frame->height, frame->width, CV_8UC3, 
                        (void *)frame->rgbVec.data());
            depth_msg = ConvertFrameToMessage(mat, sensor_msgs::image_encodings::RGB8);
            break;
        }
      }

    depth_msg->header.stamp = rclcpp::Clock().now();
    depth_msg->header.frame_id = depth_frame_id;
#endif //_USE_CV

    if(depth_info_ptr)
    {
        depth_info_ptr->header.stamp = depth_msg->header.stamp;
        depth_info_ptr->header.frame_id = depth_frame_id;
    }
    pub_depth.publish(depth_msg, depth_info_ptr);

    return true;
}

bool ApcCamera::pc_frame_callback(const libeYs3D::video::PCFrame *pcFrame) {

    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_msg, "x");
    //sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(*point_msg, "rgb");
    
    for (int index = 0; index < params_.depth_width_ * params_.depth_height_; ++index) {
        iter_x[0] = pcFrame->xyzDataVec[index * 3] / 1000.0f;
        iter_x[1] = pcFrame->xyzDataVec[index * 3 + 1] / 1000.0f;
        iter_x[2] = pcFrame->xyzDataVec[index * 3 + 2] / 1000.0f;


        //iter_rgb[2] = pcFrame->rgbDataVec[index * 3 + 2];
        //iter_rgb[1] = pcFrame->rgbDataVec[index * 3 + 1];
        //iter_rgb[0] = pcFrame->rgbDataVec[index * 3];

        ++iter_x; 
        //++iter_rgb;
    }

    //point_msg->header.stamp = rclcpp::Clock().now();
    point_msg->header.stamp = frameTime2Ros(pcFrame->tsUs);
    pub_points->publish(*point_msg);

    return true;
}

bool ApcCamera::imu_data_callback(const libeYs3D::sensors::SensorData *sensorData) {

    if (!sensorData) {
        RCLCPP_INFO(get_logger(),"IMU callback without data");
        return false;
    }
    if (sensorData->type == libeYs3D::sensors::SensorData::SensorDataType::IMU_DATA) {
        IMUData *pImuData = (IMUData *) sensorData->data;

    #ifdef _IMU_DEBUG
        char buffer[2048];
        double roll, pitch, yaw;
        sensorData->toString(buffer, sizeof(buffer));

        RCLCPP_INFO(get_logger(),"serialNumber : %d \n",sensorData->serialNumber);
        RCLCPP_INFO(get_logger(),"data : %s \n",buffer);
        RCLCPP_INFO(get_logger(),"q[0] : %.2f, q[1] : %.2f, q[2] : %.2f, q[3] : %.2f \n",
            pImuData->_quaternion[0], pImuData->_quaternion[1], pImuData->_quaternion[2], pImuData->_quaternion[3]);

        tf2::Quaternion imu_quat(
            pImuData->_quaternion[0],
            pImuData->_quaternion[1],
            pImuData->_quaternion[2],
            pImuData->_quaternion[3]);

        tf2::Matrix3x3 m(imu_quat);
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(get_logger(),"roll : %.2f, pitch : %.2f, yaw : %.2f", roll*57.295779513, pitch*57.295779513, yaw*57.295779513);
    #endif //_IMU_DEBUG

        std_msgs::msg::Header header;
        header.stamp = rclcpp::Clock().now();
        header.frame_id = imu_frame_id;

        geometry_msgs::msg::Pose pose;

        // Add all value in Pose message
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = pImuData->_quaternion[0];
        pose.orientation.y = pImuData->_quaternion[1];
        pose.orientation.z = pImuData->_quaternion[2];
        pose.orientation.w = pImuData->_quaternion[3];

        poseMsgPtr poseNoCov = std::make_unique<geometry_msgs::msg::PoseStamped>();
        poseNoCov->header = header;
        poseNoCov->pose = pose;
        // Publish pose stamped message
        pub_pose->publish(*poseNoCov);

    }
    return true;
}

rcl_interfaces::msg::SetParametersResult ApcCamera::paramChange_callback(std::vector<rclcpp::Parameter> parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    for (const auto& param : parameters)
    {
        if (param.get_name() == "ir_intensity") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            double val = param.as_int();
            libeYs3D::devices::IRProperty property = device_->getIRProperty();
            int IRMin = property.getIRMin();
            int IRMax = property.getIRMax();
            if ((val < IRMin) || (val > IRMax)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [" + std::to_string(IRMin) + "," + std::to_string(IRMax) + "]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            params_.ir_intensity_ = val;
            // libeYs3D::devices::IRProperty property = device_->getIRProperty();
            property.setIRValue(params_.ir_intensity_);
            device_->setIRProperty(property);

            RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << param.get_name() << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";

            return result;
        } else if (param.get_name() == "state_extendIR") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            bool val = param.as_bool();
            if(params_.extend_ir_ != val && device_ != nullptr) {
                libeYs3D::devices::IRProperty property = device_->getIRProperty();
                property.enableExtendIR(val);
                device_->setIRProperty(property);
                params_.extend_ir_ = val;
            }
            RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << param.get_name() << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";
            return result;
        } else if (param.get_name() == "exposure_time_step") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            if (params_.auto_exposure_){
                result.successful = false;
                result.reason = param.get_name() + " 'state_ae' must be set to false";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            double val = param.as_int();
            if ((val < -13) || (val > 3)) {
                result.successful = false;
                result.reason = param.get_name() + " must be an integer in the range [-13, 3]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            if(params_.exposure_time_step_ != val && device_ != nullptr) {
                device_->setCameraDevicePropertyValue(
                    libeYs3D::devices::CameraDeviceProperties::EXPOSURE_TIME, val);
                params_.exposure_time_step_ = val;
            }

            RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << param.get_name() << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";

            return result;
        } else if (param.get_name() == "state_ae") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            bool val = param.as_bool();

            if(params_.auto_exposure_ != val && device_ != nullptr) {
                device_->setCameraDevicePropertyValue(
                    libeYs3D::devices::CameraDeviceProperties::AUTO_EXPOSURE, val);
                params_.auto_exposure_ = val;
            }
            RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << param.get_name() << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";

            return result;
        } else if (param.get_name() == "state_awb") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_BOOL;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            bool val = param.as_bool();

            if(params_.auto_white_balance_ != val && device_ != nullptr) {
                device_->setCameraDevicePropertyValue(
                    libeYs3D::devices::CameraDeviceProperties::AUTO_WHITE_BLANCE, val);
                params_.auto_white_balance_ = val;
            }
            RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << param.get_name() << "' correctly set to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";

            return result;
        } else if (param.get_name() == "white_balance_temperature") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            if (params_.auto_white_balance_){
                result.successful = false;
                result.reason = param.get_name() + " 'state_awb' must be set to false";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            double val = param.as_int();
            if ((val < 2800) || (val > 6500)) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [2800, 6500]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            if(params_.white_balance_temperature_ != val && device_ != nullptr) {
                device_->setCameraDevicePropertyValue(
                    libeYs3D::devices::CameraDeviceProperties::WHITE_BLANCE_TEMPERATURE, val);
                params_.white_balance_temperature_ = val;
            }

            RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << param.get_name() << "' correctly set to " << val );
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";

            return result;
        } else if (param.get_name() == "auto_config_camera_mode") {
            rclcpp::ParameterType correctType = rclcpp::ParameterType::PARAMETER_INTEGER;
            if (param.get_type() != correctType) {
                result.successful = false;
                result.reason = param.get_name() + " must be a " + rclcpp::to_string(correctType);
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            double val = param.as_int();

            if (val < 0) {
                result.successful = false;
                result.reason = param.get_name() + " must be a positive integer in the range [ mode > 0]";
                RCLCPP_WARN_STREAM(get_logger(), result.reason);
                return result;
            }

            params_.auto_config_camera_mode_ = val;
            dynamicChangeMode(params_.auto_config_camera_mode_);
            RCLCPP_INFO_STREAM(get_logger(), "Parameter '" << param.get_name() << "' correctly set video mode to " << val);
            result.successful = true;
            result.reason = param.get_name() + " correctly set.";

            return result;
        } else {
            result.reason = param.get_name() + " is not a dynamic parameter";
        }
    }
    RCLCPP_WARN_STREAM(get_logger(), result.reason);
    return result;
}

std::shared_ptr<sensor_msgs::msg::Image> ApcCamera::ConvertFrameToMessage(cv::Mat &frame, const std::string enc)
{
    std_msgs::msg::Header header_;
    sensor_msgs::msg::Image ros_image;

    ros_image.header = header_;
    ros_image.height = frame.rows;
    ros_image.width = frame.cols;
    ros_image.encoding = enc;
    ros_image.is_bigendian = false;
    ros_image.step = frame.cols * frame.elemSize();
    size_t size = ros_image.step * frame.rows;
    ros_image.data.resize(size);

    if (frame.isContinuous())
    {
        memcpy(reinterpret_cast<char *>(&ros_image.data[0]), frame.data, size);
    }
    else
    {
        // Copy by row by row
        uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
        uchar *cv_data_ptr = frame.data;
        for (int i = 0; i < frame.rows; ++i)
        {
            memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
            ros_data_ptr += ros_image.step;
            cv_data_ptr += frame.step;
        }
    }

    auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
    return msg_ptr_;
}

rclcpp::Time ApcCamera::frameTime2Ros(uint64_t t, rcl_clock_type_t clock_type) {
    uint64_t ts_msec = t;
    uint32_t sec = static_cast<uint32_t>(ts_msec / 1000000);
    uint32_t nsec = static_cast<uint32_t>(ts_msec % 1000000);
    return rclcpp::Time(sec, nsec, clock_type);
}

StreamMode ApcCamera::getStreamModeIndex() {
    StreamMode mStreamMode;

    if ( params_.color_width_ == 640 && params_.color_height_ == 480) {
        mStreamMode = StreamMode::STREAM_640x480;
    } else if (params_.color_width_ == 1280 && params_.color_height_ == 480){
        mStreamMode = StreamMode::STREAM_1280x480;
    }else if (params_.color_width_ == 1280 && params_.color_height_ == 720){
        mStreamMode = StreamMode::STREAM_1280x720;
    }else if (params_.color_width_ == 2560 && params_.color_height_ == 720){
        mStreamMode = StreamMode::STREAM_2560x720;
    }else if (params_.color_width_ == 640 && params_.color_height_ == 400){
        mStreamMode = StreamMode::STREAM_640x400;
    }else{
        mStreamMode = StreamMode::STREAM_CUSTOM;
    }
    return mStreamMode;
}

StreamIntrinsics ApcCamera::getCameraIntrinsics( const StreamMode& stream_mode, bool* ok) {

    auto in = getCameraRectifyLog(stream_mode, ok);
    if (*ok) return in;

    // if false, return default intrinsics
    // {w, h, fx, fy, cx, cy, coeffs[5]{k1,k2,p1,p2,k3}}
    CameraIntrinsics cam_in;

    switch (stream_mode) {
        case StreamMode::STREAM_640x480:
            cam_in = {640, 480, 979.8, 942.8, 682.3 / 2, 254.9, {0, 0, 0, 0, 0}};
            break;
        case StreamMode::STREAM_1280x480:
            cam_in = {640, 480, 979.8, 942.8, 682.3, 254.9, {0, 0, 0, 0, 0}};
            break;
        case StreamMode::STREAM_1280x720:
            cam_in = {1280, 720, 979.8, 942.8, 682.3, 254.9 * 2, {0, 0, 0, 0, 0}};
            break;
        case StreamMode::STREAM_2560x720:
            cam_in = {1280, 720, 979.8, 942.8, 682.3 * 2, 254.9 * 2, {0, 0, 0, 0, 0}};
            break;
        default:
            cam_in = {1280, 720, 979.8, 942.8, 682.3, 254.9 * 2, {0, 0, 0, 0, 0}};
            break;
    }
    return {cam_in, cam_in};
}

StreamIntrinsics ApcCamera::getCameraRectifyLog(const StreamMode& stream_mode, bool* ok) {

    StreamIntrinsics in;
    int index = getRectifyLogIndex(stream_mode);

    RCLCPP_INFO(get_logger(), "get camera rectify log");
    if (device_ == nullptr){
        *ok = false;
        return std::move(in);
    }

    auto calib = device_->getRectifyLogData(index);
    if (calib == nullptr || calib->InImgWidth == 0) {
        *ok = false;
        return std::move(in);
    }

    in.left.width = calib->InImgWidth/2;
    in.left.height = calib->InImgHeight;
    in.left.fx = calib->CamMat1[0];
    in.left.fy = calib->CamMat1[4];
    in.left.cx = calib->CamMat1[2];
    in.left.cy = calib->CamMat1[5];
    for (int i = 0; i < 5; i++) {
        in.left.coeffs[i] = calib->CamDist1[i];
    }
    for (int i = 0; i < 12; i++) {
        in.left.p[i] = calib->NewCamMat1[i];
    }
    for (int i = 0; i < 9; i++) {
        in.left.r[i] = calib->LRotaMat[i];
    }
    in.right.width = calib->InImgWidth/2;
    in.right.height = calib->InImgHeight;
    in.right.fx = calib->CamMat2[0];
    in.right.fy = calib->CamMat2[4];
    in.right.cx = calib->CamMat2[2];
    in.right.cy = calib->CamMat2[5];
    for (int i = 0; i < 5; i++) {
        in.right.coeffs[i] = calib->CamDist2[i];
    }
    for (int i = 0; i < 12; i++) {
        in.right.p[i] = calib->NewCamMat2[i];
    }
    for (int i = 0; i < 9; i++) {
        in.right.r[i] = calib->RRotaMat[i];
    }

    in.ply_parameters.out_witdh = calib->OutImgWidth;
    in.ply_parameters.out_height = calib->OutImgHeight;
    memcpy(in.ply_parameters.ReProjectMat, calib->ReProjectMat, sizeof(float) * 16);

    *ok = true;
    RCLCPP_INFO(get_logger(), "... done");
    return std::move(in);
}

int ApcCamera::getRectifyLogIndex(StreamMode stream_mode) {
    int index;
    switch (stream_mode) {
        case StreamMode::STREAM_640x480:
        case StreamMode::STREAM_640x400:
        case StreamMode::STREAM_1280x480:  // 480p, vga
            index = 1; break;      
        case StreamMode::STREAM_1280x720:
        case StreamMode::STREAM_2560x720:  // 720p, hd
            index = 0; break;
        default:
            index =  1;
        break;
        // throw new std::runtime_error("StreamMode is unknown");
    }
    return index;
}

camInfoMsgPtr ApcCamera::createCameraInfo(const CameraIntrinsics& in) {

    camInfoMsgPtr camera_info_ptr = std::make_shared<sensor_msgs::msg::CameraInfo>();

    camera_info_ptr->width = in.width;
    camera_info_ptr->height = in.height;

    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    camera_info_ptr->k[0] = in.fx;
    camera_info_ptr->k[2] = in.cx;
    camera_info_ptr->k[4] = in.fy;
    camera_info_ptr->k[5] = in.cy;
    camera_info_ptr->k[8] = 1;

    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    camera_info_ptr->p[0] = in.p[0];
    camera_info_ptr->p[2] = in.p[2];
    camera_info_ptr->p[3] = in.p[3];
    camera_info_ptr->p[5] = in.p[5];
    camera_info_ptr->p[6] = in.p[6];
    camera_info_ptr->p[10] = in.p[10];

    camera_info_ptr->distortion_model = "plumb_bob";

    // D of plumb_bob: (k1, k2, t1, t2, k3)
    for (int i = 0; i < 5; i++) {
      camera_info_ptr->d.push_back(in.coeffs[i]);
    }

    // R to identity matrix
    camera_info_ptr->r[0] = in.r[0];
    camera_info_ptr->r[1] = in.r[1];
    camera_info_ptr->r[2] = in.r[2];
    camera_info_ptr->r[3] = in.r[3];
    camera_info_ptr->r[4] = in.r[4];
    camera_info_ptr->r[5] = in.r[5];
    camera_info_ptr->r[6] = in.r[6];
    camera_info_ptr->r[7] = in.r[7];
    camera_info_ptr->r[8] = in.r[8];

    return camera_info_ptr;
}
std::string ApcCamera::qos2str(rmw_qos_history_policy_t qos) {
    if (qos == RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
        return "KEEP_LAST";
    }

    if (qos == RMW_QOS_POLICY_HISTORY_KEEP_ALL) {
        return "KEEP_ALL";
    }

    return "Unknown QoS value";
}

std::string ApcCamera::qos2str(rmw_qos_reliability_policy_t qos) {
    if (qos == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
        return "RELIABLE";
    }

    if (qos == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT) {
        return "BEST_EFFORT";
    }

    return "Unknown QoS value";
}

std::string ApcCamera::qos2str(rmw_qos_durability_policy_t qos) {
    if (qos == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
        return "TRANSIENT_LOCAL";
    }

    if (qos == RMW_QOS_POLICY_DURABILITY_VOLATILE) {
        return "VOLATILE";
    }

    return "Unknown QoS value";
}

void ApcCamera::print_qos(const rclcpp::QoS & qos)
{
  const auto & rmw_qos = qos.get_rmw_qos_profile();
  std::cout << "HISTORY POLICY: ";
  switch (rmw_qos.history) {
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      std::cout << "keep last";
      break;
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      std::cout << "keep all";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << " (depth: " << rmw_qos.depth << ')' << std::endl;

  std::cout << "RELIABILITY POLICY: ";
  switch (rmw_qos.reliability) {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      std::cout << "reliable";
      break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      std::cout << "best effort";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << std::endl;

  std::cout << "DURABILITY POLICY: ";
  switch (rmw_qos.durability) {
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      std::cout << "transient local";
      break;
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      std::cout << "volatile";
      break;
    default:
      std::cout << "invalid";
  }
  std::cout << std::endl;
}

}  // namespace dmpreview

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(dmpreview::ApcCamera)
