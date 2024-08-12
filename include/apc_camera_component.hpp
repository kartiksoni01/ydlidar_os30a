#ifndef APC_CAMERA_COMPONENT_HPP
#define APC_CAMERA_COMPONENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rcutils/logging_macros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/publisher.hpp>

#include <stereo_msgs/msg/disparity_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "std_msgs/msg/string.hpp"

#include <string>

#include <cv_bridge/cv_bridge.h>

#include "EYS3DSystem.h"
#include "devices/CameraDevice.h"
#include "video/Frame.h"
#include "video/PCFrame.h"
#include "version_utils.h"
#include "versionInfo.h"
#include "types_calib.h"
#include "devices/IMUDevice.h"

#define TIMEZERO_ROS rclcpp::Time(0,0,RCL_ROS_TIME)
#define TIMEZERO_SYS rclcpp::Time(0,0,RCL_SYSTEM_TIME)

namespace dmpreview {

typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloudPub;
//typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imuPub;
typedef std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> posePub;

typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
typedef std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloudMsgPtr;

//typedef std::unique_ptr<sensor_msgs::msg::Imu> imuMsgPtr;
typedef std::unique_ptr<geometry_msgs::msg::PoseStamped> poseMsgPtr;


class ApcCamera : public rclcpp::Node
{

    public:
        explicit ApcCamera(const rclcpp::NodeOptions & options);

        virtual ~ApcCamera();

    protected:
        rcl_interfaces::msg::SetParametersResult paramChange_callback(std::vector<rclcpp::Parameter> parameters);

        #ifdef _TEST
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
        #endif //_TEST

        typedef struct moduleModeConfig{
            int colorFormat;
            int colorWidth;
            int colorHeight;
            int fps;
            int depthWidth;
            int depthHeight;
            int videoMode;
            bool interLeaveMode;
        } module_mode_config_t;
        module_mode_config_t moduleModeConfig_ = {
            0 ,1280 ,720 ,30 ,1280 ,720 ,4 ,false
        };

        void getModeConfig(int mode);
        void openDevice();
        void closeDevice();
        void setDefaultParams();
        void getLanuchParams();
        void deleteDevice();
        void dynamicChangeMode(int mode);
        void print_qos(const rclcpp::QoS & qos);

        std::shared_ptr<sensor_msgs::msg::Image> ConvertFrameToMessage(cv::Mat & frame, const std::string enc);
        rclcpp::Time frameTime2Ros(uint64_t t, rcl_clock_type_t clock_type = RCL_ROS_TIME);

        StreamMode getStreamModeIndex();
        StreamIntrinsics getCameraIntrinsics(
            const StreamMode& stream_mode, bool* ok);
        camInfoMsgPtr createCameraInfo(const CameraIntrinsics& in);
        StreamIntrinsics getCameraRectifyLog(
            const StreamMode& stream_mode, bool* ok);
        int getRectifyLogIndex(StreamMode stream_mode);

        //+CallBack function
        bool color_image_callback(const libeYs3D::video::Frame *frame);
        bool depth_image_callback(const libeYs3D::video::Frame *frame);
        bool pc_frame_callback(const libeYs3D::video::PCFrame *pcFrame);
        bool imu_data_callback(const libeYs3D::sensors::SensorData *sensorData);
        //-CallBack function

        std::string qos2str(rmw_qos_history_policy_t qos);
        std::string qos2str(rmw_qos_reliability_policy_t qos);
        std::string qos2str(rmw_qos_durability_policy_t qos);

    private:
        //+Publishers
        std::string left_color_topic;
        std::string right_color_topic;
        std::string depth_topic;
        std::string points_topic;
        std::string imu_topic;
        std::string imu_processed_topic;

        image_transport::CameraPublisher pub_left_color;
        image_transport::CameraPublisher pub_right_color;
        image_transport::CameraPublisher pub_depth;
        pointcloudPub pub_points;
        //imuPub pub_imu;
        //imuPub pub_imu_processed;
        posePub pub_pose;
        //-Publishers

        //Qos
        rclcpp::QoS mQos;

        //+Camera infos
        camInfoMsgPtr left_info_ptr;
        camInfoMsgPtr right_info_ptr;
        camInfoMsgPtr depth_info_ptr;
        pointcloudMsgPtr point_msg;
        std::shared_ptr<sensor_msgs::PointCloud2Modifier> point_modifier;
        //-Camera infos

        //+Frame IDs
        std::string left_color_frame_id;
        std::string right_color_frame_id;
        std::string depth_frame_id;
        std::string points_frame_id;
        std::string imu_frame_id;
        std::string imu_frame_processed_id;
        //-Frame IDs

        enum ColorFormat {
            Color_MJPEG = 0,
            Color_YUYV
        };

        enum DepthOutputType {
            Depth_Output_Raw = 0,
            Depth_Output_Gray,
            Depth_Output_Colorful,
        };

        typedef struct DeviceParams {
            bool multi_module_;

            bool enable_color_stream_;
            bool enable_depth_stream_;
            bool enable_point_cloud_stream_;
            bool enable_imu_stream_;

            int auto_config_camera_mode_;
            int framerate_;    

            int color_width_;
            int color_height_;

            int color_stream_format_;

            int depth_width_;
            int depth_height_;
            int depth_data_type_;
            bool interleave_mode_;
            DepthOutputType depth_output_type_;

            int z_maximum_mm_;

            bool auto_exposure_;
            bool auto_white_balance_;

            int exposure_time_step_;
            int white_balance_temperature_;

            bool extend_ir_;
            int ir_intensity_;

            std::string serial_number_;
            std::string kernel_name_;

            rmw_qos_history_policy_t qos_hist_;
            int qos_depth_;
            rmw_qos_reliability_policy_t qos_reliability_;
            rmw_qos_durability_policy_t qos_durability_;

        }device_params_t;
        device_params_t params_;

        typedef struct SubResult {
            bool left_color;
            bool right_color;
            bool depth;
            bool points;
            bool imu;
            bool imu_processed;
        } sub_result_t;
        sub_result_t sub_result;

        std::uint64_t unit_hard_time = 4294900000;

        std::shared_ptr<libeYs3D::EYS3DSystem> eYs3DSystem_;
        std::shared_ptr<libeYs3D::devices::CameraDevice> device_;

        std::unique_ptr<ModeConfigOptions> modeConfigOptions;
        ModeConfig::MODE_CONFIG modeConfig;
        libeYs3D::video::DEPTH_RAW_DATA_TYPE depth_raw_data_type;

        libeYs3D::video::Producer::Callback mColorStreamCallback;
        libeYs3D::video::Producer::Callback mDepthStreamCallback;
        libeYs3D::video::PCProducer::PCCallback mPCStreamCallback;
        libeYs3D::sensors::SensorDataProducer::AppCallback mIMUStreamCallback;

};

} // namespace dmpreview

#endif // APC_CAMERA_COMPONENT_HPP
