#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdlib.h>

typedef struct 
{
    int rows = 480;
    int cols = 640;
} picture;

void publishStaticTransforms(tf2_ros::StaticTransformBroadcaster& static_broadcaster) {
    geometry_msgs::TransformStamped transformStamped;

    // Transform for RGB image frame
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_link";
    transformStamped.child_frame_id = "camera_color_optical_frame";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    static_broadcaster.sendTransform(transformStamped);

    // Transform for Depth image frame
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_link";
    transformStamped.child_frame_id = "camera_depth_optical_frame";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    static_broadcaster.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bimu_rgbd_camera");
    ros::NodeHandle n;
    ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("camera/color/image_raw", 1000);
    ros::Publisher depth_pub = n.advertise<sensor_msgs::Image>("camera/depth/image_raw", 1000);

    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    publishStaticTransforms(static_broadcaster);

    ros::Rate loop_rate(10);
    int count = 0;

    sensor_msgs::Image ros_image;
    sensor_msgs::Image depth_image;
    unsigned char *RGBbuffer = NULL;
    unsigned short *DEPTHbuffer = NULL;
    picture picture_data;
    picture depth_data;
    ROS_INFO("%d, %d", picture_data.rows, picture_data.cols);

    RGBbuffer = (unsigned char *)malloc(picture_data.rows * picture_data.cols * 3 * sizeof(unsigned char));
    for(int v = 0; v < picture_data.rows; v++)
        for(int u = 0; u < picture_data.cols; u++){
            RGBbuffer[3 * (v * picture_data.cols + u)] = v % 255;
            RGBbuffer[3 * (v * picture_data.cols + u) + 1] = u % 255;
            RGBbuffer[3 * (v * picture_data.cols + u) + 2] = 0;
        }

    DEPTHbuffer = (unsigned short *)malloc(depth_data.rows * depth_data.cols * sizeof(unsigned short));
    for(int v = 0; v < depth_data.rows; v++)
        for(int u = 0; u < depth_data.cols; u++){
            DEPTHbuffer[v * depth_data.cols + u] = v;
        }

    while (ros::ok())
    {
        ros_image.header.stamp = ros::Time::now();
        ros_image.header.frame_id = "camera_color_optical_frame"; // Set the camera frame
        ros_image.height = picture_data.rows;
        ros_image.width = picture_data.cols;
        ros_image.encoding = "rgb8";
        ros_image.is_bigendian = false;
        ros_image.step = picture_data.cols * 3;
        size_t size = ros_image.step * picture_data.rows;
        ros_image.data.resize(size);

        memcpy((char*)(&ros_image.data[0]), RGBbuffer, size);

        depth_image.header.stamp = ros::Time::now();
        depth_image.header.frame_id = "camera_depth_optical_frame"; // Set the depth camera frame
        depth_image.height = depth_data.rows;
        depth_image.width = depth_data.cols;
        depth_image.encoding = "mono16";
        depth_image.is_bigendian = false;
        depth_image.step = depth_data.cols * sizeof(unsigned short);
        size_t depth_size = depth_image.step * depth_data.rows;
        depth_image.data.resize(depth_size);

        memcpy((char*)(&depth_image.data[0]), DEPTHbuffer, depth_size);

        img_pub.publish(ros_image);
        depth_pub.publish(depth_image);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    free(RGBbuffer);
    free(DEPTHbuffer);

    return 0;
}
