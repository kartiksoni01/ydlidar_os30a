#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/CameraInfo.h>

void chatterCallback(const sensor_msgs::CameraInfo& msg)
{
    ROS_INFO("I height: [%d]", msg.height);
    ROS_INFO("I width: [%d]", msg.width);
    for(int i=0; i<9; i++)
        ROS_INFO("I K: [%f]", msg.K[i]);
}

// void img_chatterCallback(const )
// {

// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bimu_rgbd_receive");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera1/camera_info", 1000, chatterCallback);

    // ros::Subscriber img_sub = n.subscribe("bimu_rgbd_camera", 1000, img_chatterCallback);


    ros::spin();

    return 0;
}
