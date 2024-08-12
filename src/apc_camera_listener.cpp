/*Copyright:
    This file copyright (C) 2017 by eYs3D company
    An unpublished work.  All rights reserved.
    This file is proprietary information, and may not be disclosed or
    copied without the prior permission of eYs3D company.
*/
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

struct wm_cv {
    const char *id;
    int width;
    int height;
    int x;
    int y;
};
wm_cv color = {"color", 640, 360, 0, 0};
wm_cv depth = {"depth", 640, 360, 720, 0};

class DMPreviewSubscriber : public rclcpp::Node
{
  public:
  DMPreviewSubscriber()
  : Node("dmpreview_subscriber")
  {
    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    video_qos.best_effort();
    //video_qos.reliable();
    video_qos.durability_volatile();

    RCLCPP_INFO(this->get_logger(),"listen ...");

    auto imageColorCallback =
      [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
      {
        cv_bridge::CvImagePtr cv_ptr;

        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

        cv::imshow(color.id, cv_ptr->image);
        cv::waitKey(3);
      };

    auto imageDepthCallback =
      [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
      {
        cv_bridge::CvImagePtr cv_ptr;

        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

        cv::imshow(depth.id, cv_ptr->image);
        cv::waitKey(3);
      };

    // Create color image subscriber
    color_sub = this->create_subscription<sensor_msgs::msg::Image> (
        "/apc/left/image_color", video_qos, imageColorCallback);

    // Create depth image subscriber
    depth_sub = this->create_subscription<sensor_msgs::msg::Image> (
        "/apc/depth/image_raw", video_qos, imageDepthCallback);
  }

  private:

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
};

int main(int argc, char * argv[])
{
    cv::namedWindow(color.id, cv::WINDOW_NORMAL);
    cv::resizeWindow(color.id, color.width, color.height);
    cv::moveWindow(color.id, color.x, color.y);

    cv::namedWindow(depth.id, cv::WINDOW_NORMAL);
    cv::resizeWindow(depth.id, depth.width, depth.height);
    cv::moveWindow(depth.id, depth.x, depth.y);

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DMPreviewSubscriber>());
    rclcpp::shutdown();
    return 0;
}
