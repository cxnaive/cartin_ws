#ifndef THERMAL_IMGPROC_HPP
#define THERMAL_IMGPROC_HPP
#include <atomic>
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
//opencv
#include <opencv2/opencv.hpp>
#define UNDEFINEED (0x0FFFFFFF)
namespace thermal_imgproc {

struct Params {
    std::string image_topic;
    std::string camera_name;
    std::string camera_info_url;
    int image_width_origin;
    int image_height_origin;
    bool show_img;
    bool use_sensor_data_qos;
};


class ThermalImageProcNode : public rclcpp::Node {
   public:
    explicit ThermalImageProcNode(const rclcpp::NodeOptions& options);

   private:
    Params params;
    // 相机图像接受
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    // 相机图像发布
    image_transport::Publisher image_pub;
    // 相机信息发布
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;

    void proc(const sensor_msgs::msg::Image::SharedPtr img_msg);
    void load_params();
};

}  // namespace opencv_cam

#endif