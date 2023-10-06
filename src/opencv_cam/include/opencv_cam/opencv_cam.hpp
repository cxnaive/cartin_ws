#ifndef OPENCV_CAM_HPP
#define OPENCV_CAM_HPP
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
namespace opencv_cam {

struct CamParams {
    std::string device_name;
    std::string camera_name;
    std::string camera_info_url;
    std::string codec;
    int image_width;
    int image_height;
    double fps;
    bool auto_white_balance;
    bool auto_exposure;
    bool use_sensor_data_qos;
    int white_balance;
    int exposure;
    int brightness;
    int contrast;
    int saturation;
    int hue;
    int gain;
    int gamma;
    int sharpness;
    int backlight;
    bool show_img;
};


class OpencvCameraNode : public rclcpp::Node {
   public:
    explicit OpencvCameraNode(const rclcpp::NodeOptions& options);
    ~OpencvCameraNode() override;

   private:
    std::thread capture_thread;
    std::thread monitor_thread;
    CamParams params;
    // handle
    std::shared_ptr<cv::VideoCapture> handle;
    // 相机图像发布
    image_transport::Publisher image_pub;
    // 相机信息发布
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;

    std::atomic_bool grab_on;
    void load_params();
    void init_camera();
    void start_grab();
    void stop_grab();
    void grab();
};

}  // namespace opencv_cam

#endif