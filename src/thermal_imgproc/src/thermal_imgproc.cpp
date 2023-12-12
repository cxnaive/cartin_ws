#include <assert.h>

#include <cstdio>
#include <cstdlib>
#include <functional>
#include <thermal_imgproc/perf.hpp>
#include <thermal_imgproc/thermal_imgproc.hpp>
using namespace thermal_imgproc;

ThermalImageProcNode::ThermalImageProcNode(const rclcpp::NodeOptions& options)
    : Node("thermal_imgproc", options) {
    RCLCPP_INFO(this->get_logger(), "Starting ThermalImageProcNode!");
    load_params();
    // 创建pub/sub
    bool use_intra = options.use_intra_process_comms();
    if (!use_intra) {
        RCLCPP_WARN(get_logger(), "Not In Intra Process Mode");
    }
    if (!params.use_sensor_data_qos) {
        RCLCPP_WARN(get_logger(), "Not Use Sensor Data Qos");
    }
    auto qos = params.use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    image_pub = image_transport::create_publisher(this, params.camera_name, qos);
    image_sub = create_subscription<sensor_msgs::msg::Image>(
        params.image_topic, 5, std::bind(&ThermalImageProcNode::proc, this, std::placeholders::_1));
    // load camera info
    camera_info_manager =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, params.camera_name);
    if (camera_info_manager->validateURL(params.camera_info_url) &&
        camera_info_manager->loadCameraInfo(params.camera_info_url)) {
        camera_info_msg = camera_info_manager->getCameraInfo();
        if (params.use_sensor_data_qos) {
            camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(
                "/" + params.camera_name + "/camera_info", rclcpp::SensorDataQoS());
        } else {
            camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(
                "/" + params.camera_name + "/camera_info", 100);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                    params.camera_info_url.c_str());
    }
}

void ThermalImageProcNode::load_params() {
    params.camera_name = this->declare_parameter("camera_name", "camera_raw");
    params.image_topic = this->declare_parameter("image_topic", "image_test");
    params.camera_info_url = this->declare_parameter(
        "camera_info_url", "package://thermal_imgproc/config/camera_info.yaml");
    params.show_img = this->declare_parameter("show_img", false);
    params.image_width_origin = this->declare_parameter("image_width_origin", 110);
    params.image_height_origin = this->declare_parameter("image_height_origin", 222);
    RCLCPP_INFO(get_logger(), "Camera Name: %s", params.camera_name.c_str());
    RCLCPP_INFO(get_logger(), "Image Topic: %s", params.image_topic.c_str());
    RCLCPP_INFO(get_logger(), "image_width_origin: %d", params.image_width_origin);
    RCLCPP_INFO(get_logger(), "image_height_origin: %d", params.image_height_origin);
}

void ThermalImageProcNode::proc(const sensor_msgs::msg::Image::SharedPtr img_msg){
    // zero-copy mat
    cv::Mat img_origin(img_msg->height, img_msg->width, CV_8UC3, img_msg->data.data());
    cv::Mat img_roi = img_origin(cv::Rect2i(0,0,params.image_width_origin / 2,params.image_height_origin));
    if(params.show_img){
        cv::imshow("proc_img",img_roi);
        cv::waitKey(1);
    }
    // delay test
    // static Perf delay_perf("delay_perf",1000);
    // double delay_ms = (this->now().nanoseconds() - rclcpp::Time(img_msg->header.stamp).nanoseconds()) / 1000000.0;
    // delay_perf.update(delay_ms);

    // 创建消息
    sensor_msgs::msg::Image image_msg;
    image_msg.encoding = "bgr8";
    image_msg.header.frame_id = params.camera_name;
    image_msg.header.stamp = img_msg->header.stamp;
    image_msg.height = img_roi.rows;
    image_msg.width = img_roi.cols;
    image_msg.step = img_roi.cols * 3;
    image_msg.data.resize(img_roi.rows * img_roi.cols * 3);

    // zero-copy mat
    cv::Mat img_output(image_msg.height, image_msg.width, CV_8UC3, image_msg.data.data());
    img_roi.copyTo(img_output);
    image_pub.publish(std::move(image_msg));
    if(camera_info_pub){
        camera_info_msg.header.stamp = image_msg.header.stamp;
        camera_info_pub->publish(camera_info_msg);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(thermal_imgproc::ThermalImageProcNode)