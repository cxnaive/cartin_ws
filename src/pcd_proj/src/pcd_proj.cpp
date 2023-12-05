#include <assert.h>

#include <cstdio>
#include <cstdlib>
#include <functional>
#include <pcd_proj/pcd_proj.hpp>
#include <pcd_proj/perf.hpp>
using namespace pcd_proj;

PointcloudProjectionNode::PointcloudProjectionNode(const rclcpp::NodeOptions& options)
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
    // publisher
    auto qos = params.use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    image_pub = image_transport::create_publisher(this, params.pub_topic, qos);

    image_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this,params.image_topic,qos);
    pcd_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this,params.pcd_topic,qos);
    sync_sub = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(10),*image_sub,*pcd_sub);
    sync_sub->registerCallback(&PointcloudProjectionNode::proc, this);
};

void PointcloudProjectionNode::load_params() {
    params.image_topic = this->declare_parameter("image_topic", "image_topic");
    params.pcd_topic = this->declare_parameter("pcd_topic", "pcd_topic");
    params.pub_topic = this->declare_parameter("pub_topic", "pub_topic");
    params.show_img = this->declare_parameter("show_img", false);
    params.use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    RCLCPP_INFO(get_logger(), "Image Topic: %s", params.image_topic.c_str());
    RCLCPP_INFO(get_logger(), "Poindcloud Topic: %s", params.pcd_topic.c_str());
    RCLCPP_INFO(get_logger(), "Publish Topic: %s", params.pub_topic.c_str());
};

void PointcloudProjectionNode::proc(const sensor_msgs::msg::Image::SharedPtr img_msg, const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg){
    // zero-copy mat
    cv::Mat img_origin(img_msg->height, img_msg->width, CV_8UC3, img_msg->data.data());
    // cv::Mat img_roi = img_origin(cv::Rect2i(0,0,params.image_width_origin / 2,params.image_height_origin));
    if(params.show_img){
        cv::imshow("proc_img",img_origin);
        cv::waitKey(1);
    }
    // delay test
    // static Perf delay_perf("delay_perf",1000);
    // double delay_ms = (this->now().nanoseconds() - rclcpp::Time(img_msg->header.stamp).nanoseconds()) / 1000000.0;
    // delay_perf.update(delay_ms);

    // 创建消息
    // sensor_msgs::msg::Image image_msg;
    // image_msg.encoding = "bgr8";
    // image_msg.header.frame_id = params.camera_name;
    // image_msg.header.stamp = img_msg->header.stamp;
    // image_msg.height = img_roi.rows;
    // image_msg.width = img_roi.cols;
    // image_msg.step = img_roi.cols * 3;
    // image_msg.data.resize(img_roi.rows * img_roi.cols * 3);

    // // zero-copy mat
    // cv::Mat img_output(image_msg.height, image_msg.width, CV_8UC3, image_msg.data.data());
    // img_roi.copyTo(img_output);
    // image_pub.publish(std::move(image_msg));
    // if(camera_info_pub){
    //     camera_info_msg.header.stamp = image_msg.header.stamp;
    //     camera_info_pub->publish(camera_info_msg);
    // }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd_proj::PointcloudProjectionNode)