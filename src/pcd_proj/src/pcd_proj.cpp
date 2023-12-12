#include <assert.h>

#include <cstdio>
#include <cstdlib>
#include <functional>
#include <pcd_proj/pcd_proj.hpp>
#include <pcd_proj/perf.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace pcd_proj;

PointcloudProjectionNode::PointcloudProjectionNode(const rclcpp::NodeOptions& options)
    : Node("pcd_proj", options) {
    RCLCPP_INFO(this->get_logger(), "Starting PointcloudProjectionNode!");
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
    // image_transport::ImageTransport it(rclcpp::Node::SharedPtr(this));
    // image_pub = it.advertise(params.pub_topic, 10);
    image_pub = image_transport::create_publisher(this, params.pub_topic, qos);

    image_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(
        this, params.camera_name+"/compressed", qos);
    pcd_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, params.pcd_topic, qos);
    sync_sub = std::make_shared<message_filters::Synchronizer<approximate_policy>>(
        approximate_policy(10), *image_sub, *pcd_sub);
    sync_sub->registerCallback(&PointcloudProjectionNode::proc, this);

    // load camera info
    camera_info_manager =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, params.camera_name);
    if (camera_info_manager->validateURL(params.camera_info_url) &&
        camera_info_manager->loadCameraInfo(params.camera_info_url)) {
        camera_info_msg = camera_info_manager->getCameraInfo();
        K = Eigen::Matrix3d(camera_info_msg.k.data()).transpose();
        // std::cout << K <<std::endl;
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                    params.camera_info_url.c_str());
    }

    // tf2 relevant
    tf2_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer->setCreateTimerInterface(timer_interface);
    tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer);
};

void PointcloudProjectionNode::load_params() {
    params.camera_name = this->declare_parameter("camera_name", "camera_name");
    params.pcd_topic = this->declare_parameter("pcd_topic", "pcd_topic");
    params.pub_topic = this->declare_parameter("pub_topic", "pub_topic");
    params.max_color_distance = this->declare_parameter("max_color_distance", 50.0);
    params.show_img = this->declare_parameter("show_img", false);
    params.use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    params.camera_info_url = this->declare_parameter("camera_info_url", "invalid_url");
    RCLCPP_INFO(get_logger(), "Camera Name: %s", params.camera_name.c_str());
    RCLCPP_INFO(get_logger(), "Poindcloud Topic: %s", params.pcd_topic.c_str());
    RCLCPP_INFO(get_logger(), "Publish Topic: %s", params.pub_topic.c_str());
};

typedef struct {
    double r, g, b;
} COLOUR;

COLOUR GetColour(double v, double vmin, double vmax) {
    COLOUR c = {1.0, 1.0, 1.0};  // white
    double dv;
    if (v < vmin) v = vmin;
    if (v > vmax) v = vmax;
    dv = vmax - vmin;
    if (v < (vmin + 0.25 * dv)) {
        c.r = 0;
        c.g = 4 * (v - vmin) / dv;
    } else if (v < (vmin + 0.5 * dv)) {
        c.r = 0;
        c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
    } else if (v < (vmin + 0.75 * dv)) {
        c.r = 4 * (v - vmin - 0.5 * dv) / dv;
        c.b = 0;
    } else {
        c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
        c.b = 0;
    }
    return (c);
}

void PointcloudProjectionNode::proc(const sensor_msgs::msg::CompressedImage::SharedPtr img_msg,
                                    const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg) {
    // zero-copy mat
    // cv::Mat img_origin(img_msg->height, img_msg->width, CV_8UC3, img_msg->data.data());

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
    cv::Mat img_origin = cv_ptr->image;
    // cv::Mat img_roi = img_origin(cv::Rect2i(0,0,params.image_width_origin /
    // 2,params.image_height_origin));

    // 获得 transformation
    geometry_msgs::msg::TransformStamped t;
    Eigen::Matrix4d transform_mat;
    try {
        t = tf2_buffer->lookupTransform(img_msg->header.frame_id, pcd_msg->header.frame_id,
                                        img_msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
        Eigen::Isometry3d trans_eigen = tf2::transformToEigen(t);
        // std::cout << trans_eigen.matrix() << std::endl;
        transform_mat = trans_eigen.matrix();
    } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                    pcd_msg->header.frame_id.c_str(), img_msg->header.frame_id.c_str(), ex.what());
        return;
    }
    //
    // pcl::PointCloud<pcl::PointXYZ>::Ptr plain_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // std::string filter_channel = "intensity";
    // int filter_field_offset = -1;
    // for(int i = 0;i < (int)pcd_msg->fields.size();++i){
    //     auto & fieldnow = pcd_msg->fields[i];

    // }
    for (int i = 0; i < (int)(pcd_msg->width * pcd_msg->height); ++i) {
        // xyz
        float x, y, z;
        memcpy(&x, &pcd_msg->data[pcd_msg->point_step * i], 4);
        memcpy(&y, &pcd_msg->data[pcd_msg->point_step * i + 4], 4);
        memcpy(&z, &pcd_msg->data[pcd_msg->point_step * i + 8], 4);
        // std::cout << "1:" << x << " " << y << " " << z << std::endl;
        Eigen::Vector4d p(x, y, z, 1);
        // transform
        Eigen::Vector4d p_camera = transform_mat * p;
        Eigen::Vector3d p_cam_3 = Eigen::Vector3d(p_camera[0], p_camera[1], p_camera[2]);
        // 在相机背后
        if(p_cam_3[2] < 0) continue;
        Eigen::Vector3d p_img = K * p_cam_3 / p_cam_3[2];
        // std::cout << p_img << std::endl;
        // draw
        if (p_img[0] < 0 || p_img[0] > img_origin.cols) continue;
        if (p_img[1] < 0 || p_img[1] > img_origin.rows) continue;

        COLOUR p_col = GetColour(p_cam_3.norm(), 0, params.max_color_distance);
        cv::circle(img_origin, cv::Point2d(p_img[0], p_img[1]), 1.0,
                   cv::Scalar(p_col.b, p_col.g, p_col.r) * 255.0, 1.5, 8);
    }

    if (params.show_img) {
        cv::imshow("proc_img", img_origin);
        cv::waitKey(1);
    }
    // pcl::fromROSMsg()
    // delay test
    // static Perf delay_perf("delay_perf",1000);
    // double delay_ms = (this->now().nanoseconds() -
    // rclcpp::Time(img_msg->header.stamp).nanoseconds()) / 1000000.0; delay_perf.update(delay_ms);

    // 创建消息
    sensor_msgs::msg::Image image_msg;
    image_msg.encoding = "bgr8";
    image_msg.header.frame_id = params.camera_name;
    image_msg.header.stamp = img_msg->header.stamp;
    image_msg.height = img_origin.rows;
    image_msg.width = img_origin.cols;
    image_msg.step = img_origin.cols * 3;
    image_msg.data.resize(img_origin.rows * img_origin.cols * 3);

    // zero-copy mat
    cv::Mat img_output(image_msg.height, image_msg.width, CV_8UC3, image_msg.data.data());
    img_origin.copyTo(img_output);
    image_pub.publish(std::move(image_msg));
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd_proj::PointcloudProjectionNode)