#ifndef THERMAL_IMGPROC_HPP
#define THERMAL_IMGPROC_HPP
#include <atomic>
// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// opencv
#include <opencv2/opencv.hpp>
namespace pcd_proj {

struct Params {
    std::string camera_name;
    std::string pcd_topic;
    std::string pub_topic;
    std::string camera_info_url;
    double max_color_distance;
    bool show_img;
    bool use_sensor_data_qos;
};

class PointcloudProjectionNode : public rclcpp::Node {
   public:
    explicit PointcloudProjectionNode(const rclcpp::NodeOptions& options);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage,
                                                            sensor_msgs::msg::PointCloud2>
        approximate_policy;

   private:
    Params params;
    // 图像/点云接收
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> image_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pcd_sub;
    std::shared_ptr<message_filters::Synchronizer<approximate_policy>> sync_sub;
    // 投影图像发布
    image_transport::Publisher image_pub;
    // camera_info
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;
    sensor_msgs::msg::CameraInfo camera_info_msg;
    // 相机内参矩阵
    Eigen::Matrix3d K;
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener;

    void proc(const sensor_msgs::msg::CompressedImage::SharedPtr img_msg,
              const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg);
    void load_params();
};

}  // namespace pcd_proj

#endif