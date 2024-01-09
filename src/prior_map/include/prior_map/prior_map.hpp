#ifndef PRIOR_MAP_HPP
#define PRIOR_MAP_HPP
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
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.hpp>
// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// nanoflann
#include <pcl_nanoflann.hpp>
// opencv
#include <opencv2/opencv.hpp>
namespace prior_map {

struct Params {
    std::string lidar_topic;
    std::string lidar_extract_topic;
    std::string priormap_topic;
    std::string map_frame;
    std::string global_map_path;
    int num_threads;
    double prior_distance;
    double distance_threshold;
    bool use_sensor_data_qos;
};

class RelocalizationPriorMapNode : public rclcpp::Node {
   public:
    explicit RelocalizationPriorMapNode(const rclcpp::NodeOptions& options);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage,
                                                            sensor_msgs::msg::PointCloud2>
        approximate_policy;

   private:
    Params params;
    // 点云接收
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub;
    // 点云发布
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr priormap_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_extract_pub;
    // 点云地图
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> global_map;
    std::shared_ptr<PCLAdaptor<pcl::PointXYZ>> map_adaptor;
    std::shared_ptr<PCLAdaptor<pcl::PointXYZ>::KDTree> global_kdtree;

    bool map_ok;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener;

    void proc(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_pcd);
    void load_params();
};

}  // namespace prior_map

#endif