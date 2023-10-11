#ifndef RADAR_NODE_HPP
#define RADAR_NODE_HPP
// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
// opencv
#include <opencv2/opencv.hpp>

// Radar SDK
#include <radar_system.h>
#include <atomic>

namespace radar_ros2 {

class RadarNode : public rclcpp::Node {
   public:
    explicit RadarNode(const rclcpp::NodeOptions& options);
    ~RadarNode() override;
   private:
    std::thread pub_thread;
    std::thread pub_enhanced_thread;
    // params
    std::string config_path;
    std::string radar_name;
    double fps;
    double power_threshold;
    double distance_threshold;
    bool use_sensor_data_qos;
    oculii::ModeCommand radar_mode;

    // pubs
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub, pcl_enhanced_pub;

    // handle
    std::shared_ptr<oculii::RadarSystem> handle;
    bool init_success;

    std::atomic_bool pub_on;
    void pub_radar_enhanced();
    void pub_radar();
};

}  // namespace radar_ros2

#endif