#include <assert.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <tbb/tbb.h>

#include <cstdio>
#include <cstdlib>
#include <functional>
#include <pcl/common/transforms.h>
#include <pcl_utils_tbb.hpp>
#include <pcl_nanoflann.hpp>
#include <perf.hpp>
#include <prior_map/prior_map.hpp>
using namespace prior_map;

RelocalizationPriorMapNode::RelocalizationPriorMapNode(const rclcpp::NodeOptions& options)
    : Node("prior_map", options) {
    RCLCPP_INFO(this->get_logger(), "Starting RelocalizationPriorMapNode!");
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
    auto qos_profile =
        params.use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);
    lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        params.lidar_topic, qos,
        std::bind(&RelocalizationPriorMapNode::proc, this, std::placeholders::_1));

    priormap_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(params.priormap_topic, qos);
    lidar_extract_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(params.lidar_extract_topic, qos);
    // load pcd
    global_map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile(params.global_map_path, *global_map)) {
        RCLCPP_ERROR(get_logger(), "Error loading global map!");
        map_ok = false;
    } else {
        map_ok = true;
        RCLCPP_WARN(get_logger(), "Global Map Size: %d", (int)global_map->size());
    }

    // BuildKDTree
    map_adaptor = std::make_shared<PCLAdaptor<pcl::PointXYZ>>(*global_map);
    global_kdtree = std::make_shared<PCLAdaptor<pcl::PointXYZ>::KDTree>(3,*map_adaptor);
    // tf2 relevant
    tf2_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer->setCreateTimerInterface(timer_interface);
    tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer);
}

void RelocalizationPriorMapNode::load_params() {
    params.lidar_topic = this->declare_parameter("lidar_topic", "lidar_topic");
    params.priormap_topic = this->declare_parameter("priormap_topic", "priormap_topic");
    params.map_frame = this->declare_parameter("map_frame", "map_frame");
    params.lidar_extract_topic =
        this->declare_parameter("lidar_extract_topic", "lidar_extract_topic");
    params.prior_distance = this->declare_parameter("prior_distance", 20.0);
    params.distance_threshold = this->declare_parameter("distance_threshold", 0.2);
    params.num_threads = this->declare_parameter("num_threads", 4);
    params.use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    params.global_map_path = this->declare_parameter("global_map_path", "invalid_path");
    RCLCPP_INFO(get_logger(), "Global Map Path: %s", params.global_map_path.c_str());
    RCLCPP_INFO(get_logger(), "Map Frame: %s", params.map_frame.c_str());
    RCLCPP_INFO(get_logger(), "Num Threads: %d", params.num_threads);
    RCLCPP_INFO(get_logger(), "PriorMap Distance: %0.3lf", params.prior_distance);
    RCLCPP_INFO(get_logger(), "Filter Distance Threshold : %0.3lf", params.distance_threshold);
    RCLCPP_INFO(get_logger(), "Lidar Topic: %s", params.lidar_topic.c_str());
    RCLCPP_INFO(get_logger(), "Lidar Extract Topic: %s", params.lidar_extract_topic.c_str());
    RCLCPP_INFO(get_logger(), "PriorMap Topic: %s", params.priormap_topic.c_str());
}

void RelocalizationPriorMapNode::proc(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_pcd) {
    // 获得 transformation
    geometry_msgs::msg::TransformStamped t;
    Eigen::Matrix4d transform_mat;
    try {
        t = tf2_buffer->lookupTransform(params.map_frame, lidar_pcd->header.frame_id,
                                        lidar_pcd->header.stamp,
                                        rclcpp::Duration::from_seconds(0.5));
        Eigen::Isometry3d trans_eigen = tf2::transformToEigen(t);
        transform_mat = trans_eigen.matrix();
    } catch (const std::exception& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                    lidar_pcd->header.frame_id.c_str(), params.map_frame.c_str(), ex.what());
        return;
    }

    static Perf prior_map_perf("Perf:Build Prior Map", 100);
    Event prior_map_event;
    prior_map_event.start();

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar_cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*lidar_pcd, *lidar_cloud);

    Eigen::Vector4d lidar_pos = transform_mat.block<4, 1>(0, 3);
    Eigen::Vector4d distance_modifier(params.prior_distance, params.prior_distance,
                                      params.prior_distance, params.prior_distance);

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> local_map =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // std
    PointCloudInBox_tbb(params.num_threads, *global_map, lidar_pos - distance_modifier,
                        lidar_pos + distance_modifier, *local_map);

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> local_map_lidar =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(*local_map, *local_map_lidar, transform_mat.inverse().cast<float>());
    //
    sensor_msgs::msg::PointCloud2 local_map_msg;
    pcl::toROSMsg(*local_map_lidar, local_map_msg);
    local_map_msg.header.stamp = lidar_pcd->header.stamp;
    local_map_msg.header.frame_id = lidar_pcd->header.frame_id;
    priormap_pub->publish(std::move(local_map_msg));

    prior_map_event.end();
    prior_map_perf.update(prior_map_event.duration());

    // search KDT
    static Perf search_kdtree_perf("Perf:Search KDTree", 100);
    Event search_kdtree_event;
    search_kdtree_event.start();

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar_cloud_extracted_map =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar_cloud_extracted =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar_cloud_map =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(*lidar_cloud, *lidar_cloud_map, transform_mat.cast<float>());

    DistanceFilter_tbb(params.num_threads, *lidar_cloud_map, *global_kdtree, params.distance_threshold,
                       *lidar_cloud_extracted_map);

    pcl::transformPointCloud(*lidar_cloud_extracted_map, *lidar_cloud_extracted,
                             transform_mat.inverse().cast<float>());

    sensor_msgs::msg::PointCloud2 lidar_extract_msg;
    pcl::toROSMsg(*lidar_cloud_extracted, lidar_extract_msg);
    lidar_extract_msg.header.stamp = lidar_pcd->header.stamp;
    lidar_extract_msg.header.frame_id = lidar_pcd->header.frame_id;
    lidar_extract_pub->publish(std::move(lidar_extract_msg));

    search_kdtree_event.end();
    search_kdtree_perf.update(search_kdtree_event.duration());
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(prior_map::RelocalizationPriorMapNode)
