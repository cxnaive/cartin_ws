#include <assert.h>

#include <radar_ros2/radar_node.hpp>
using namespace radar_ros2;

RadarNode::RadarNode(const rclcpp::NodeOptions& options) : Node("radar_ros2", options) {
    RCLCPP_INFO(this->get_logger(), "Starting RadarNode!");

    config_path = declare_parameter("config_path", "path_to_xml");
    use_sensor_data_qos = declare_parameter("use_sensor_data_qos", true);
    radar_name = declare_parameter("radar_name", "test_radar");
    fps = declare_parameter("fps", 10.0);
    power_threshold = declare_parameter("power_threshold", 10.0);
    distance_threshold = declare_parameter("distance_threshold", 5.0);
    int modeint = declare_parameter("mode", 0);

    switch (modeint) {
        case 0:
            radar_mode = oculii::ModeCommand::SENSOR_MODE_0;
            break;
        case 1:
            radar_mode = oculii::ModeCommand::SENSOR_MODE_1;
            break;
        case 2:
            radar_mode = oculii::ModeCommand::SENSOR_MODE_2;
            break;
        default:
            assert(false && "Invalid mode for Radar!");
            break;
    }

    RCLCPP_INFO(get_logger(), "Config Path: %s", config_path.c_str());
    RCLCPP_INFO(get_logger(), "Radar Name: %s", radar_name.c_str());
    RCLCPP_INFO(get_logger(), "FPS: %lf", fps);
    RCLCPP_INFO(get_logger(), "power_threshold: %lf", power_threshold);
    RCLCPP_INFO(get_logger(), "distance_threshold: %lf", distance_threshold);
    RCLCPP_INFO(get_logger(), "Radar_mode: %d", modeint);

    bool use_intra = options.use_intra_process_comms();
    if (!use_intra) {
        RCLCPP_WARN(get_logger(), "Not In Intra Process Mode");
    }
    if (!use_sensor_data_qos) {
        RCLCPP_WARN(get_logger(), "Not Use Sensor Data Qos");
    }
    // create pub
    if (use_sensor_data_qos) {
        pcl_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/" + radar_name,
                                                                  rclcpp::SensorDataQoS());
        pcl_enhanced_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/" + radar_name + "/enhanced", rclcpp::SensorDataQoS());
    } else {
        pcl_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/" + radar_name, 100);
        pcl_enhanced_pub =
            create_publisher<sensor_msgs::msg::PointCloud2>("/" + radar_name + "/enhanced", 100);
    }

    // create handle
    handle = std::shared_ptr<oculii::RadarSystem>(
        oculii::RadarSystem::GetRadarSystemInstance(config_path));
    init_success = true;

    // start radar system
    oculii::RadarErrorCode status = handle->StartSystem();
    if (status == oculii::RadarErrorCode::SUCCESS)
        RCLCPP_INFO(get_logger(), "Start Radar System success! ");
    else {
        RCLCPP_ERROR(get_logger(), "Start Radar System fail with error: %s",
                     oculii::ErrorToString(status).c_str());
        return;
    }

    std::vector<int> IDs{1};
    std::vector<oculii::ModeCommand> modes{radar_mode};
    handle->SendModeSwitchCmd(IDs, modes);

    status = handle->StartRadarReceive();
    if (status == oculii::RadarErrorCode::SUCCESS)
        RCLCPP_INFO(get_logger(), "Start Radar Receive success! ");
    else {
        RCLCPP_ERROR(get_logger(), "Start Radar Receive fail with error: %s",
                     oculii::ErrorToString(status).c_str());
        return;
    }

    // get frame rate
    int radar_frame_rate = handle->GetFrameRate(1);
    RCLCPP_INFO(get_logger(), "Radar Device frame rate %d!", radar_frame_rate);

    pub_on = true;
    pub_thread = std::thread(&RadarNode::pub_radar, this);
    pub_enhanced_thread = std::thread(&RadarNode::pub_radar_enhanced, this);
}

RadarNode::~RadarNode() {
    pub_on = false;
    if (pub_thread.joinable()) {
        pub_thread.join();
    }
    if (pub_enhanced_thread.joinable()){
        pub_enhanced_thread.join();
    }

    if (handle) {
        handle->PauseRadarReceive(std::vector<int>({1}));
        handle->Close();
    }
}

void RadarNode::pub_radar() {
    /* Declare Containers for Point Cloud */
    std::map<int, oculii::RadarDetectionPacket> pcl;

    /* Set Publish Rate */
    rclcpp::WallRate loop_rate(fps);

    while (rclcpp::ok() && pub_on) {
        /* Get PCL data */
        oculii::RadarErrorCode pclStatus = handle->GetPointcloud(pcl);

        /*Publish detection data for each sensor respectively*/
        if (pclStatus == oculii::RadarErrorCode::SUCCESS) {
            sensor_msgs::msg::PointCloud frameDetection;
            sensor_msgs::msg::PointCloud2 frameDetection2;
            frameDetection.header.frame_id = radar_name;
            frameDetection.header.stamp = now();

            for (auto it = pcl.begin(); it != pcl.end(); ++it) {
                geometry_msgs::msg::Point32 coord;
                sensor_msgs::msg::ChannelFloat32 doppler, range, power, alpha, beta;
                doppler.name = "Doppler";
                range.name = "Range";
                power.name = "Power";
                alpha.name = "Alpha";
                beta.name = "Beta";

                auto packet = it->second;
                for (int i = 0; i < (int)packet.data.size(); ++i) {
                    if (packet.data[i].power >= power_threshold) {
                        coord.x = packet.data[i].z;
                        coord.y = -packet.data[i].x;
                        coord.z = -packet.data[i].y;
                        double distance =
                            sqrt(coord.x * coord.x + coord.y * coord.y + coord.z * coord.z);
                        if (distance > distance_threshold) continue;
                        frameDetection.points.push_back(coord);

                        doppler.values.push_back(packet.data[i].doppler);
                        range.values.push_back(packet.data[i].range);
                        power.values.push_back(packet.data[i].power);
                        alpha.values.push_back(packet.data[i].alpha);
                        beta.values.push_back(packet.data[i].beta);
                    }
                }

                frameDetection.channels.push_back(doppler);
                frameDetection.channels.push_back(range);
                frameDetection.channels.push_back(power);
                frameDetection.channels.push_back(alpha);
                frameDetection.channels.push_back(beta);
            }
            sensor_msgs::convertPointCloudToPointCloud2(frameDetection, frameDetection2);
            frameDetection2.header.stamp = frameDetection.header.stamp;
            frameDetection2.header.frame_id = frameDetection.header.frame_id;
            pcl_pub->publish(std::move(frameDetection2));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            // RCLCPP_ERROR(get_logger(),"fuck~!!");
            continue;
        }
        loop_rate.sleep();
    }
}

void RadarNode::pub_radar_enhanced() {
    /* Declare Containers for Point Cloud */
    std::map<int, std::vector<oculii::RadarDetectionPacket> > pcl;

    /* Set Publish Rate */
    rclcpp::WallRate loop_rate(fps);

    while (rclcpp::ok() && pub_on) {
        /* Get PCL data */
        oculii::RadarErrorCode pclStatus = handle->GetEnhancedPointcloud(pcl);

        /*Publish detection data for each sensor respectively*/
        if (pclStatus == oculii::RadarErrorCode::SUCCESS) {
            sensor_msgs::msg::PointCloud frameDetection;
            sensor_msgs::msg::PointCloud2 frameDetection2;
            frameDetection.header.frame_id = radar_name;
            frameDetection.header.stamp = now();

            for (auto it = pcl.begin(); it != pcl.end(); ++it) {
                geometry_msgs::msg::Point32 coord;
                sensor_msgs::msg::ChannelFloat32 doppler, range, power, alpha, beta;
                doppler.name = "Doppler";
                range.name = "Range";
                power.name = "Power";
                alpha.name = "Alpha";
                beta.name = "Beta";

                for (auto& packet : it->second) {
                    for (int i = 0; i < (int)packet.data.size(); ++i) {
                        if (packet.data[i].denoiseFlag != 0 &&
                            packet.data[i].power >= power_threshold) {
                            coord.x = packet.data[i].z;
                            coord.y = -packet.data[i].x;
                            coord.z = -packet.data[i].y;
                            double distance =
                                sqrt(coord.x * coord.x + coord.y * coord.y + coord.z * coord.z);
                            if (distance > distance_threshold) continue;
                            frameDetection.points.push_back(coord);

                            doppler.values.push_back(packet.data[i].doppler);
                            range.values.push_back(packet.data[i].range);
                            power.values.push_back(packet.data[i].power);
                            alpha.values.push_back(packet.data[i].alpha);
                            beta.values.push_back(packet.data[i].beta);
                        }
                    }
                }
                frameDetection.channels.push_back(doppler);
                frameDetection.channels.push_back(range);
                frameDetection.channels.push_back(power);
                frameDetection.channels.push_back(alpha);
                frameDetection.channels.push_back(beta);
            }
            sensor_msgs::convertPointCloudToPointCloud2(frameDetection, frameDetection2);
            frameDetection2.header.stamp = frameDetection.header.stamp;
            frameDetection2.header.frame_id = frameDetection.header.frame_id;
            pcl_enhanced_pub->publish(std::move(frameDetection2));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        loop_rate.sleep();
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_ros2::RadarNode)