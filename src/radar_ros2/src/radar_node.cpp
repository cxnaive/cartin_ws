#include <assert.h>

#include <radar_ros2/radar_node.hpp>
using namespace radar_ros2;

RadarNode::RadarNode(const rclcpp::NodeOptions &options) : Node("radar_ros2", options) {
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
        pcl_pub = create_publisher<sensor_msgs::msg::PointCloud>("/" + radar_name,
                                                                 rclcpp::SensorDataQoS());
    } else {
        pcl_pub = create_publisher<sensor_msgs::msg::PointCloud>("/" + radar_name, 100);
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

    pub_on = true;
    pub_thread = std::thread(&RadarNode::pub_radar_data, this);
}

RadarNode::~RadarNode() {
    pub_on = false;
    if (pub_thread.joinable()) {
        pub_thread.join();
    }

    if (handle) {
        handle->PauseRadarReceive(std::vector<int>({1}));
        handle->Close();
    }
}

void RadarNode::pub_radar_data() {
    /* Declare Containers for Point Cloud */
    std::map<int, oculii::RadarDetectionPacket> pcl;

    /* Declare Containers for tracker */
    std::map<int, oculii::RadarTrackerPacket> trk;

    /* Set Publish Rate */
    rclcpp::WallRate loop_rate(fps);

    while (rclcpp::ok() && pub_on) {
        /* Get PCL data */
        oculii::RadarErrorCode pclStatus = handle->GetPointcloud(pcl);

        /*Publish detection data for each sensor respectively*/
        if (pclStatus == oculii::RadarErrorCode::SUCCESS) {
            sensor_msgs::msg::PointCloud frameDetection;
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

                for (int i = 0; i < (int)it->second.data.size(); ++i) {
                    if (it->second.data[i].power < power_threshold) continue;
                    coord.x = it->second.data[i].z;
                    coord.y = -it->second.data[i].x;
                    coord.z = -it->second.data[i].y;
                    double distance = sqrt(coord.x * coord.x + coord.y * coord.y + coord.z * coord.z);
                    if(distance > distance_threshold) continue;
                    frameDetection.points.push_back(coord);

                    doppler.values.push_back(it->second.data[i].doppler);
                    range.values.push_back(it->second.data[i].range);
                    power.values.push_back(it->second.data[i].power);
                    alpha.values.push_back(it->second.data[i].alpha);
                    beta.values.push_back(it->second.data[i].beta);
                    // std::cout << it->second.data[i].power << std::endl;
                }
                frameDetection.channels.push_back(doppler);
                frameDetection.channels.push_back(range);
                frameDetection.channels.push_back(power);
                frameDetection.channels.push_back(alpha);
                frameDetection.channels.push_back(beta);
            }
            pcl_pub->publish(std::move(frameDetection));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        loop_rate.sleep();
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_ros2::RadarNode)