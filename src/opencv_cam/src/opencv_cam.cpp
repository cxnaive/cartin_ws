#include <assert.h>

#include <cstdio>
#include <cstdlib>
#include <opencv_cam/opencv_cam.hpp>
#include <opencv_cam/perf.hpp>
using namespace opencv_cam;

OpencvCameraNode::OpencvCameraNode(const rclcpp::NodeOptions& options)
    : Node("opencv_cam", options) {
    RCLCPP_INFO(this->get_logger(), "Starting OpenCVCameraNode!");
    load_params();
    // 创建pub
    bool use_intra = options.use_intra_process_comms();
    if (!use_intra) {
        RCLCPP_WARN(get_logger(), "Not In Intra Process Mode");
    }
    if (!params.use_sensor_data_qos) {
        RCLCPP_WARN(get_logger(), "Not Use Sensor Data Qos");
    }
    auto qos = params.use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    image_pub = image_transport::create_publisher(this, params.camera_name, qos);

    // load camera info
    camera_info_manager =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, params.camera_name);
    if (params.camera_info_url != "none" && camera_info_manager->validateURL(params.camera_info_url) && camera_info_manager->loadCameraInfo(params.camera_info_url)) {
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
    init_camera();
    RCLCPP_WARN(get_logger(), "Starting Camera Monitor thread.");
    start_grab();
    // monitor_on = true;
    // monitor_thread = std::thread(&HikCameraNode::monitor, this);
}

OpencvCameraNode::~OpencvCameraNode() { stop_grab(); }

void OpencvCameraNode::load_params() {
    params.camera_name = this->declare_parameter("camera_name", "camera_raw");
    params.device_name = this->declare_parameter("device_name", "video_test");
    params.codec = this->declare_parameter("codec", "XXXX");  // should be MJEG/YUYV/UYVY like
    assert(params.codec == "MJPG" || params.codec == "YUYV" || params.codec == "UYVY");
    params.camera_info_url =
        this->declare_parameter("camera_info_url", "package://opencv_cam/config/camera_info.yaml");
    params.image_width = this->declare_parameter("image_width", 0);
    params.image_height = this->declare_parameter("image_height", 0);
    params.fps = this->declare_parameter("fps", 30.0);
    params.auto_exposure = this->declare_parameter("auto_exposure", true);
    params.auto_white_balance = this->declare_parameter("auto_white_balance", true);
    params.use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    params.white_balance = this->declare_parameter("white_balance", UNDEFINEED);
    params.exposure = this->declare_parameter("exposure", UNDEFINEED);
    params.brightness = this->declare_parameter("brightness", UNDEFINEED);
    params.contrast = this->declare_parameter("contrast", UNDEFINEED);
    params.saturation = this->declare_parameter("saturation", UNDEFINEED);
    params.hue = this->declare_parameter("hue", UNDEFINEED);
    params.gain = this->declare_parameter("gain", UNDEFINEED);
    params.gamma = this->declare_parameter("gamma", UNDEFINEED);
    params.sharpness = this->declare_parameter("sharpness", UNDEFINEED);
    params.backlight = this->declare_parameter("backlight", UNDEFINEED);
    params.show_img = this->declare_parameter("show_img", false);
}

void check_set(std::shared_ptr<cv::VideoCapture> handle, int flag, int value, std::string info = "",
               const rclcpp::Logger& logger = rclcpp::get_logger("normal_logger")) {
    if (value != UNDEFINEED) {
        handle->set(flag, value);
        if (info.size() > 0) {
            double fedback = handle->get(flag);
            RCLCPP_INFO(logger, "Setting %s to %d, after %.2lf", info.c_str(), value, fedback);
        }
    }
}

void print_param(std::shared_ptr<cv::VideoCapture> handle, int flag, std::string info = "",
                 const rclcpp::Logger& logger = rclcpp::get_logger("normal_logger")) {
    double fedback = handle->get(flag);
    RCLCPP_INFO(logger, "Setted %s is %.2lf", info.c_str(), fedback);
}

void OpencvCameraNode::init_camera() {
    handle = std::make_shared<cv::VideoCapture>();
    // open
    bool success_found = false;
    while (rclcpp::ok() && !success_found) {
        int res = system(("ls " + params.device_name).c_str());
        success_found = res == 0;
        if (!success_found) {
            RCLCPP_ERROR(get_logger(), "Failed to find Device: %s", params.device_name.c_str());
            RCLCPP_WARN(get_logger(), "Retrying....");
            handle->release();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    handle->open(params.device_name);
    //fix v4l backend issues when reading yuv images
    if(params.codec == "YUVY" || params.codec == "UYVY"){
        handle->set(cv::CAP_PROP_CONVERT_RGB,0);
    }

    // basic
    check_set(handle, cv::CAP_PROP_FOURCC,
              cv::VideoWriter::fourcc(params.codec[0], params.codec[1], params.codec[2],
                                      params.codec[3]));
    check_set(handle, cv::CAP_PROP_FRAME_HEIGHT, params.image_height, "height", get_logger());
    check_set(handle, cv::CAP_PROP_FRAME_WIDTH, params.image_width, "width", get_logger());
    check_set(handle, cv::CAP_PROP_FPS, params.fps, "fps", get_logger());
    // exposure
    check_set(handle, cv::CAP_PROP_AUTO_EXPOSURE, params.auto_exposure ? 3 : 1, "auto_exposure",
              get_logger());
    if (!params.auto_exposure && params.exposure != UNDEFINEED) {
        check_set(handle, cv::CAP_PROP_EXPOSURE, params.exposure, "exposure", get_logger());
    }
    // white balance
    check_set(handle, cv::CAP_PROP_AUTO_WB, params.auto_white_balance, "auto_white_balance",
              get_logger());
    if (!params.auto_white_balance && params.white_balance != UNDEFINEED) {
        check_set(handle, cv::CAP_PROP_WB_TEMPERATURE, params.white_balance, "white_balance",
                  get_logger());
    }
    check_set(handle, cv::CAP_PROP_BRIGHTNESS, params.brightness, "brightness", get_logger());
    check_set(handle, cv::CAP_PROP_CONTRAST, params.contrast, "contrast", get_logger());
    check_set(handle, cv::CAP_PROP_SATURATION, params.saturation, "saturation", get_logger());
    check_set(handle, cv::CAP_PROP_HUE, params.hue, "hue", get_logger());
    check_set(handle, cv::CAP_PROP_GAIN, params.gain, "gain", get_logger());
    check_set(handle, cv::CAP_PROP_GAMMA, params.gamma, "gamma", get_logger());
    check_set(handle, cv::CAP_PROP_SHARPNESS, params.sharpness, "sharpness", get_logger());
    check_set(handle, cv::CAP_PROP_BACKLIGHT, params.backlight, "backlight", get_logger());
    print_param(handle, cv::CAP_PROP_APERTURE, "aperture", get_logger());
    // print_param(handle, cv::CAP_PROP_BITRATE, "bitrate", get_logger());
    print_param(handle, cv::CAP_PROP_FORMAT, "Mat format", get_logger());
}

void OpencvCameraNode::start_grab() {
    grab_on = true;
    capture_thread = std::thread(&OpencvCameraNode::grab, this);
}

void OpencvCameraNode::stop_grab() {
    grab_on = false;
    if (capture_thread.joinable()) {
        capture_thread.join();
    }
}

void OpencvCameraNode::grab() {
    // 创建消息
    sensor_msgs::msg::Image image_msg;
    image_msg.encoding = "bgr8";
    image_msg.header.frame_id = params.camera_name;
    image_msg.height = params.image_height;
    image_msg.width = params.image_width;
    image_msg.step = params.image_width * 3;
    image_msg.data.resize(params.image_height * params.image_width * 3);
    // zero-copy mat
    cv::Mat img_handle;
    cv::Mat img_origin(image_msg.height, image_msg.width, CV_8UC3, image_msg.data.data());
    Perf grab_perf(params.camera_name, 500);
    Event grab_event;
    grab_event.start();
    while (rclcpp::ok() && grab_on) {
        if(params.codec == "YUVY"){
            handle->read(img_handle);
            cv::cvtColor(img_handle,img_origin,cv::COLOR_YUV2BGR_YUYV);
        } else if (params.codec == "UYVY") {
            handle->read(img_handle);
            cv::cvtColor(img_handle,img_origin,cv::COLOR_YUV2BGR_UYVY);
        } else if (params.codec == "MJPG"){
            handle->read(img_origin);
        }
        grab_event.end();
        grab_perf.update(grab_event.duration());
        grab_event.start();
        if (params.show_img) {
            cv::imshow("grab", img_origin);
            cv::waitKey(1);
        }
        image_msg.header.stamp = this->now();
        image_pub.publish(image_msg);
        if(camera_info_pub){
            camera_info_msg.header.stamp = image_msg.header.stamp;
            camera_info_pub->publish(camera_info_msg);
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam::OpencvCameraNode)