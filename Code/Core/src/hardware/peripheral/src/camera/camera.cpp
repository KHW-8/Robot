#include "camera.h"

//////////* Headers *//////////

/* ROS */
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include <std_msgs/msg/header.hpp>

/* STD */
#include <functional>

///////////////////////////////

//////////* Functions *//////////

Camera::Camera() 
    : Node("camera")
{
    initialize();
}

auto Camera::initializetion_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void {
    response->success = true;
}

auto Camera::instric_param_to_camera_info(const deptrum::Intrinsic& param) -> sensor_msgs::msg::CameraInfo {
    sensor_msgs::msg::CameraInfo camera_info;

    // Width
    camera_info.width = param.rows;

    // Height
    camera_info.height = param.cols;

    /*
     * Calibration Matrix
     *
     *     [ fx, 0,  cx ]
     * K = [ 0,  fy, cy ]
     *     [ 0,  0,  1  ]
     */
    camera_info.k = {
        param.focal_length[0],
        0,
        param.principal_point[0],
        0,
        param.focal_length[1],
        param.principal_point[1],
        0,
        0,
        1
    };

    return camera_info; 
}

auto Camera::initialize() -> void {
    // Initialize camera
    initialize_camera();

    // Create publishers
    this->pub_bgr_image = this->create_publisher<sensor_msgs::msg::Image>("~/frame/bgr", 1);
    this->pub_depth_image = this->create_publisher<sensor_msgs::msg::Image>("~/frame/depth", 1);
    this->pub_infrared_image = this->create_publisher<sensor_msgs::msg::Image>("~/frame/infrared", 1);

    this->pub_camera_info_infrared = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info/infrared", 1);
    this->pub_camera_info_rgb = this->create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info/rgb", 1);

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&Camera::initializetion_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto Camera::initialize_camera() -> void {
    // Get device list
    std::vector<deptrum::DeviceInformation> device_list;
    deptrum::stream::DeviceManager::GetInstance()->GetDeviceList(device_list);

    // Check device count
    if (device_list.empty()) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: No connected devices.", 
            __FILE__, 
            __LINE__
        );

        return;
    }

    // Open device
    this->device = deptrum::stream::DeviceManager::GetInstance()->CreateDevice(device_list[0]);
    if (this->device->Open() != 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: Failed to open depth camera.", 
            __FILE__, 
            __LINE__
        );

        return;
    }

    // Acquire frame modes
    std::vector<std::tuple<deptrum::FrameMode, deptrum::FrameMode, deptrum::FrameMode> > frame_mode_vec;
    if (this->device->GetSupportedFrameMode(frame_mode_vec) != 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: Failed to acquire device supported frame modes.", 
            __FILE__, 
            __LINE__
        );

        return;
    }

    auto ir_mode = std::get<0>(frame_mode_vec[0]);
    auto rgb_mode = std::get<1>(frame_mode_vec[0]);
    auto depth_mode = std::get<2>(frame_mode_vec[0]);

    // Set frame mode
    if (this->device->SetMode(ir_mode, rgb_mode, depth_mode) != 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: Failed to set frame mode.", 
            __FILE__, 
            __FUNCTION__,
            __LINE__
        );

        return;
    }


    // Create stream
    if (this->device->CreateStream(
        this->stream, 
        {
            deptrum::stream::StreamType::kRgb, 
            deptrum::stream::StreamType::kDepthIr
        }
    ) != 0) 
    {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: Failed to create stream.", 
            __FILE__, 
            __FUNCTION__,
            __LINE__
        );

        return;
    }
}

auto Camera::publish_images() -> void {
    deptrum::stream::StreamFrames frames;
    stream->GetFrames(frames);

    for (int i = 0; i < frames.count; i++) {
        auto frame = frames.frame_ptr[i];
        switch (frame->frame_type) {
        case deptrum::FrameType::kRgbFrame: { 
            // Acquire a BGR frame
            cv::Mat yuv_frame(frame->rows*1.5, frame->cols, CV_8UC1, frame->data.get());
            cv::Mat bgr_frame;
            cv::cvtColor(yuv_frame, bgr_frame, cv::COLOR_YUV2BGR_NV12);
            
            // Convert cv::Mat to sensor_msg::msg::Image
            auto msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), 
                sensor_msgs::image_encodings::BGR8,
                bgr_frame
            ).toImageMsg();

            // Publish a BGR frame
            this->pub_bgr_image->publish(*msg);
        } break;
        case deptrum::FrameType::kIrFrame: { 
            // Acquire an infrared frame
            auto infrared_frame = cv::Mat(frame->rows, frame->cols, CV_8UC1, frame->data.get());

            // Convert cv::Mat to sensor_msg::msg::Image
                auto msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), 
                sensor_msgs::image_encodings::TYPE_8UC1,
                infrared_frame
            ).toImageMsg();

            // Publish an infrared frame
            this->pub_infrared_image->publish(*msg);
        } break;
        case deptrum::FrameType::kDepthFrame: { 
            auto depth_frame = cv::Mat(frame->rows, frame->cols, CV_16UC1, frame->data.get());

            // Convert cv::Mat to sensor_msg::msg::Image
            auto msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), 
                sensor_msgs::image_encodings::TYPE_16UC1,
                depth_frame
            ).toImageMsg();

            // Publish an infrared frame
            this->pub_depth_image->publish(*msg);
        } break;
        default: break;
        }
    }
}

auto Camera::publish_camera_info() -> void {
    deptrum::Intrinsic infrared_intrinsic_param;
    deptrum::Intrinsic rgb_intrinsic_param;
    deptrum::Extrinsic extrinsic_param;

    // Get camera parameters
    if (device->GetCameraParameters(
        infrared_intrinsic_param,
        rgb_intrinsic_param, 
        extrinsic_param
    ) == 0) 
    { // If camera parameters are valid, then publish them.
        this->pub_camera_info_infrared->publish(instric_param_to_camera_info(infrared_intrinsic_param));
        this->pub_camera_info_rgb->publish(instric_param_to_camera_info(rgb_intrinsic_param));
    }
}

void Camera::start() {
    this->stream->Start();

    publish_camera_info();

    while (true) 
        publish_images();
}

void Camera::stop() {
    this->stream->Stop();
    this->device->DestroyStream(stream);

    this->device->Close();
}


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Camera>();
    node->start();
    rclcpp::shutdown();

    return 0;
}

/////////////////////////////////