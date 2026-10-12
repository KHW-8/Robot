#include "camera.h"
#include "std_srvs/srv/trigger.hpp"

//////////* Headers *//////////

/* ROS */
#include <cv_bridge/cv_bridge.hpp>
#include <functional>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

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


auto Camera::initialize() -> void {
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
            "File: %s, Line: %d, Error: Failed to set frame mode.", 
            __FILE__, 
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
            "File: %s, Line: %d, Error: Failed to create stream.", 
            __FILE__, 
            __LINE__
        );

        return;
    }

    // Create publishers
    this->pub_bgr = this->create_publisher<sensor_msgs::msg::Image>("~/frame/bgr", 1);
    this->pub_depth = this->create_publisher<sensor_msgs::msg::Image>("~/frame/depth", 1);
    this->pub_infrared = this->create_publisher<sensor_msgs::msg::Image>("~/frame/infrared", 1);

    // Complete initialization and notify all other nodes
    this->srv_ini = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&Camera::initializetion_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void Camera::start() {
    this->stream->Start();

    auto cv_image = cv_bridge::CvImage();

    while (true) {
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
                cv_image.encoding = "bgr8";
                cv_image.image = bgr_frame;

                sensor_msgs::msg::Image msg;
                cv_image.toImageMsg(msg);

                // Publish a BGR frame
                this->pub_bgr->publish(msg);
            } break;
            case deptrum::FrameType::kIrFrame: { 
                // Acquire an infrared frame
                auto infrared_frame = cv::Mat(frame->rows, frame->cols, CV_8UC1, frame->data.get());

                // Convert cv::Mat to sensor_msg::msg::Image
                auto cv_image = cv_bridge::CvImage();
                cv_image.encoding = "8UC1";
                cv_image.image = infrared_frame;

                sensor_msgs::msg::Image msg;
                cv_image.toImageMsg(msg);

                // Publish an infrared frame
                this->pub_infrared->publish(msg);
            } break;
            case deptrum::FrameType::kDepthFrame: { 
                auto depth_frame = cv::Mat(frame->rows, frame->cols, CV_16UC1, frame->data.get());

                  // Convert cv::Mat to sensor_msg::msg::Image
                auto cv_image = cv_bridge::CvImage();
                cv_image.encoding = "16UC1";
                cv_image.image = depth_frame;

                sensor_msgs::msg::Image msg;
                cv_image.toImageMsg(msg);

                // Publish an infrared frame
                this->pub_depth->publish(msg);
            } break;
            default: break;
            }
        }
    }
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
    // rclcpp::spin(std::make_shared<Camera>());
    rclcpp::shutdown();

    return 0;
}

/////////////////////////////////