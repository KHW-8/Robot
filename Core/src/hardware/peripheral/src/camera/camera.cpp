#include "camera.h"

//////////* Headers *//////////

/* OpenCV */
#include <opencv2/highgui.hpp>

/* ROS */
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/logging.hpp>

/* STD */

///////////////////////////////

//////////* Functions *//////////

Camera::Camera() 
    : Node("camera")
{
    initialize();
}


auto Camera::initialize() -> void {
    // Get device list
    std::vector<deptrum::DeviceInformation> device_list;
    deptrum::stream::DeviceManager::GetInstance()->GetDeviceList(device_list);

    // Check device count
    if (device_list.empty()) {
        std::cerr << "No connected devices." << std::endl;
        return;
    }

    // Open device
    this->device = deptrum::stream::DeviceManager::GetInstance()->CreateDevice(device_list[0]);
    if (this->device->Open() != 0) {
        std::cerr << "Failed to open depth camera." << std::endl;
        return;
    }

    // Acquire frame modes
    std::vector<std::tuple<deptrum::FrameMode, deptrum::FrameMode, deptrum::FrameMode> > frame_mode_vec;
    if (this->device->GetSupportedFrameMode(frame_mode_vec) != 0) {
        std::cerr << "Failed to acquire device supported frame modes." << std::endl;
        return;
    }

    auto ir_mode = std::get<0>(frame_mode_vec[0]);
    auto rgb_mode = std::get<1>(frame_mode_vec[0]);
    auto depth_mode = std::get<2>(frame_mode_vec[0]);

    // Set frame mode
    if (this->device->SetMode(ir_mode, rgb_mode, depth_mode) != 0) {
        std::cerr << "Failed to set frame mode." << std::endl;
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
        std::cerr << "Failed to create stream." << std::endl;
        return;
    }
}

auto Camera::depth_frame(const std::shared_ptr<deptrum::stream::StreamFrame>& frame) -> cv::Mat {
    return  cv::Mat(frame->rows, frame->cols, CV_16UC1, frame->data.get());
}

auto Camera::ir_frame(const std::shared_ptr<deptrum::stream::StreamFrame>& frame) -> cv::Mat {
    return  cv::Mat(frame->rows, frame->cols, CV_8UC1, frame->data.get());
}

auto Camera::rgb_frame(const std::shared_ptr<deptrum::stream::StreamFrame>& frame) -> cv::Mat {
    cv::Mat yuv_frame(frame->rows*1.5, frame->cols, CV_8UC1, frame->data.get());
    cv::Mat rgb_frame(frame->rows, frame->cols, CV_8UC3);
    cv::cvtColor(yuv_frame, rgb_frame, cv::COLOR_YUV2BGR_NV12);
    
    return rgb_frame;
}

void Camera::start() {
    this->stream->Start();

    cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("ir", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("rgb", cv::WINDOW_AUTOSIZE);

    while (true) {
        deptrum::stream::StreamFrames frames;
        stream->GetFrames(frames);

        for (int i = 0; i < frames.count; i++) {
            auto frame = frames.frame_ptr[i];
            switch (frame->frame_type) {
            case deptrum::FrameType::kRgbFrame: cv::imshow("rgb", rgb_frame(frame)); break;
            case deptrum::FrameType::kIrFrame: cv::imshow("ir", ir_frame(frame)); break;
            case deptrum::FrameType::kDepthFrame: cv::imshow("depth", depth_frame(frame)); break;
            default: break;
            }
        }

        cv::waitKey(1);
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