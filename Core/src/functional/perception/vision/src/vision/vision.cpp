//////////* Headers *//////////
#include "vision.h"
#include "std_msgs/msg/header.hpp"

/* OpenCV */
#include <algorithm>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>

/* ROS */
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

/* YAML-CPP */
#include <sensor_msgs/image_encodings.hpp>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

/* STD */
#include <vector>
#include <memory>
#include <filesystem>

///////////////////////////////

//////////* Functions *//////////

Color::Color(const std::string& type) {
    this->type = this->to_type(type);
}

auto Color::to_type(const std::string& type) -> Color::Type {
    if (type == "black")
        return Type::BLACK;
    else if (type == "blue")
        return Type::BLUE;
    else if (type == "red")
        return Type::RED;

    return Type::NONE;
}
auto Color::to_string() -> std::string {
    switch (this->type) {
    case Type::BLACK: return "black";
    case Type::BLUE: return "blue";
    case Type::RED: return "red";
    default: return "";
    }
}

Vision::Vision() 
    :Node("vision")
{
    initialize();
}

auto Vision::receive_bgr_frame(const sensor_msgs::msg::Image& msg) -> void {
    auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");
    auto bgr_frame = cv_image->image;

    recognize_color(msg, Color(Color::RED));

    // cv::imshow("BGR", bgr_frame);
    // cv::waitKey(1);
}

auto Vision::receive_depth_frame(const sensor_msgs::msg::Image& msg) -> void {
    auto cv_image = cv_bridge::toCvCopy(msg, "8UC1");
    auto depth_frame = cv_image->image;

    // cv::imshow("Depth", depth_frame);
    // cv::waitKey(1);
}

auto Vision::receive_infrared_frame(const sensor_msgs::msg::Image& msg) -> void {
    auto cv_image = cv_bridge::toCvCopy(msg, "16UC1");
    auto infrared_frame = cv_image->image;

    // cv::imshow("Infrared", infrared_frame);
    // cv::waitKey(1);
}

auto Vision::initialize() -> void {
    // Waiting for camera
    this->cli_camera = this->create_client<std_srvs::srv::Trigger>("/camera/initialization_complete");
    this->cli_camera->wait_for_service();

    // Create subscriptions
    this->sub_bgr_frame = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/frame/bgr", 
        1, 
        std::bind(&Vision::receive_bgr_frame, this, std::placeholders::_1)
    );

    this->sub_depth_frame = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/frame/depth", 
        1, 
        std::bind(&Vision::receive_depth_frame, this, std::placeholders::_1)
    );

    this->sub_infrared_frame = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/frame/infrared", 
        1, 
        std::bind(&Vision::receive_infrared_frame, this, std::placeholders::_1)
    );

    load_config();
}

auto Vision::load_config() -> bool {
    auto config_path = std::filesystem::current_path().string() + "/src/functional/perception/vision/config/config.yaml";

    try  {
        this->config = YAML::LoadFile(config_path);
    } catch (YAML::BadFile& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Line: %d, Error: %s", 
            __FILE__, 
            __LINE__, 
            e.what()
        );

        return false;
    }

    return true;
}


auto Vision::recognize_color(const sensor_msgs::msg::Image& msg, 
                             Color color)-> void {
    // Get BGR frame
    auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");
    auto bgr_frame = cv_image->image;

    if (bgr_frame.empty())
        return;

    // Reduce BGR noise
    cv::Mat blured_bgr_frame;
    cv::GaussianBlur(bgr_frame, blured_bgr_frame, {3, 3}, 3);
    
    // Convert BGR to LAB
    cv::Mat lab_frame;
    cv::cvtColor(blured_bgr_frame, lab_frame, cv::COLOR_BGR2Lab);

    // Convert LAB to threthord
    cv::Mat thresholded_frame;
    auto lowverb = config[color.to_string()]["min"].as<std::vector<int>>();
    auto upperb = config[color.to_string()]["max"].as<std::vector<int>>();
    cv::inRange(lab_frame, lowverb, upperb, thresholded_frame);

    // Find color contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(thresholded_frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    if (!contours.empty()) {
        auto max_it = std::max_element(contours.begin(), contours.end(), 
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });

        if (max_it != contours.end()) {
            auto area = std::abs(cv::contourArea(*max_it));
            if (area >= 2000) {
                RCLCPP_INFO(rclcpp::get_logger(""), "Detected %s", color.to_string().c_str());

                auto msg = cv_bridge::CvImage(
                    std_msgs::msg::Header(), 
                    sensor_msgs::image_encodings::MONO8,
                    thresholded_frame
                ).toImageMsg();

                find_center_point(*msg);
            }
        }
    }

    cv::imshow("BGR", bgr_frame);
    cv::imshow("Thresholding", thresholded_frame);
    cv::waitKey(1);
}


auto Vision::find_center_point(const sensor_msgs::msg::Image& msg) -> void {
    auto cv_image = cv_bridge::toCvCopy(msg, "mono8");
    auto gray_frame = cv_image->image;

    if (gray_frame.empty())
        return;

    // Find color contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gray_frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    if (!contours.empty()) {
        auto max_it = std::max_element(contours.begin(), contours.end(), 
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });

        if (max_it != contours.end()) {
            auto rect = cv::minAreaRect(*max_it);

            RCLCPP_INFO(rclcpp::get_logger(""), "%f, %f", rect.center.x, rect.center.y);
        }
    }
}


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Vision>());
    rclcpp::shutdown();

    return 0;
}

/////////////////////////////////