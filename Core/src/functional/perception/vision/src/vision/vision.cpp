//////////* Headers *//////////
#include "vision.h"
#include "std_srvs/srv/trigger.hpp"

/* OpenCV */
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>

/* ROS */
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

///////////////////////////////

//////////* Functions *//////////
Vision::Vision() 
    :Node("vision")
{
    initialize();
}

void Vision::receive_bgr_frame(const sensor_msgs::msg::Image& msg) {
    auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");
    auto bgr_frame = cv_image->image;
    
    // Convert BGR to LAB
    cv::Mat lab_frame;
    cv::cvtColor(bgr_frame, lab_frame, cv::COLOR_BGR2Lab);

    // Convert LAB to threthord
    cv::Mat thresholded_frame;
    std::vector lowverb{34, 145, 98};
    std::vector upperb{255, 255, 255};
    cv::inRange(lab_frame, lowverb, upperb, thresholded_frame);


    cv::imshow("BGR", bgr_frame);
    cv::imshow("LAB", lab_frame);
    cv::imshow("Thresholding", thresholded_frame);
    cv::waitKey(1);
}

void Vision::receive_depth_frame(const sensor_msgs::msg::Image& msg) {
    auto cv_image = cv_bridge::toCvCopy(msg, "8UC1");
    auto depth_frame = cv_image->image;

    // cv::imshow("Depth", depth_frame);
    // cv::waitKey(1);
}

void Vision::receive_infrared_frame(const sensor_msgs::msg::Image& msg) {
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
}


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Vision>());
    rclcpp::shutdown();

    return 0;
}

/////////////////////////////////