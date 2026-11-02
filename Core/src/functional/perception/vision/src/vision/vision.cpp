//////////* Headers *//////////
#include "vision.h"

/* Eigen */
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/util/Constants.h>

/* OpenCV */
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

/* ROS */
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include "kinematics_msgs/msg/transformation.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <sensor_msgs/image_encodings.hpp>

/* YAML-CPP */
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

/* STD */
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <vector>
#include <memory>

///////////////////////////////

//////////* Global Variables *//////////

double perpendicular_distance_between_link4_and_camera_z_axes = 0.03;

////////////////////////////////////////

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

auto Vision::initialization_complete([[maybe_unused]]const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void {
    response->success = true;
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

auto Vision::receive_camera_info_rgb(const sensor_msgs::msg::CameraInfo& msg) -> void {
    this->K = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(msg.k.data());
}

auto Vision::pixels_to_world_coordinate(const Eigen::Vector2d& pos) -> void {
    // Convert the pixel coordinate to homogeneous form.
    Eigen::Vector3d p = { pos.x(), pos.y(), 1 };

    // Prepare some parameters
    auto inv_K = this->K.inverse();

    auto R = this->T.block(0, 0, 3, 3);
    auto t = this->T.topRightCorner<3, 1>();
    
    auto R_T = R.transpose();
    
    /*
     * Calculate Scale(λ)：
     * 
     * Pc = R * Pw + t  ==>  Pw = R^T * Pc - R^T * t (1)
     * Pc = λ * K^-1 * p                             (2)
     * Substitute equation (2) into (1), get Pw = R^T * λ * K^-1 * p - R^T * t
     * 
     * We choose an arbitary 3D coordinate on ground which Z-component equals 0(Zw = 0).
     * So, for Z-component, we get:
     *      Zw = R^T * Zc - R^T * t   
     * ==>  R^T * Zc - R^T * t  = 0   
     * ==>  R^T * λ * K^-1 * z - R^T * t  = 0  
     * ==>  λ = R^T * t /  R^T * K^-1 * z
     */

    auto scale =  (R_T * t)[2] / (R_T * inv_K * p)[2];

    // Calculate world coordinate (Pw = R^T * Pc - R^T * t)
    auto Pc = scale * inv_K * p;
    auto Pw = R_T * Pc - R_T * t;

    std::cout << Pw << std::endl;
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


    this->sub_camera_info_rgb = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info/rgb",
        1,
        std::bind(&Vision::receive_camera_info_rgb, this, std::placeholders::_1)
    );

    this->pub_kinematics_calculator = this->create_publisher<kinematics_msgs::msg::Transformation>("~/extrinsic_parameters", 1);

    // Load config
    this->color = load_config(std::filesystem::current_path().string() + "/src/functional/perception/vision/config/color.yaml");
    this->extrinsic = load_config(std::filesystem::current_path().string() + "/src/functional/perception/vision/config/extrinsic.yaml");

    // Parse config
    parse_config();

    // Complete initialization and notify all other nodes
    this->srv_init = this->create_service<std_srvs::srv::Trigger>(
        "~/initialization_complete", 
        std::bind(&Vision::initialization_complete, this, std::placeholders::_1, std::placeholders::_2)
    );
}

auto Vision::load_config(const std::string& path) -> YAML::Node {
    YAML::Node node;

    try  {
        node = YAML::LoadFile(path);
    } catch (YAML::BadFile& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger(""), 
            "File: %s, Function: %s, Line: %d, Error: %s", 
            __FILE__, 
            __FUNCTION__,
            __LINE__, 
            e.what()
        );

        return node;
    }

    return node;
}

auto Vision::parse_config() -> void {
    this->T = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>(
        extrinsic["extrinsic"].as<std::vector<double>>().data()
    );

    // Eigen::Vector3d t = { 0, 0, perpendicular_distance_between_link4_and_camera_z_axes };
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
    auto lowverb = this->color[color.to_string()]["min"].as<std::vector<int>>();
    auto upperb = this->color[color.to_string()]["max"].as<std::vector<int>>();
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
            if (area >= 1000) {
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

            // RCLCPP_INFO(rclcpp::get_logger(""), "Center:%f, %f", rect.center.x, rect.center.y);
            // RCLCPP_INFO(rclcpp::get_logger(""), "Angle:%f", rect.angle);

            pixels_to_world_coordinate({ rect.center.x, rect.center.y });
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