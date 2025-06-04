#include "waypoint_generator.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

WaypointGenerator::WaypointGenerator() : Node("waypoint_generator_node"), x_old(0.0), y_old(0.0) {
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("min_distance", 0.05);
    this->declare_parameter("save_path", "/home/meric/f1tenth/src/waypoint_generator/src/waypoints_meric.csv");

    odom_topic = this->get_parameter("odom_topic").as_string();
    min_distance = this->get_parameter("min_distance").as_double();
    save_path = this->get_parameter("save_path").as_string();

    last_saved_time = this->now();  // 초기값

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 1000, std::bind(&WaypointGenerator::odom_callback, this, std::placeholders::_1));
}

void WaypointGenerator::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
    rclcpp::Time current_time = this->now();
    rclcpp::Duration time_diff = current_time - last_saved_time;

    // 0.1초(=100ms) 이상 지났을 때만 저장
    if (time_diff.seconds() < 0.18) {
        return;
    }

    double diff = std::hypot(
        odom_submsgObj->pose.pose.position.x - x_old,
        odom_submsgObj->pose.pose.position.y - y_old);

    if (diff > min_distance) {
        double x = odom_submsgObj->pose.pose.position.x;
        double y = odom_submsgObj->pose.pose.position.y;

        std::ofstream csv_odom(save_path, std::ios::out | std::ios::app);
        if (!csv_odom.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", save_path.c_str());
            return;
        }

        csv_odom << x << ", " << y << "\n";
        csv_odom.close();

        x_old = x;
        y_old = y;
        last_saved_time = current_time;

        RCLCPP_INFO(this->get_logger(), "Saved: %f, %f | dist: %f", x, y, diff);
    }
}

// 
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

