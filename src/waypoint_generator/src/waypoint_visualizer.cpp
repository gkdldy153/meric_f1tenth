#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <fstream>
#include <sstream>
#include <string>

class WaypointVisualizer : public rclcpp::Node {
public:
    WaypointVisualizer() : Node("waypoint_visualizer") {
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_marker", 10);
        this->declare_parameter<std::string>("csv_path", "/home/meric/f1tenth_ws/src/waypoint_generator/src/waypoints_opt.csv");

        std::string csv_path = this->get_parameter("csv_path").as_string();

        // Load waypoints from CSV
        std::ifstream file(csv_path);
        std::string line;
        int id = 0;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str;
            if (!std::getline(ss, x_str, ',')) continue;
            if (!std::getline(ss, y_str)) continue;

            double x = std::stod(x_str);
            double y = std::stod(y_str);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.ns = "waypoints";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0.1;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.r = 0.3f;
            marker.color.g = 0.2f;
            marker.color.b = 0.7f;
            marker.color.a = 1.0f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // Forever
            marker_array_.markers.push_back(marker);
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&WaypointVisualizer::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints.", marker_array_.markers.size());
    }

private:
    void timer_callback() {
        for (auto &marker : marker_array_.markers) {
            marker.header.stamp = this->now();
        }
        publisher_->publish(marker_array_);
        RCLCPP_INFO(this->get_logger(), "Published %zu waypoints.", marker_array_.markers.size());
    }

    visualization_msgs::msg::MarkerArray marker_array_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointVisualizer>());
    rclcpp::shutdown();
    return 0;
}

