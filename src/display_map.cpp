#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class DisplayMap : public rclcpp::Node
{

public:
    DisplayMap() : Node("display_map"), count_(0)
    {
        message_.header.frame_id = "map";
        message_.info.resolution = 0.1;
        message_.info.width = 100;
        message_.info.height = 100;
        message_.info.origin.position.x = 0.0;
        message_.info.origin.position.y = 0.0;
        message_.info.origin.position.z = 0.0;
        message_.info.origin.orientation.w = 1.0;
        message_.data.resize(message_.info.width * message_.info.height, 0);

        // QoS beállítások
        rclcpp::QoS qos(1);
        qos.transient_local();

        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);
        RCLCPP_INFO(this->get_logger(), "Starting display_map");

        message_.header.stamp = this->now();
        publisher_->publish(message_);
        RCLCPP_INFO(this->get_logger(), "Published map");
    }

private:
    nav_msgs::msg::OccupancyGrid message_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisplayMap>());
    rclcpp::shutdown();
    return 0;
}