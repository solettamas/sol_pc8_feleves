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
        this->declare_parameter("width", 100);
        this->declare_parameter("height", 100);
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();

        message_.header.frame_id = "map";
        message_.info.resolution = 0.1;
        message_.info.width = width_;
        message_.info.height = height_;
        message_.info.origin.position.x = 0.0;
        message_.info.origin.position.y = 0.0;
        message_.info.origin.position.z = 0.0;
        message_.info.origin.orientation.w = 1.0;
        message_.data.assign(width_ * height_, 0);

        // Példa akadályok
        drawObstacle(0, height_ - 1, width_ - 1, height_ - 1); // Felső sor
        drawObstacle(0, 0, width_ - 1, 0);                     // Alsó sor
        drawObstacle(0, 0, 0, height_ - 1);                    // Bal oldal
        drawObstacle(width_ - 1, 0, width_ - 1, height_ - 1);  // Jobb oldal

        drawObstacle(20, 20, 80, 25);
        drawObstacle(15, 40, 65, 50);
        drawObstacle(75, 35, 85, 90);
        drawObstacle(10, 60, 50, 90);

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
    void drawObstacle(int x1, int y1, int x2, int y2)
    {
        for (int row = y1; row <= y2; row++)
        {
            for (int col = x1; col <= x2; col++)
            {
                if (row >= 0 && row < height_ && col >= 0 && col < width_)
                {
                    // Konvertálás indexre
                    int index = (row * width_) + col;
                    message_.data[index] = 100;
                }
            }
        }
    }
    int width_;
    int height_;

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