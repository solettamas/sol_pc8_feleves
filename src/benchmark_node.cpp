#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class BenchmarkNode : public rclcpp::Node
{
public:
  BenchmarkNode() : Node("benchmark_node")
  {
    this->declare_parameter<double>("start_x_", 1.0);
    this->declare_parameter<double>("start_y_", 1.0);

    rclcpp::QoS qos(1);
    qos.transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos,
        std::bind(&BenchmarkNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1,
        std::bind(&BenchmarkNode::goalCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/planner_markers", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&BenchmarkNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Benchmark node started. Waiting for /map");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
    RCLCPP_INFO(this->get_logger(), "Map received!");
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    has_goal_ = true;
    is_finished_ = false;

    RCLCPP_INFO(this->get_logger(), "Goal set to (%.2f, %.2f).", goal_x_, goal_y_);

  }

  void timerCallback()
  {
    if (!map_)
    {
      // Várakozás a térképre
      return;
    }
    if (!has_goal_)
    {
      // Várakozás a célpontra
      return;
    }
    if (!is_finished_)
    {
      //is_finished_ = !rrt_->planStep();
      //marker_pub_->publish(rrt_->getVisualizationMarkers());
      return;
    }
  }

  double start_x_ = 1.0;
  double start_y_ = 1.0;
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  bool has_goal_ = false;
  bool is_finished_ = false;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  //std::shared_ptr<RRT> rrt_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
