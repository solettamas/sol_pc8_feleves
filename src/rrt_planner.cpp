#include <vector>
#include <random>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std;

struct Point
{
    double x;
    double y;
};

struct Node
{
    Point position;
    int parent;
};

class RRTPlanner
{
public:
    RRTPlanner(rclcpp::Clock::SharedPtr clock) : rng_(random_device{}()), clock_(clock) {}

    bool planstep()
    {
        if (finished_)
        {
            return true;
        }

        // 1. Random pont generálás
        Point sample = randomPoint();

        // 2. Legközelebbi node keresése
        int best_idx = 0;
        double best_dist = numeric_limits<double>::max();

        for (size_t i = 0; i < nodes_.size(); i++)
        {
            double d = hypot(nodes_[i].position.x - sample.x,
                             nodes_[i].position.y - sample.y);
            if (d < best_dist)
            {
                best_dist = d;
                best_idx = i;
            }
        }
        Point nearest_point = nodes_[best_idx].position;

        // 3. Új pont létrehozása a random pont irányában
        Point new_point;

        double dx = sample.x - nearest_point.x;
        double dy = sample.y - nearest_point.y;
        double dist = hypot(dx, dy);

        if (dist <= step_size_)
        {
            new_point = sample;
        }
        else
        {
            double ratio = step_size_ / dist;
            new_point = {nearest_point.x + dx * ratio, nearest_point.y + dy * ratio};
        }

        // 4. Ütközés ellenőrzés
        if (!isCollisionFree(nearest_point, new_point))
        {
            return false;
        }

        // 5. Új node hozzáadása
        int new_idx = nodes_.size();
        nodes_.push_back({new_point, best_idx});

        // 6. Célpont elérése ellenőrzés
        double distance = hypot(new_point.x - goal_.x,
                                new_point.y - goal_.y);
        if (distance <= step_size_)
        {
            if (isCollisionFree(new_point, goal_))
            {
                int goal_idx = nodes_.size();
                nodes_.push_back({goal_, new_idx});
                finished_ = true;
                constructPath(goal_idx);
                return true;
            }
        }

        return false;
    }

    visualization_msgs::msg::MarkerArray getVisualizationMarkers() const
    {
        visualization_msgs::msg::MarkerArray markers;

        // Keresési fa
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = clock_->now();
            m.ns = "rrt_search";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::LINE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.02; // Vonal szélessége
            m.color.r = 1.0f;
            m.color.g = 0.2f;
            m.color.b = 0.2f;
            m.color.a = 0.8f;
            m.pose.orientation.w = 1.0;

            for (size_t i = 1; i < nodes_.size(); ++i)
            {
                const Node child = nodes_[i];
                const Node parent = nodes_[static_cast<size_t>(child.parent)];

                geometry_msgs::msg::Point p1;
                p1.x = parent.position.x;
                p1.y = parent.position.y;
                p1.z = 0.01;

                geometry_msgs::msg::Point p2;
                p2.x = child.position.x;
                p2.y = child.position.y;
                p2.z = 0.01;

                m.points.push_back(p1);
                m.points.push_back(p2);
            }
            markers.markers.push_back(m);
        }

        // Végső útvonal
        if (!final_path_.empty())
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = clock_->now();
            m.ns = "rrt_path";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.08;
            m.color.r = 1.0f;
            m.color.g = 0.2f;
            m.color.b = 0.8f;
            m.color.a = 1.0f;
            m.pose.orientation.w = 1.0;

            for (const auto &wp : final_path_)
            {
                geometry_msgs::msg::Point pt;
                pt.x = wp.x;
                pt.y = wp.y;
                pt.z = 0.05;
                m.points.push_back(pt);
            }
            markers.markers.push_back(m);
        }

        return markers;
    }

    void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        map_ = map;
        min_x_ = map_->info.origin.position.x;
        min_y_ = map_->info.origin.position.y;
        max_x_ = min_x_ + map_->info.width * map_->info.resolution;
        max_y_ = min_y_ + map_->info.height * map_->info.resolution;
        resolution_ = map_->info.resolution;
        width_ = map_->info.width;
        height_ = map_->info.height;
    }

    void Initialize(Point start, Point goal)
    {
        start_ = start;
        goal_ = goal;
        nodes_.clear();
        nodes_.push_back({start_, -1});
        finished_ = false;
        final_path_.clear();
    }

private:
    Point randomPoint()
    {
        bernoulli_distribution choose_goal(goal_bias_);
        if (choose_goal(rng_))
        {
            return goal_;
        }

        uniform_real_distribution<double> x_dist_(min_x_, max_x_);
        uniform_real_distribution<double> y_dist_(min_y_, max_y_);
        return {x_dist_(rng_), y_dist_(rng_)};
    }

    bool isCollisionFree(const Point a, const Point b) const
    {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double dist = hypot(dx, dy);
        double step = resolution_ * 0.5;
        int num_steps = static_cast<int>(ceil(dist / step));

        for (int i = 0; i <= num_steps; i++)
        {
            double t = (num_steps == 0) ? 0.0 : static_cast<double>(i) / num_steps;
            double wx = a.x + dx * t;
            double wy = a.y + dy * t;
            if (isOccupied(wx, wy))
            {
                return false;
            }
        }
        return true;
    }

    bool isOccupied(double wx, double wy) const
    {
        int col = static_cast<int>(floor((wx - min_x_) / resolution_));
        int row = static_cast<int>(floor((wy - min_y_) / resolution_));

        if (row < 0 || row >= height_ || col < 0 || col >= width_)
        {
            return true;
        }

        int idx = row * width_ + col;
        int8_t val = map_->data[idx];
        return val != 0;
    }

    void constructPath(int goal_node_idx) // Végső útvonal felesleges Node-ok nélkül
    {
        int idx = goal_node_idx;
        while (idx >= 0)
        {
            final_path_.push_back(nodes_[idx].position);
            idx = nodes_[idx].parent;
        }
        reverse(final_path_.begin(), final_path_.end());
    }

    double min_x_ = 0.0;
    double max_x_ = 10.0;
    double min_y_ = 0.0;
    double max_y_ = 10.0;
    int width_ = 0;
    int height_ = 0;
    double step_size_ = 0.3;
    double goal_bias_ = 0.1;
    double resolution_ = 0.1;
    bool finished_ = false;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    vector<Node> nodes_;
    vector<Point> final_path_;
    Point start_ = {1, 1};
    Point goal_ = {10.0, 10.0};
    mt19937 rng_;
    rclcpp::Clock::SharedPtr clock_;
};