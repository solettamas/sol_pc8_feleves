#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std;


struct GridCell
{
    int row;
    int col;

    bool operator==(const GridCell other) const
    {
        return row == other.row && col == other.col;
    }
};

struct OpenEntry
{
    double f_cost;
    GridCell cell;
    bool operator>(const OpenEntry rhs) const
    {
        return f_cost > rhs.f_cost;
    }
};

class AStarPlanner
{
public:
    AStarPlanner(rclcpp::Clock::SharedPtr clock) : clock_(clock) {}

    bool planStep()
    {
        if (finished_)
        {
            return false;
        }

        if (open_pq_.empty())
        {
            finished_ = true;
            return false;
        }

        // Legkisebb f-költségű cella kivétele
        OpenEntry current = open_pq_.top();
        open_pq_.pop();

        // Ha a cella már lezárt, kihagyás.
        const size_t current_index = cellIndex(current.cell);
        if (closed_set_[current_index])
        {
            return true; 
        }

        open_set_[current_index] = false;
        closed_set_[current_index] = true;
        nodes_expanded_++;

        // Cél ellenőrzése
        if (current.cell == goal_cell_)
        {
            finished_ = true;
            path_found_ = true;
            reconstructPath();
            return false;
        }

        // A 8 szomszédos cella vizsgálata.
        static constexpr int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        static constexpr int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

        for (int i = 0; i < 8; ++i)
        {
            GridCell neighbour{current.cell.row + dx[i], current.cell.col + dy[i]};

            if (!inBounds(neighbour) || isOccupied(neighbour))
            {
                continue;
            }

            const size_t neighbour_index = cellIndex(neighbour);
            if (closed_set_[neighbour_index])
            {
                continue;
            }

            // Átlós lépés költsége: sqrt(2), egyenes lépés: 1.
            double move_cost = (dx[i] != 0 && dy[i] != 0) ? sqrt(2.0) : 1.0;
            double tentative_g = g_cost_[current_index] + move_cost;

            if (tentative_g < g_cost_[neighbour_index])
            {
                g_cost_[neighbour_index] = tentative_g;
                came_from_[neighbour_index] = current.cell;
                double f = tentative_g + heuristic(neighbour, goal_cell_);
                open_pq_.push({f, neighbour});
                open_set_[neighbour_index] = true;
            }
        }

        return true;
    }

    void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        map_ = map;
        map_width_ = static_cast<int>(map_->info.width);
        map_height_ = static_cast<int>(map_->info.height);
        resolution_ = map_->info.resolution;
        min_x_ = map_->info.origin.position.x;
        min_y_ = map_->info.origin.position.y;
    }

    void initialize(Point start, Point goal)
    {
        start_ = start;
        goal_ = goal;
        start_cell_ = worldToGrid(start_);
        goal_cell_ = worldToGrid(goal_);

        open_pq_ = {};
        const size_t cell_count =
            static_cast<size_t>(map_width_) * static_cast<size_t>(map_height_);
        open_set_.assign(cell_count, false);
        closed_set_.assign(cell_count, false);
        came_from_.assign(cell_count, GridCell{-1, -1});
        g_cost_.assign(cell_count, numeric_limits<double>::infinity());
        final_path_.clear();

        finished_ = false;
        path_found_ = false;
        nodes_expanded_ = 0;

        if (!inBounds(start_cell_) || !inBounds(goal_cell_))
        {
            finished_ = true;
            return;
        }

        // A nyílt halmaz feltöltése a kezdőcellával.
        double h = heuristic(start_cell_, goal_cell_);
        open_pq_.push({h, start_cell_});
        const size_t start_index = cellIndex(start_cell_);
        open_set_[start_index] = true;
        g_cost_[start_index] = 0.0;
    }

    visualization_msgs::msg::MarkerArray getVisualizationMarkers() const
    {
        visualization_msgs::msg::MarkerArray markers;

        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = clock_->now();
            m.ns = "astar_search";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::CUBE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = resolution_;
            m.scale.y = resolution_;
            m.scale.z = 0.02;
            m.color.r = 0.56f;
            m.color.g = 0.93f;
            m.color.b = 0.56f;
            m.color.a = 0.6f;
            m.pose.orientation.w = 1.0;

            for (int row = 0; row < map_height_; ++row)
            {
                for (int col = 0; col < map_width_; ++col)
                {
                    if (!closed_set_[cellIndex({row, col})])
                    {
                        continue;
                    }

                    geometry_msgs::msg::Point pt;
                    Point w = gridToWorld({row, col});
                    pt.x = w.x;
                    pt.y = w.y;
                    pt.z = 0.01;
                    m.points.push_back(pt);
                }
            }
            markers.markers.push_back(m);
        }

        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = clock_->now();
            m.ns = "astar_search";
            m.id = 1;
            m.type = visualization_msgs::msg::Marker::CUBE_LIST;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = resolution_;
            m.scale.y = resolution_;
            m.scale.z = 0.02;
            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 1.0f;
            m.color.a = 0.7f;
            m.pose.orientation.w = 1.0;

            for (int row = 0; row < map_height_; ++row)
            {
                for (int col = 0; col < map_width_; ++col)
                {
                    if (!open_set_[cellIndex({row, col})])
                    {
                        continue;
                    }

                    geometry_msgs::msg::Point pt;
                    Point w = gridToWorld({row, col});
                    pt.x = w.x;
                    pt.y = w.y;
                    pt.z = 0.01;
                    m.points.push_back(pt);
                }
            }
            markers.markers.push_back(m);
        }

        if (!final_path_.empty())
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = clock_->now();
            m.ns = "astar_path";
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.08; // Vonalvastagság.
            m.color.r = 1.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
            m.pose.orientation.w = 1.0;

            for (const auto wp : final_path_)
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

private:
    size_t cellIndex(const GridCell &cell) const
    {
        return static_cast<size_t>(cell.row) * static_cast<size_t>(map_width_) + static_cast<size_t>(cell.col);
    }

    GridCell worldToGrid(Point p) const
    {
        int col = static_cast<int>(floor((p.x - min_x_) / resolution_));
        int row = static_cast<int>(floor((p.y - min_y_) / resolution_));
        return {row, col};
    }

    Point gridToWorld(const GridCell &c) const
    {
        double x = min_x_ + (c.col + 0.5) * resolution_;
        double y = min_y_ + (c.row + 0.5) * resolution_;
        return {x, y};
    }

    bool isPathFound() const
    {
        return path_found_;
    }

    bool inBounds(const GridCell c) const
    {
        return c.row >= 0 && c.row < map_height_ &&
               c.col >= 0 && c.col < map_width_;
    }

    bool isOccupied(const GridCell c) const
    {
        int8_t val = map_->data[cellIndex(c)];
        return val != 0;
    }

    double heuristic(const GridCell &a, const GridCell &b) const
    {
        double dr = abs(a.row - b.row);
        double dc = abs(a.col - b.col);
        return (dr + dc) + (sqrt(2.0) - 2.0) * min(dr, dc);
    }

    void reconstructPath()
    {
        final_path_.clear();
        GridCell cur = goal_cell_;
        while (!(cur == start_cell_))
        {
            final_path_.push_back(gridToWorld(cur));
            cur = came_from_[cellIndex(cur)];
        }
        final_path_.push_back(gridToWorld(start_cell_));
        reverse(final_path_.begin(), final_path_.end());
    }

    double min_x_ = 0.0;
    double min_y_ = 0.0;
    int width_ = 0;
    int height_ = 0;
    double resolution_ = 0.1;
    bool finished_ = false;
    int map_width_ = 0;
    int map_height_ = 0;
    GridCell start_cell_ = {0, 0};
    GridCell goal_cell_ = {10, 10};
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    vector<Point> final_path_;
    Point start_ = {1, 1};
    Point goal_ = {10.0, 10.0};
    rclcpp::Clock::SharedPtr clock_;

    int nodes_expanded_ = 0;
    bool path_found_ = false;
    priority_queue<OpenEntry, vector<OpenEntry>, greater<OpenEntry> > open_pq_;
    vector<bool> open_set_;
    vector<bool> closed_set_;
    vector<GridCell> came_from_;
    vector<double> g_cost_;
};
