#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Transform.h>

void scan_to_map(const sensor_msgs::msg::LaserScan& scan, 
                nav_msgs::msg::OccupancyGrid& map, 
                const tf2::Transform& transform);