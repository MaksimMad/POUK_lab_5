#include "simple_map/scan_to_map.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>


void scan_to_map(const sensor_msgs::msg::LaserScan& scan, 
                nav_msgs::msg::OccupancyGrid& map, 
                const tf2::Transform& transform) 
{
    

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        
        double angle = scan.angle_min + scan.angle_increment * i;
        // Точка в системе координат лидара
        tf2::Vector3 point_lidar(scan.ranges[i] * std::cos(angle),
                                scan.ranges[i] * std::sin(angle),
                                0.0);
        
        // Преобразование в систему координат карты
        tf2::Vector3 point_map = transform * point_lidar;
        
        // Переход к координатам карты
        point_map -= tf2::Vector3(map.info.origin.position.x,
                                 map.info.origin.position.y,
                                 0.0);

        // Проверка границ карты
        if (point_map.x() >= 0 && point_map.y() >= 0 &&
            point_map.x() < map.info.width * map.info.resolution &&
            point_map.y() < map.info.height * map.info.resolution) 
        {
            int x = static_cast<int>(point_map.x() / map.info.resolution);
            int y = static_cast<int>(point_map.y() / map.info.resolution);
            
            // Запись в карту (100 - занято, 0 - свободно)
            if (y * map.info.width + x < static_cast<int>(map.data.size())) {
                map.data[y * map.info.width + x] = 100;
            }
        }
    }
}