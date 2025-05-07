// simple_map.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <string>


class SimpleMap : public rclcpp::Node
{
public:
    SimpleMap() : Node("simple_map")
    {
        // Объявление параметров узла
        this->declare_parameter<std::string>("map_frame", "odom");
        this->declare_parameter<double>("map_resolution", 0.1);
        this->declare_parameter<int>("map_width", 100);
        this->declare_parameter<int>("map_height", 100);
        
        // Получение параметров
        map_frame_ = this->get_parameter("map_frame").as_string();
        map_resolution_ = this->get_parameter("map_resolution").as_double();
        map_width_ = this->get_parameter("map_width").as_int();
        map_height_ = this->get_parameter("map_height").as_int();
        
        // Инициализация TF2 для работы с трансформациями
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Создание публикатора для карты
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/simple_map", 10);
        
        // Создание подписчика на данные лазерного скана
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 100, std::bind(&SimpleMap::laserCallback, this, std::placeholders::_1));
    }

private:

    // Подготовка сообщения с картой
    void prepareMapMessage(nav_msgs::msg::OccupancyGrid &map_msg, const builtin_interfaces::msg::Time &stamp)
    {
        map_msg.header.frame_id = map_frame_;
        map_msg.header.stamp = stamp;
        map_msg.info.height = map_height_;
        map_msg.info.width = map_width_;
        map_msg.info.resolution = map_resolution_;
        map_msg.data.resize(map_height_ * map_width_, -1); // Инициализация неизвестными значениями
    }

    //Получение трансформации между системами координат сканера и карты
    bool determineScanTransform(geometry_msgs::msg::TransformStamped &scanTransform,
                              const builtin_interfaces::msg::Time &stamp,
                              const std::string &laser_frame)
    {
        try
        {
            // Попытка получить трансформацию
            scanTransform = tf_buffer_->lookupTransform(
                map_frame_,
                laser_frame,
                tf2::TimePoint(std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)),
                tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                       laser_frame.c_str(), map_frame_.c_str(), e.what());
            return false;
        }
        return true;
    }

    // Обратный вызов при получении данных лазерного скана
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        geometry_msgs::msg::TransformStamped scanTransform;
        const std::string &laser_frame = scan->header.frame_id;
        const builtin_interfaces::msg::Time &laser_stamp = scan->header.stamp;
        
        // Получение трансформации между сканером и картой
        if (!determineScanTransform(scanTransform, laser_stamp, laser_frame))
        {
            return;
        }
        
        // Создание сообщения с картой
        nav_msgs::msg::OccupancyGrid map_msg;
        prepareMapMessage(map_msg, laser_stamp);
        
        // Преобразование трансформации в формат tf2
        tf2::Transform tf_transform;
        tf2::fromMsg(scanTransform.transform, tf_transform);
        
        // Положение сканера в системе координат карты
        tf2::Vector3 zero_pose(0, 0, 0);
        tf2::Vector3 scan_pose = tf_transform * zero_pose;
        
        RCLCPP_DEBUG(this->get_logger(), "scan pose %f %f", scan_pose.x(), scan_pose.y());
        
        // Установка начала координат карты так, чтобы сканер был в центре
        map_msg.info.origin.position.x = scan_pose.x() - map_width_ * map_resolution_ / 2.0;
        map_msg.info.origin.position.y = scan_pose.y() - map_height_ * map_resolution_ / 2.0;
        
        // Вычисление положения сканера в координатах карты
        int center_y = (scan_pose.y() - map_msg.info.origin.position.y) / map_resolution_;
        int center_x = (scan_pose.x() - map_msg.info.origin.position.x) / map_resolution_;
        
        RCLCPP_DEBUG(this->get_logger(), "publish map %d %d", center_x, center_y);
        
        // Помечаем положение сканера как свободное пространство
        map_msg.data[center_y * map_width_ + center_x] = 0;
        
        int map_idx_max = map_width_ * map_height_;
        
        // Обработка каждого измерения лазера
        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            // Пропуск невалидных измерений
            if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max)
            {
                continue;
            }
            
            float angle = scan->angle_min + i * scan->angle_increment;
            
            // Положение препятствия в системе координат сканера
            tf2::Vector3 obstacle_pose(scan->ranges[i] * cos(angle), scan->ranges[i] * sin(angle), 0.0);
            
            // Трассировка луча от сканера до препятствия (отметка свободного пространства)
            double step = 0.1;
            for (double r = scan->range_min; r < scan->ranges[i] - step; r += step)
            {
                tf2::Vector3 free_pos(r * cos(angle), r * sin(angle), 0.0);
                tf2::Vector3 free_pos_map = tf_transform * free_pos;
                
                int free_x = (free_pos_map.x() - map_msg.info.origin.position.x) / map_resolution_;
                int free_y = (free_pos_map.y() - map_msg.info.origin.position.y) / map_resolution_;
                int map_free_idx = free_y * map_width_ + free_x;
                
                if (map_free_idx > 0 && map_free_idx < map_idx_max)
                {
                    map_msg.data[map_free_idx] = 0; // Свободное пространство
                }
            }
            
            // Положение препятствия в системе координат карты
            tf2::Vector3 obstacle_pose_map = tf_transform * obstacle_pose;
            
            int obstacle_x = (obstacle_pose_map.x() - map_msg.info.origin.position.x) / map_resolution_;
            int obstacle_y = (obstacle_pose_map.y() - map_msg.info.origin.position.y) / map_resolution_;
            int map_idx = obstacle_y * map_width_ + obstacle_x;
            
            if (map_idx > 0 && map_idx < map_idx_max)
            {
                map_msg.data[map_idx] = 100; // Помечаем как занятое пространство
            }
        }
        
        // Публикация карты
        map_pub_->publish(map_msg);
    }

    // Члены класса для работы с ROS2
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Параметры карты
    std::string map_frame_;
    double map_resolution_;
    int map_width_;
    int map_height_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMap>());
    rclcpp::shutdown();
    return 0;
}