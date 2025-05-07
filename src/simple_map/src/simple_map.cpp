// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// // #include "tf/transform_listener.h"
// // #include "simple_map/scan_to_map.h"

// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <memory>
// #include <string>


// // #include "geometry_msgs/msg/twist.hpp"
// // #include "nav_msgs/msg/odometry.hpp"
// // #include "std_msgs/msg/float64.hpp"
// // #include "std_msgs/msg/u_int16.hpp"

// // #include "control.h"
// // #include "voyagercontrol.h"
// // #include "dummy_control.h"
// // #include "wall_control.h"


// #include <chrono>    
// using namespace std::chrono_literals;

// class SimpleMap : public rclcpp::Node{
//     public:
//     SimpleMap() : Node("simple_map"){

//         mapPub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("simple_map", 10);

//         // publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
//         laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "base_scan", 10, std::bind(&ControlSelector::laserCallback, this, std::placeholders::_1));

//         // pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         //     "base_pose_ground_truth", 10, std::bind(&ControlSelector::poseCallback, this, std::placeholders::_1));
        
//         // selector_sub_  = this->create_subscription<std_msgs::msg::UInt16>(
//         //     "/selector", 10, std::bind(&ControlSelector::selectCallback, this, std::placeholders::_1));
            

//         // timer_ = this->create_wall_timer(
//         //     500ms, std::bind(&ControlSelector::timerCallback, this));

//     }

//     ~SimpleMap(){
        
//     }

//     void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
//         tf::StampedTransform scanTransform;
//         const std::string &laser_frame = scan.header.frame_id;
//         const ros::Time &laser_stamp = scan.header.stamp;
//         if (!determineScanTransform(scanTransform, laser_stamp,laser_frame))							
//         {
//             return;
//         }
//         // создаем сообщение карты
//         nav_msgs::OccupancyGrid map_msg;
//         // заполняем информацию о карте - готовим сообщение
//         prepareMapMessage(map_msg, laser_stamp);
//         // положение центра дальномера в СК дальномера
//         tf::Vector3 zero_pose(0, 0, 0);
//         // положение дальномера в СК карты
//         tf::Vector3 scan_pose = scanTransform(zero_pose);
//         ROS_DEBUG_STREAM("scan pose " << scan_pose.x() << " " << scan_pose.y());
//         // задаем начало карты так, чтобы сканнер находился в центре карты
//         map_msg.info.origin.position.x = scan_pose.x() - map_width * map_resolution / 2.0;
//         map_msg.info.origin.position.y = scan_pose.y() - map_height * map_resolution / 2.0;
//         // индексы карты, соответствующие положению центра лазера
//         int center_y = (scan_pose.y() - map_msg.info.origin.position.y) / map_resolution;
//         int center_x = (scan_pose.x() - map_msg.info.origin.position.x) / map_resolution;
//         ROS_DEBUG_STREAM("publish map " << center_x << " " << center_y);
//         // в клетку карты соотвтествующую центру лазера - записываем значение 0
//         map_msg.data[center_y * map_width + center_x] = 0;
//         int map_idx_max = map_width * map_height;
//         // проходим по каждому измерению лидара
//         for (int i = 0; i < scan.ranges.size(); i++)
//         {
//             if (scan.ranges[i] < scan.range_min || scan.ranges[i] >
//             scan.range_max)
//             {
//                 continue;
//             }
//             // Угол в ПСК ЛД
//             float angle = scan.angle_min + i * scan.angle_increment;
//             // вычисляем позицию препятствия в системе координат ЛД
//             tf::Vector3 obstacle_pose(scan.ranges[i] * cos(angle),scan.ranges[i] * sin(angle),0.0);
//             // Шаг для прохода по лучу
//             double step = 0.1;
//             // Идем по лучу от ЛД до препятствия
//             for (double r = scan.range_min; r < scan.ranges[i] - step; r += step)
//             {
//                 // Точка в ДСК ЛД
//                 tf::Vector3 free_pos(r * cos(angle), r * sin(angle), 0.0);
//                 // Точка в ДСК Карты
//                 tf::Vector3 free_pos_map = scanTransform * free_pos;
//                 // коорд точки карты
//                 int free_x = (free_pos_map.x() - map_msg.info.origin.position.x) / map_resolution;
//                 int free_y = (free_pos_map.y() - map_msg.info.origin.position.y) / map_resolution;
//                 // индекс в массиве карты
//                 int map_free_idx = free_y * map_width + free_x;
//                 // проверяем, что ячейка не находится за пределами карты
//                 if (map_free_idx > 0 && map_free_idx < map_idx_max)
//                 {
//                     map_msg.data[map_free_idx] = 0;
//                 }
//             }
//             // вычисляем позицию препятствия в системе координат карты
//             tf::Vector3 obstacle_pose_map = scanTransform * obstacle_pose;
//             // индексы ячейки, соответствующей позиции препятствия
//             int obstacle_x = (obstacle_pose_map.x() - map_msg.info.origin.position.x) / map_resolution;
//             int obstacle_y = (obstacle_pose_map.y() - map_msg.info.origin.position.y) / map_resolution;
//             int map_idx = obstacle_y * map_width + obstacle_x;
//             // проверяем, что ячейка не находится за пределами карты
//             if (map_idx > 0 && map_idx < map_idx_max)
//             {
//                 map_msg.data[map_idx] = 100;
//             }
//             // публикуем сообщение с построенной картой
//             mapPub.publish(map_msg);
//         }
//     }

//     void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
//         // обновляем переменные класса, отвечающие за положение робота
//         x = msg->pose.pose.position.x;
//         y = msg->pose.pose.position.y;
//         theta = 2*atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//         if ( controlPtr ) // если указатель не нулевой - вызываем функцию текущего алгоритма
//             controlPtr->setRobotPose(x, y, theta);
//     }

//     void timerCallback(/*const rclcpp::TimerBase::SharedPtr*/){
//         geometry_msgs::msg::Twist cmd;
//         if ( controlPtr )
//         {
//             controlPtr->getControl(cmd.linear.x, cmd.angular.z);
//         }
//         else {
//             // это сообщение будет печататься не чаще 1 раза в секунду
//             // ROS_INFO_STREAM_THROTTLE(1.0, "no control");
//             RCLCPP_INFO(this->get_logger(), "no control");
//         }
//         // ROS_DEBUG_STREAM("cmd v = "<<cmd.linear.x<<" "<<cmd.angular.z);
//         RCLCPP_DEBUG(this->get_logger(), "cmd v = %d cmd z = %d", cmd.linear.x, cmd.angular.z);
//         // отправляем (публикуем) команду
//         publisher_->publish(cmd);

//     }


//     // функция обработки сообщения по топику '/selector`, управляющего выбором алгоритма
//     // можно отправить сообщение из консоли
//     // rostopic pub /selector std_msgs/UInt16 <номер алгоритма начиная с 0>
//     void selectCallback(const std_msgs::msg::UInt16::SharedPtr msg)
//     {
//         // ROS_INFO_STREAM("select callback "<<msg->data);      
//         RCLCPP_DEBUG(this->get_logger(),"select callback ", msg->data);

//         if ( msg->data >= nControls)
//         {
//             // ROS_ERROR_STREAM("Wrong algorithm number " << msg->data);
//             RCLCPP_WARN(this->get_logger(),"Wrong algorithm number ", msg->data);
//             controlPtr = nullptr;
//         }
//         else
//         {
//             controlPtr = controls[msg->data];
//             // ROS_INFO_STREAM("Select " << controlPtr->getName() << " control");
//             RCLCPP_DEBUG(this->get_logger(),"Select %s control", controlPtr->getName());
//         }
//     }

//     void prepareMapMessage(nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, const ros::Time &stamp)
//     {
//         map_msg.header.frame_id = map_frame;
//         map_msg.header.stamp = stamp;
//         map_msg.info.height = map_height;
//         map_msg.info.width = map_width;
//         map_msg.info.resolution = map_resolution;
//         // изменяем размер вектора, который является хранилищем данных карты, и заполняем его значением (-1) - неизвестное значение
//         map_msg.data.resize(map_height * map_width, -1);
//     }

//     bool determineScanTransform(tf::StampedTransform &scanTransform, const ros::Time &stamp, const std::string &laser_frame)
//     {
//         try
//         {
//             if (!tfListener->waitForTransform(map_frame, laser_frame, stamp, ros::Duration(0.1)))
//             {
//                 ROS_WARN_STREAM("no transform to scan " << laser_frame);
//                 return false;
//             }

//             tfListener->lookupTransform(map_frame, laser_frame, stamp, scanTransform);
//         }

//         catch (tf::TransformException &e)
//         {
//             ROS_ERROR_STREAM("got tf exception " << e.what());
//             return false;
//         }
//         return true;
//     }


//     private:
//     // глобальная переменная - публикатор сообщения карты
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mapPub_;
//     // глоабльный указатель на tfListener, который будет проинициализирован в main
//     tf::TransformListener *tfListener;
//     // имя для СК карты
//     std::string map_frame;
//     // разрешение карты
//     double map_resolution = 0.1;
//     // размер карты в клетках
//     int map_width = 100;
//     int map_height = 100;

//     // подписчики
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
//     // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
//     // rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr selector_sub_;
//     // rclcpp::TimerBase::SharedPtr timer_;
// };


// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SimpleMap>());
//     rclcpp::shutdown();
//     return 0;
// }