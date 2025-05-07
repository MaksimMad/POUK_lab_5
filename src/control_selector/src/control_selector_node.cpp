#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"

#include "control.h"
#include "voyagercontrol.h"
#include "dummy_control.h"
#include "wall_control.h"


#include <chrono>    
using namespace std::chrono_literals;

class ControlSelector : public rclcpp::Node{
    public:
    ControlSelector() : Node("control_selector"){

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "base_scan", 10, std::bind(&ControlSelector::laserCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "base_pose_ground_truth", 10, std::bind(&ControlSelector::poseCallback, this, std::placeholders::_1));
        
        selector_sub_  = this->create_subscription<std_msgs::msg::UInt16>(
            "/selector", 10, std::bind(&ControlSelector::selectCallback, this, std::placeholders::_1));
            

        timer_ = this->create_wall_timer(
            500ms, std::bind(&ControlSelector::timerCallback, this));
        // err_pub_ = this->create_publisher<std_msgs::msg::Float64>("err", 10);

        controls[DUMMY] = new DummyControl();
        controls[VOYAGER] = new VoyagerControl(this->declare_parameter("min_range", 1.0),
                                                this->declare_parameter("max_vel", 0.5),
                                                this->declare_parameter("max_omega", 0.5));

        controls[WALLFOLLOWER] = new WallControl(this->declare_parameter("range", 1.0),
                                                this->declare_parameter("task_vel", 1.0),
                                                this->declare_parameter("min_range_obstacle", 0.5),
                                                this->declare_parameter("prop_factor", 5.0),
                                                this->declare_parameter("int_factor", 0.001),
                                                this->declare_parameter("diff_factor", 14));
                            
        // controls[WALLFOLLOWER] = new WallControl();
        //устанавливаем указатель на действующий алгоритм управления на любой (например первый) элемент массива
        controlPtr = controls[VOYAGER];
    }

    ~ControlSelector(){
        for (std::size_t i = 0; i < nControls; ++i)
        {
            delete controls[i];
        }
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        if ( controlPtr ) // если указатель не нулевой - вызываем функцию текущего алгоритма
            controlPtr->setLaserData(msg->ranges);
    }

    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        // обновляем переменные класса, отвечающие за положение робота
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        theta = 2*atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        if ( controlPtr ) // если указатель не нулевой - вызываем функцию текущего алгоритма
            controlPtr->setRobotPose(x, y, theta);
    }

    void timerCallback(/*const rclcpp::TimerBase::SharedPtr*/){
        geometry_msgs::msg::Twist cmd;
        if ( controlPtr )
        {
            controlPtr->getControl(cmd.linear.x, cmd.angular.z);
        }
        else {
            // это сообщение будет печататься не чаще 1 раза в секунду
            // ROS_INFO_STREAM_THROTTLE(1.0, "no control");
            RCLCPP_INFO(this->get_logger(), "no control");
        }
        // ROS_DEBUG_STREAM("cmd v = "<<cmd.linear.x<<" "<<cmd.angular.z);
        RCLCPP_DEBUG(this->get_logger(), "cmd v = %d cmd z = %d", cmd.linear.x, cmd.angular.z);
        // отправляем (публикуем) команду
        publisher_->publish(cmd);

    }


    // функция обработки сообщения по топику '/selector`, управляющего выбором алгоритма
    // можно отправить сообщение из консоли
    // rostopic pub /selector std_msgs/UInt16 <номер алгоритма начиная с 0>
    void selectCallback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
        // ROS_INFO_STREAM("select callback "<<msg->data);      
        RCLCPP_DEBUG(this->get_logger(),"select callback ", msg->data);

        if ( msg->data >= nControls)
        {
            // ROS_ERROR_STREAM("Wrong algorithm number " << msg->data);
            RCLCPP_WARN(this->get_logger(),"Wrong algorithm number ", msg->data);
            controlPtr = nullptr;
        }
        else
        {
            controlPtr = controls[msg->data];
            // ROS_INFO_STREAM("Select " << controlPtr->getName() << " control");
            RCLCPP_DEBUG(this->get_logger(),"Select %s control", controlPtr->getName());
        }
    }



    private:
    // заданная координата линии, вдоль которой должен двигаться робот
    double line_y;
    double cx, cy, R;
    // заданная скорость движения
    double task_vel;
    // пропрциональный коэффициент регулятора обратной связи
    double prop_factor;
    // интегральный коэффициент регулятора
    double int_factor;
    // дифференциальный коэффициент регулятора
    double diff_factor;
    // интеграл ошибки
    double int_error;
    // старое значение ошибки
    double old_error;
    // минимально допустимое значение расстояния до препятствия
    double min_obstacle_range;
    // флаг наличия препятствия
    bool obstacle;
    // положение робота
    double x, y, theta;

    enum ControlEnum
    {
        DUMMY,
        VOYAGER,
        WALLFOLLOWER, // раскомментировать после добавления нового алгоритма
        nControls
    };

    Control* controlPtr = nullptr;
    Control* controls[nControls];

    // публикатор команд управления
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    // подписчики
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr selector_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlSelector>());
    rclcpp::shutdown();
    return 0;
}