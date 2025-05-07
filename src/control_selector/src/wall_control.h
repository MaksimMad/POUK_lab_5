#pragma once

#include "control.h"

class WallControl : public Control
{
private:
    double range;
    bool obstacle = false;
    double min_range_obstacle;

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
    // ошибка
    double err = 0;

public:
    //установка данных лазера
    void setLaserData(const std::vector<float>& data) override;

    //установка текущей позиции робота - для данного вида управления не требуется - поэтому пустая
    void setRobotPose(double x, double y, double theta) override {}

    //получение управления
    void getControl(double& v, double& w) override;

    std::string getName() override { return "WallControl"; }
    
    WallControl(double range = 1.0, double task_vel = 1, double min_range_obstacle = 0.5, double prop_factor = 5.0, double int_factor = 0.001, double diff_factor = 14):
        range(range),
        task_vel(task_vel),
        min_range_obstacle(min_range_obstacle),
        prop_factor(prop_factor),
        int_factor(int_factor),
        diff_factor(diff_factor)
    {
        // ROS_DEBUG_STREAM("VoyagerControl constructor");
    }
};

