#include "wall_control.h"


void WallControl::setLaserData(const std::vector<float> &data)
{
    obstacle = false;
    float cur_range = data[0];
    for (size_t i = 0; i<data.size(); i++)
    {
        if(data[i] < cur_range){
            cur_range = data[i];
        }
        if ( data[i] < min_range_obstacle )
        {
            obstacle = true;
            // ROS_WARN_STREAM("OBSTACLE!!!");
            break;
        }
    }
    err = range - cur_range;
}

//получение управления
void WallControl::getControl(double& v, double& w)
{
    if (obstacle)
    {
        v = 0;
        w = 0.5;
    }
    else
    {
        int_error += err;
        //  диффференцируем ошибку
        double diff_error = err - old_error;
        //   запоминаем значение ошибки для следующего момента времени
        old_error = err;
        v = task_vel;
        //  ПИД регулятор угловой скорости w = k*err + k_и * инт_err + k_д * дифф_err
        w = prop_factor * err + int_factor*int_error + diff_error * diff_factor;
    }
}
