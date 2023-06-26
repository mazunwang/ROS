/**
 * @file test.cpp
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "nav_base.h"
#include "nav_sl.h"
// #include "parameters/parameters_config.hpp"

using namespace basicfunction;

int main(){
    // std::shared_ptr<nav::NavigationBase> nav_ptr = std::make_shared<nav::NavigationStraightLine>("Nav");
    // Vec4 quat(0.99998594901, -0.00521880544114, -0.000917173203543, 0.00015698994999);
    // std::cout << QuatToRpy(quat) << std::endl;

    parameter::ParametersConfig config;

    config.AddParameter<double>("max_vel_x", 0.5, "max forward velocity", 0.2, 2.5);
    std::cout << "value:  " << config.GetParameter<double>("max_vel_x") << std::endl;

    // 设置参数
    // config.setParameter("max_speed", "100.5");
    // config.setParameter("min_distance", "10.5");
    // config.setParameter("is_enabled", "true");

    // // 获取参数
    // int maxSpeed = config.getParameter<int>("max_speed");
    // double minDistance = config.getParameter<double>("min_distance");
    // bool isEnabled = config.getParameter<bool>("is_enabled");

    // std::cout << "Type:  " << config.GetParameterType<bool>() << std::endl;

    // std::cout << "Max Speed: " << maxSpeed << std::endl;
    // std::cout << "Min Distance: " << minDistance << std::endl;
    // std::cout << "Is Enabled: " << std::boolalpha << isEnabled << std::endl;

    return 0;
}