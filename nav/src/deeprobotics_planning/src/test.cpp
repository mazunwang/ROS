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

using namespace basicfunction;

int main(){
    std::shared_ptr<nav::NavigationBase> nav_ptr = std::make_shared<nav::NavigationStraightLine>("Nav");
    // Vec4 quat(0.99998594901, -0.00521880544114, -0.000917173203543, 0.00015698994999);
    // std::cout << QuatToRpy(quat) << std::endl;
}