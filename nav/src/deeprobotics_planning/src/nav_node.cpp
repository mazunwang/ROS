/**
 * @file nav_node.cpp
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "nav_interface.h"

using namespace interface;

int main(int argc, char** argv){
    ros::init(argc, argv, "nav");
    interface::NavgationInterface ni;
    ros::spin();
}