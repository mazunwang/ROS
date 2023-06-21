/**
 * @file nav_interface.cpp
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "nav_interface.h"

using namespace interface;

NavgationInterface::NavgationInterface(/* args */){
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base",
                                    boost::bind(&MoveBase::executeCb, this, _1),
                                    false);
}

NavgationInterface::~NavgationInterface(){
}