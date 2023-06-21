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
using namespace basicfunction;

NavgationInterface::NavgationInterface(/* args */){

    as_ = new NavigationActionServer(ros::NodeHandle(), "navigation", boost::bind(&NavgationInterface::NavigationStraightLineCb, this, _1), false);
    nav_ptr_ = std::make_shared<nav::NavigationStraightLine>("nav_sl");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    odom_sub_ = nh.subscribe("/odom", 10, &NavgationInterface::OdometryCallback, this);
    as_->start();
    // ROS_WARN_STREAM("debug!!!!");
}

NavgationInterface::~NavgationInterface(){
}

void NavgationInterface::NavigationStraightLineCb(const nav::NavigationGoalConstPtr& goal){
    Vec3 target_pos(goal->target_pose.pose.position.x, goal->target_pose.pose.position.y, goal->target_pose.pose.position.z);
    Vec4 target_quat(goal->target_pose.pose.orientation.w, goal->target_pose.pose.orientation.x, goal->target_pose.pose.orientation.y, goal->target_pose.pose.orientation.z);
    Vec3 target_rpy = QuatToRpy(target_quat);
    nav_ptr_->SetTargetPos(target_pos, target_rpy);
    nav_ptr_->Plan();
}

void NavgationInterface::OdometryCallback(nav_msgs::OdometryConstPtr msg){
    Vec3 pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Vec4 quat(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Vec3 rpy = QuatToRpy(quat);
    nav_ptr_->SetCurrentPos(pos, rpy, Vec3::Zero(), Vec3::Zero());
}