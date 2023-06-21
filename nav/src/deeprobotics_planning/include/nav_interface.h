/**
 * @file nav_interface.h
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef NAV_INTERFACE_H_
#define NAV_INTERFACE_H_


#include "common.h"
#include <actionlib/server/simple_action_server.h>
#include "nav/NavigationAction.h"
#include "nav_base.h"
#include "nav_sl.h"

namespace interface{
    typedef actionlib::SimpleActionServer<nav::NavigationAction> NavigationActionServer;

    class NavgationInterface{
    private:
        NavigationActionServer* as_;
        std::shared_ptr<nav::NavigationBase> nav_ptr_;
        ros::Subscriber odom_sub_;

    public:
        NavgationInterface();
        ~NavgationInterface();

        void NavigationStraightLineCb(const nav::NavigationGoalConstPtr& goal);

        void OdometryCallback(nav_msgs::OdometryConstPtr msg);
    };
    

    
};

#endif