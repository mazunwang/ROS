/**
 * @file nav_base.h
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef NAV_BASE_H_
#define NAV_BASE_H_

#include"common.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

namespace base{
    class NavigationBase
    {
    private:
        /* data */
    public:
        NavigationBase(const std::string& name):nav_name_(name){};
        virtual ~NavigationBase(){};

        std::string nav_name_;
        int nav_mode_;
        // PlanState plan_state_;

        virtual void SetCurrentPos(const Vec3& pos, const Vec3& rpy, const Vec3& vel, const Vec3& omg) = 0;
        virtual void SetTargetPos(const Vec3& pos, const Vec3& rpy) = 0;
        virtual void Plan() = 0;
        virtual PlanState GetPlanState() = 0;
    };
    

    
};


#endif
