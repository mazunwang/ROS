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

#include "common.h"
#include "basic_function.h"
#include "parameters_config.hpp"


namespace nav{
    class NavigationBase
    {
    private:
        /* data */
    public:
        NavigationBase(const std::string& name):nav_name_(name){};
        virtual ~NavigationBase(){};

        std::string nav_name_;
        int nav_mode_;
        static parameter::ParametersConfig* para_cfg_;

        virtual void SetCurrentPos(const Vec3& pos, const Vec3& rpy, const Vec3& vel, const Vec3& omg) = 0;
        virtual void SetTargetPos(const Vec3& pos, const Vec3& rpy) = 0;
        virtual void Plan() = 0;
        virtual PlanState GetPlanState() = 0;
    };
    

    
};


#endif
