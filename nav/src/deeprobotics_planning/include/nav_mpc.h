/**
 * @file nav_mpc.h
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef NAV_MPC_H_
#define NAV_MPC_H_

#include "nav_base.h"

namespace nav{
    class NavigationMPC : public NavigationBase
    {
    private:


    public:
        NavigationMPC(const std::string& name);
        ~NavigationMPC();

        virtual void SetCurrentPos(const Vec3& pos, const Vec3& rpy, const Vec3& vel, const Vec3& omg);
        virtual void SetTargetPos(const Vec3& pos, const Vec3& rpy);
        virtual void Plan();
        virtual PlanState GetPlanState();
    };
    

    
};

#endif