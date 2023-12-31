/**
 * @file nav_sl.h
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef NAV_SL_H_
#define NAV_SL_H_

#include "nav_base.h"

namespace nav{
    class NavigationStraightLine : public NavigationBase
    {
    private:
        PlanState plan_state_;
        bool inital_pose_flag_;
        bool target_pose_flag_;
        num_type cur_pos_x_, cur_pos_y_, cur_theta_;
        num_type goal_pos_x_, goal_pos_y_, goal_theta_;
        num_type cmd_vel_x_, cmd_vel_y_, cmd_omg_z_;
        int motion_state_;

        void SetVelocityZero();
        bool StraightLinePlanProcessing();

        int timer_fd_;


    public:
        NavigationStraightLine(const std::string& name);
        ~NavigationStraightLine();

        virtual void SetCurrentPos(const Vec3& pos, const Vec3& rpy, const Vec3& vel, const Vec3& omg);
        virtual void SetTargetPos(const Vec3& pos, const Vec3& rpy);
        virtual void Plan();
        virtual PlanState GetPlanState();
    };
    

    
};

#endif