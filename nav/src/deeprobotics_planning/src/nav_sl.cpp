/**
 * @file nav_sl.cpp
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "nav_sl.h"

using namespace nav;
using namespace basicfunction;

NavigationStraightLine::NavigationStraightLine(const std::string& name):NavigationBase(name){
    inital_pose_flag_ = false;
    target_pose_flag_ = false;
    motion_state_ = 0;

    timer_fd_ = timerfd_create(CLOCK_MONOTONIC, 0);
    if (timer_fd_ == -1) {
        perror("timerfd_create");
        exit(EXIT_FAILURE);
    }

    struct itimerspec timer_spec;
    timer_spec.it_interval.tv_sec = 0;
    timer_spec.it_interval.tv_nsec = 1e8;
    timer_spec.it_value.tv_sec = 1;
    timer_spec.it_value.tv_nsec = 0;

    if (timerfd_settime(timer_fd_, 0, &timer_spec, NULL) == -1) {
        perror("timerfd_settime");
        close(timer_fd_);
        exit(EXIT_FAILURE);
    }
}

NavigationStraightLine::~NavigationStraightLine(){
}

void NavigationStraightLine::SetCurrentPos(const Vec3& pos, const Vec3& rpy, const Vec3& vel, const Vec3& omg){
    inital_pose_flag_ = true;
    cur_pos_x_ = pos[0];
    cur_pos_y_ = pos[1];
    cur_theta_ = rpy[2];
}

void NavigationStraightLine::SetTargetPos(const Vec3& pos, const Vec3& rpy){
    target_pose_flag_ = true;
    goal_pos_x_ = pos[0];
    goal_pos_y_ = pos[1];
    goal_theta_ = rpy[2];
}

void NavigationStraightLine::Plan(){
    while (true) {

        uint64_t expirations;
        if (read(timer_fd_, &expirations, sizeof(expirations)) == -1) {
            perror("read");
            exit(EXIT_FAILURE);
        }

        // 在这里执行你想要循环执行的代码
        std::cout << "Timer expired" << std::endl;
        // sleep(10);
    }

}

PlanState NavigationStraightLine::GetPlanState(){
    return plan_state_;
}

bool NavigationStraightLine::StraightLinePlanProcessing(){
    double dx = goal_pos_x_ - cur_pos_x_;
    double dy = goal_pos_y_ - cur_pos_y_;
    double delta_theta_goal = goal_theta_ - cur_theta_;
    // if(delta_theta_goal > M_PI){//与目标的角度偏差
    //     delta_theta_goal -= M_PI * 2.;
    // }else if(delta_theta_goal < -M_PI){
    //     delta_theta_goal += M_PI* 2.;
    // }
    LimitAngle(delta_theta_goal);
//   if(std::sqrt(dx*dx + dy*dy) > 0.5 || fabs(delta_theta_goal) > M_PI / 4.){
//     is_enter_normal_mode = false;
//   }
//   if(is_enter_normal_mode==false){
    double angle_line = 0.0;//直线在全局地图的角度
    double delta_theta_line = 0.0;//与直线的角度偏差
    double distance_to_goal = std::sqrt(dx*dx + dy*dy);
    double forward_scale = 1.0;
    if(nav_mode_==1){
        angle_line = atan2(-dy, -dx);
        forward_scale = -1.0;
    }else{
        angle_line = atan2(dy, dx);
        forward_scale = 1.0;
    }
    delta_theta_line = angle_line - cur_theta_;
    // if(delta_theta_line > M_PI){
    //     delta_theta_line -= M_PI * 2.;
    // }else if(delta_theta_line < -M_PI){
    //     delta_theta_line += M_PI* 2.;
    // }
    LimitAngle(delta_theta_line);
    switch (motion_state_){
        case 0:{//initial state
            SetVelocityZero();
            motion_state_ = 1;
        }
        break;

        case 1:{//初始原地转圈
            cmd_vel_x_ = 0.0;
            cmd_vel_y_ = 0.0;
            if(distance_to_goal < 0.2){
                motion_state_ = 3;
                cmd_omg_z_ = 0.;
                break;
            }
            cmd_omg_z_ = 1.0*(delta_theta_line);
            double sign_num = 1.;
            if(delta_theta_line < 0) sign_num = -1.;
            if(fabs(cmd_omg_z_) < 0.1) cmd_omg_z_ = sign_num * 0.1; 
            if(fabs(delta_theta_line) < 0.08){
                motion_state_ = 2;
                cmd_omg_z_ = 0.;
            }
        }
        break;

        case 2:{//直线靠近
            cmd_vel_y_ = 0.;
            cmd_omg_z_ = 0.;
            double theta = cur_theta_;
            double delta_x = std::cos(theta) * dx + std::sin(theta) * dy;
            double delta_y = std::sin(theta) * dx - std::cos(theta) * dy;
            if(fabs(delta_theta_line) > 0.2 && distance_to_goal > 0.4) {
                cmd_vel_x_ = 0.0;
                motion_state_ = 1;
                break;
            }
            // cmd_omg_z_ = 1.0*(delta_theta_line);
            if(distance_to_goal < 0.2){
                // double x_sign = (delta_x == 0)? 0 : ((delta_x < 0)? -1 : 1);
                // double y_sign = (delta_y == 0)? 0 : ((delta_y < 0)? -1 : 1);
                cmd_vel_x_ = forward_scale*0.50*delta_x;
                cmd_vel_y_ = forward_scale*0.35*delta_y;
                cmd_omg_z_ = 0.0;
                // ROS_WARN_STREAM("delta_x : " << delta_x << "  delta_y : " << delta_y);
            }else if(std::sqrt(dx*dx + dy*dy) < 3.0){
                cmd_vel_x_ = forward_scale*0.6*(delta_x);
                cmd_omg_z_ = 1.0*(delta_theta_line);
            }else{ 
                cmd_vel_x_ = forward_scale*0.6*(delta_x);
                cmd_omg_z_ = 1.0*(delta_theta_line);
            }

            if(distance_to_goal < 0.1){
                SetVelocityZero();
                motion_state_ = 3;
            }
        }
        break;

        case 3:{//结束点转圈
            cmd_vel_x_ = 0.0;
            cmd_vel_y_ = 0.0;
            if(distance_to_goal > 0.6){
                motion_state_ = 2;
                cmd_omg_z_ = 0;
                break;
            }
            cmd_omg_z_ = 0.6*delta_theta_goal;
            // if()
            if(fabs(delta_theta_goal) < 0.1){
                cmd_omg_z_ = 0;
                motion_state_ = 4;
            }
        }
        break;

        case 4:{
        //   ROS_ERROR("NORMAL mode");
        //   is_enter_normal_mode = true;
            motion_state_ = 0;
        }
        break;
    
        default:
            break;
    }
}

void NavigationStraightLine::SetVelocityZero(){
    cmd_vel_x_ = 0;
    cmd_vel_y_ = 0;
    cmd_omg_z_ = 0;
}