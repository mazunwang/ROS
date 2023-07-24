/**
 * @file nav_mpc.cpp
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "nav_mpc.h"

using namespace nav;
using namespace math;

NavigationMPC::NavigationMPC(const std::string& name):NavigationBase(name){

}

NavigationMPC::~NavigationMPC(){
}

void NavigationMPC::SetCurrentPos(const Vec3& pos, const Vec3& rpy, const Vec3& vel, const Vec3& omg){

}

void NavigationMPC::SetTargetPos(const Vec3& pos, const Vec3& rpy){

}

void NavigationMPC::Plan(){


}

PlanState NavigationMPC::GetPlanState(){

}