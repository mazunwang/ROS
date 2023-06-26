/**
 * @file common.h
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <iostream>
#include <iomanip>
#include <string>
#include <mutex>
#include <Eigen/Dense>
#include <stdio.h>
#include <stdlib.h>
#include <sys/timerfd.h>
#include <time.h>
#include<sys/time.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <assert.h>
#include "toml.hpp"


enum PlanState{
    kIdle = 0,
    kStart,
    kPlanning,
    kError,
    kReachGoal,
};

typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Matrix3d Mat3;

#endif