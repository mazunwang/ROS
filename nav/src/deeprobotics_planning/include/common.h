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
#include <vector>

typedef float num_type;

enum PlanState{
    kIdle = 0,
    kStart,
    kPlanning,
    kError,
    kReachGoal,
};

typedef Eigen::Matrix<num_type, 3, 1> Vec3;
typedef Eigen::Matrix<num_type, 4, 1> Vec4;
typedef Eigen::Matrix<num_type, 3, 3> Mat3;


struct Point2D{
    num_type x, y;
};

#endif