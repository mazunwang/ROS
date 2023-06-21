/**
 * @file basic_function.h
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef BASIC_FUNCTION_H_
#define BASIC_FUNCTION_H_

#include "common.h"

namespace basicfunction
{
    void LimitAngle(double& angle);
    Vec3 QuatToRpy(const Vec4& q);//w,x,y,z
    Vec4 RpyToQuat(const Vec3& rpy);//r,p,y
    Mat3 RpyToRotMat(const Vec3& rpy);
    Vec3 RotMatToRpy(const Mat3& R);
    // Vec4 RpyToQuat(const Vec3& rpy);
    Mat3 QuatToRotMat(const Vec4& q);
    Vec4 RotMatToQuat(const Mat3& R);
    // Vec3 QuatToRpy(const Vec4& q);
} // namespace basicfunction


#endif
