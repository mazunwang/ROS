/**
 * @file basic_function.cpp
 * @author mazunwang (mazunwang@163.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "basic_function.h"

namespace math{

num_type Sign(num_type x) {
    if(fabs(x) < 1e-4) return 0.;
    return x < 0.0 ? -1.0 : 1.0; 
}

void LimitAngle(num_type& angle){
    angle = fmod(angle, 2 * M_PI);
    if (angle > M_PI) angle -= 2 * M_PI;
}

Vec3 QuatToRpy(const Vec4& q){
    Vec3 rpy;
    num_type as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
    rpy(2) = std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    rpy(1) = std::asin(as);
    rpy(0) = std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    return rpy;
}

Mat3 RpyToRotMat(const Vec3& rpy){
    Eigen::AngleAxis<num_type> yawAngle(rpy[2], Vec3 ::UnitZ());
    Eigen::AngleAxis<num_type> pitchAngle(rpy[1], Vec3::UnitY());
    Eigen::AngleAxis<num_type> rollAngle(rpy[0], Vec3::UnitX());
    Eigen::Quaternion<num_type> q = yawAngle * pitchAngle * rollAngle;
    return q.matrix();
}

Vec3 RotMatToRpy(const Mat3& R){
    Vec3 euler =  R.eulerAngles(2, 1, 0);
    return Vec3(euler(2), euler(1), euler(0));
}

Vec4 RpyToQuat(const Vec3& rpy){
    Mat3 R = RpyToRotMat(rpy);
    return RotMatToQuat(R);
}

Mat3 QuatToRotMat(const Vec4& q){
    num_type w = q(0);
    num_type x = q(1), y = q(2), z = q(3);
    Mat3 R;
    R << 1-2*y*y-2*z*z, 2*x*y+2*w*z, 2*x*z-2*w*y,
        2*x*y-2*w*z, 1-2*x*x-2*z*z, 2*y*z+2*w*x,
        2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x*x-2*y*y;
    R.transposeInPlace();
    return R;
}

Vec4 RotMatToQuat(const Mat3& rm){
    Vec4 q;
    num_type tr = rm.trace();
    if (tr > 0.0) {
        num_type S = sqrt(tr + 1.0) * 2.0;
        q(0) = 0.25 * S;
        q(1) = (rm(2, 1) - rm(1, 2)) / S;
        q(2) = (rm(0, 2) - rm(2, 0)) / S;
        q(3) = (rm(1, 0) - rm(0, 1)) / S;
    } else if ((rm(0, 0) > rm(1, 1)) && (rm(0, 0) > rm(2, 2))) {
        num_type S = sqrt(1.0 + rm(0, 0) - rm(1, 1) - rm(2, 2)) * 2.0;
        q(0) = (rm(2, 1) - rm(1, 2)) / S;
        q(1) = 0.25 * S;
        q(2) = (rm(0, 1) + rm(1, 0)) / S;
        q(3) = (rm(0, 2) + rm(2, 0)) / S;
    } else if (rm(1, 1) > rm(2, 2)) {
        num_type S = sqrt(1.0 + rm(1, 1) - rm(0, 0) - rm(2, 2)) * 2.0;
        q(0) = (rm(0, 2) - rm(2, 0)) / S;
        q(1) = (rm(0, 1) + rm(1, 0)) / S;
        q(2) = 0.25 * S;
        q(3) = (rm(1, 2) + rm(2, 1)) / S;
    } else {
        num_type S = sqrt(1.0 + rm(2, 2) - rm(0, 0) - rm(1, 1)) * 2.0;
        q(0) = (rm(1, 0) - rm(0, 1)) / S;
        q(1) = (rm(0, 2) + rm(2, 0)) / S;
        q(2) = (rm(1, 2) + rm(2, 1)) / S;
        q(3) = 0.25 * S;
    }
    return q;
}


bool Intersects(std::vector<Point2D>& polygon, num_type testx, num_type testy){
    bool c = false;
    int i, j, nvert = polygon.size();
    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        num_type yi = polygon[i].y, yj = polygon[j].y, xi = polygon[i].x, xj = polygon[j].x;
        if (((yi > testy) != (yj > testy)) && (testx < (xj - xi) * (testy - yi) / (yj - yi) + xi)) c = !c;
    }
    return c;
}

bool intersects_helper(std::vector<Point2D>& polygon1, std::vector<Point2D>& polygon2) {
    for (unsigned int i = 0; i < polygon1.size(); i++)
        if (Intersects(polygon2, polygon1[i].x, polygon1[i].y)) return true;
    return false;
}

bool Intersects(std::vector<Point2D>& polygon1, std::vector<Point2D>& polygon2) {
    return intersects_helper(polygon1, polygon2) || intersects_helper(polygon2, polygon1);
}

};