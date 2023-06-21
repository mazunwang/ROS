#ifndef _TYPES_H_
#define _TYPES_H_

#include <eigen3/Eigen/Dense>
#include <vector>
#include <assert.h>

using Mat3 = Eigen::Matrix<double,3,3>;

using Vec3 = Eigen::Matrix<double,3,1>;

using Vec4 = Eigen::Matrix<double,4,1>;

using Mat4 = Eigen::Matrix<double,4,4>;

using Mat43 = Eigen::Matrix<double,4,3>;

using Mat34 = Eigen::Matrix<double,3,4>;

using Vec12 = Eigen::Matrix<double,12,1>;

using Vec6 = Eigen::Matrix<double,6,1>;

using Mat6 = Eigen::Matrix<double,6,6>;



#endif
