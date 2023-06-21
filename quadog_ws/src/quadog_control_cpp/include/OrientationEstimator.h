#ifndef _ORIENTATIONESTIMATOR_H_
#define _ORIENTATIONESTIMATOR_H_

#include"GenericEstimator.h"
#include <sensor_msgs/Imu.h>

class OrientationEstimator:public GenericEstimator{
public:
    OrientationEstimator(){
        // this->_stateEstimatorData = _data;
    }
    ~OrientationEstimator(){
    }

    void run();
    void setup();

    // void messageCallback(const sensor_msgs::Imu::ConstPtr& msg);
private:

    int iter;
    Mat3 RPYToMat(Vec3 rpy);
};


#endif