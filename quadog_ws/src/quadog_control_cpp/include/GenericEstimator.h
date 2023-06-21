#ifndef _GENERICESTIMATOR_H_
#define _GENERICESTIMATOR_H_

#include "Types.h"
#include <sensor_msgs/Imu.h>
#include"LegController.h"

struct StateEstimate{
// public:
    Vec3 BodyVel;
    Vec3 rpy;
    Mat3 Body_Matrix;
    Vec4 orientation;
    Vec3 BodyAcc;
    Vec3 BodyOmiga;
    Vec3 BodyAccWorld;
    Vec3 BodyOmigaWorld;
    double t;
    // void setZero(){
    //     BodyVel<<0,0,0;
    //     rpy<<0,0,0;
    //     Body_Matrix<<0,0,0,0,0,0,0,0,0;
    //     orientation<<0,0,0,0;
    //     BodyAcc<<0,0,0;
    //     BodyOmiga<<0,0,0;
    //     BodyAccWorld<<0,0,0;
    //     BodyOmigaWorld<<0,0,0;
    // }
};

struct  StateEstimatorData
{
public:
    StateEstimate* result;
    LegController* _legcontrol;
    StateEstimatorData(LegController* _leg,StateEstimate* _res):_legcontrol(_leg),result(_res){
    //     // this->_legcontrol = _leg;
    //     // result->setZero();
    //     // cout<<"there"<<endl;
    //     // iter =0;
    }


// private:
    // int iter;
};


class GenericEstimator {
 public:
  virtual void run() = 0;
  virtual void setup() = 0;

  void setData(StateEstimatorData data) { _stateEstimatorData = &data; }

  virtual ~GenericEstimator() = default;
  StateEstimatorData* _stateEstimatorData;
};


#endif