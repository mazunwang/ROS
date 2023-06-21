#include"OrientationEstimator.h"


void OrientationEstimator::run(){
    if(iter<5) iter++;
    else{
        iter = 0;
        double x = this->_stateEstimatorData->result->orientation[0];
        double y = this->_stateEstimatorData->result->orientation[1];
        double z = this->_stateEstimatorData->result->orientation[2];
        double w = this->_stateEstimatorData->result->orientation[3];
        this->_stateEstimatorData->result->rpy<<atan2(2 * (w * x + y * z), 1 - 2 * (x*x + y*y)), 
                                                asin(2 * (w * y - z * x)),
                                                atan2(2 * (w * z + x * y), 1 - 2 * (y*y + z*z));
        this->_stateEstimatorData->result->Body_Matrix = RPYToMat(this->_stateEstimatorData->result->rpy);
        this->_stateEstimatorData->result->BodyAccWorld = this->_stateEstimatorData->result->Body_Matrix*this->_stateEstimatorData->result->BodyAcc;
        this->_stateEstimatorData->result->BodyOmigaWorld = this->_stateEstimatorData->result->Body_Matrix*this->_stateEstimatorData->result->BodyOmiga;
        // cout<<this->_stateEstimatorData->result->Body_Matrix<<endl;
        // cout<<this->_stateEstimatorData->result->BodyAccWorld.transpose()<<endl;
        // cout<<this->_stateEstimatorData->result->rpy.transpose()<<endl;
    }

}

void OrientationEstimator::setup(){
    // cout<<"IMU Start"<<endl;
}

Mat3 OrientationEstimator::RPYToMat(Vec3 rpy){
    double r = rpy[0];
    double p = rpy[1];
    double y = rpy[2];
    Mat3 RotX,RotY,RotZ;
    RotX<<1.0,0.0,0.0,0.0,cos(r),-sin(r),0.0,sin(r),cos(r);
    RotY<<cos(p),0.0,sin(p),0.0,1.0,0.0,-sin(p),0.0,cos(p);
    RotZ<<cos(y),-sin(y),0.0,sin(y),cos(y),0.0,0.0,0.0,1.0;
    return RotZ*RotY*RotX;
}

