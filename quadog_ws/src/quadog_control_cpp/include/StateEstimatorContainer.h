#ifndef _STATEESTIMATORCONTAINER_H_
#define _STATEESTIMATORCONTAINER_H_

#include"OrientationEstimator.h"

class StateEstimatorContainer
{
private:




public:
    StateEstimatorContainer(StateEstimatorData* _data){
        _stateEstimatorData = _data;
        _oriEsti = new OrientationEstimator();
        _oriEsti->_stateEstimatorData = _data;
        _oriEsti->setup();
        cout<<"StateEstimatorContainer Init Finished"<<endl;
    }
    ~StateEstimatorContainer(){
        // cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        // delete _oriEsti;
        // delete _stateEstimatorData;
    }
    void run();

    OrientationEstimator* _oriEsti;
    StateEstimatorData* _stateEstimatorData;

    // void messageCallback(const sensor_msgs::Imu::ConstPtr& msg){
    //     this->_stateEstimatorData->result->orientation << msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w;
    //     this->_stateEstimatorData->result->BodyOmiga << msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z;
    //     this->_stateEstimatorData->result->BodyAcc << msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
    // // cout<<iter++<<endl;
    // }

};





#endif