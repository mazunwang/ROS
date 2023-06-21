#ifndef _STATE_JOINTPD_H_
#define _STATE_JOINTPD_H_

#include"State_Base.h"

class State_JointPD:public State_Base{
public:
    State_JointPD(LegController* _leg,ControlParameters* _qua,StateEstimatorContainer* container):State_Base(_leg,_qua,container){
        _legController = _leg;
        _quaCP = _qua;
        _container = container;
    }

    void onEnter();

    void run();

    void onExit();

    State_Name checkTransition();

    TransitionData getTransitionData();


private:

    int iter;
    Vec3 FL_Joint_Target_Pos,HL_Joint_Target_Pos,HR_Joint_Target_Pos,FR_Joint_Target_Pos;
    Vec3 FL_Joint_Target_Vel,HL_Joint_Target_Vel,HR_Joint_Target_Vel,FR_Joint_Target_Vel;

    Vec3 FL_Joint_Pos,HL_Joint_Pos,HR_Joint_Pos,FR_Joint_Pos;
    Vec3 FL_Joint_Vel,HL_Joint_Vel,HR_Joint_Vel,FR_Joint_Vel;

    Vec3 FL_Joint_Pos_Des,HL_Joint_Pos_Des,HR_Joint_Pos_Des,FR_Joint_Pos_Des;
    Vec3 FL_Joint_Vel_Des,HL_Joint_Vel_Des,HR_Joint_Vel_Des,FR_Joint_Vel_Des;
    double _time;
};

#endif
