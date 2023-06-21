#ifndef _STATE_STANDUP_H_
#define _STATE_STANDUP_H_

#include "State_Base.h"

class State_StandUp:public State_Base{
public:
    State_StandUp(LegController* _leg,ControlParameters* _qua,StateEstimatorContainer* container):State_Base(_leg,_qua,container),
        Leg_Pos_Init(4),
        Joint_Vel_Init(4),
        Joint_Pos_Init(4),
        Target_Joint_Pos(4),
        Des_Joint_Vel(4),
        Des_Joint_Pos(4){

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
    double _time;

    std::vector<Vec3> Leg_Pos_Init;
    std::vector<Vec3> Joint_Pos_Init;
    std::vector<Vec3> Joint_Vel_Init;
    std::vector<Vec3> Target_Joint_Pos;
    std::vector<Vec3> Des_Joint_Pos;
    std::vector<Vec3> Des_Joint_Vel;

    // Vec3 FL_Leg_Pos_Init,HL_Leg_Pos_Init,HR_Leg_Pos_Init,FR_Leg_Pos_Init;

    // Vec3 FL_Joint_Target_Pos,HL_Joint_Target_Pos,HR_Joint_Target_Pos,FR_Joint_Target_Pos;
    // Vec3 FL_Joint_Target_Vel,HL_Joint_Target_Vel,HR_Joint_Target_Vel,FR_Joint_Target_Vel;

    // Vec3 FL_Joint_Pos,HL_Joint_Pos,HR_Joint_Pos,FR_Joint_Pos;
    // Vec3 FL_Joint_Vel,HL_Joint_Vel,HR_Joint_Vel,FR_Joint_Vel;

    // Vec3 FL_Joint_Pos_Des,HL_Joint_Pos_Des,HR_Joint_Pos_Des,FR_Joint_Pos_Des;
    // Vec3 FL_Joint_Vel_Des,HL_Joint_Vel_Des,HR_Joint_Vel_Des,FR_Joint_Vel_Des;
};

#endif