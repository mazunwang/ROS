#ifndef _STATE_SWING_H_
#define _STATE_SWING_H_


#include"State_Base.h"

class State_Swing:public State_Base{
public:
    State_Swing(LegController* _leg,ControlParameters* _qua,StateEstimatorContainer* container):State_Base(_leg,_qua,container),
    Leg_Pos_Init(4),
    Leg_Pos_Init_World(4),
    Leg_Vel_Init(4),
    Leg_Vel_Init_World(4),
    Leg_Pos_Vector(4),
    Leg_Pos_World_Vector(4),
    Leg_Vel_Vector(4),
    Leg_Vel_World_Vector(4),
    legNum(0),
    Leg_Force(4),
    Leg_Torq(4)
    {
        _legController = _leg;
        _quaCP = _qua;
        _container = container;
        dt = 0.001;
    }
    ~State_Swing(){}

    void onEnter();

    void run();

    void onExit();

    State_Name checkTransition();

    TransitionData getTransitionData();


private:
    int iter;
    Mat3 body_matrix;

    double start_time;
    double _time;
    double fOpt[12];

    Mat43 Leg_Pos_World_Mat;
    Vec6 F_Vec;

    Vec3 RPY,RPY_Init;
    Vec3 OmigaWorld;
 
    int legNum;

    double dt;

    int curLegNum;
    int refLegNum;

    void getXYInitState();
    void getXYCurState();

    std::vector<Vec3> Leg_Pos_Init;
    std::vector<Vec3> Leg_Pos_Init_World;
    std::vector<Vec3> Leg_Vel_Init;
    std::vector<Vec3> Leg_Vel_Init_World;
    std::vector<Vec3> Leg_Pos_Vector;
    std::vector<Vec3> Leg_Pos_World_Vector;
    std::vector<Vec3> Leg_Vel_Vector;
    std::vector<Vec3> Leg_Vel_World_Vector;

    double h_set;
    double x_set;
    double y_set;

    double h_pos_Init;
    double x_pos_Init;
    double y_pos_Init;

    double h_vel_Init;
    double x_vel_Init;
    double y_vel_Init;

    double x_pos;
    double y_pos;
    double h_pos;

    double x_vel;
    double y_vel;
    double h_vel;

    double x_des_pos;
    double x_des_vel;
    double y_des_pos;
    double y_des_vel;
    double h_des_pos;
    double h_des_vel;

    std::vector<Vec3> Leg_Force;
    std::vector<Vec3> Leg_Torq;

    Vec4 contact_foot;

    double alpha;

    Vec3 HipY_Pos,HipY_Pos_World;
    Vec3 HipX_Pos,HipX_Pos_World;

    Vec3 Swing_Leg_Pos_Init,Swing_Leg_Vel_Init;

    Vec3 Target_Pos,Target_Vel;

    Vec3 Swing_Leg_Des_Pos,Swing_Leg_Des_Vel;

    Vec3 curPos,nextPos;
    Vec3 desVel;

    Vec3 curJointPos,nextJointPos;
    Vec3 desJointVel;

    Mat3 SwingLeg_Jacobi;
    Vec3 SwingLeg_Torq;
    Vec3 SwingLeg_Force;
};

#endif