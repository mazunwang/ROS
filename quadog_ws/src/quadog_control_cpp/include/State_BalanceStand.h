#ifndef _STATE_BALANCESTAND_H_
#define _STATE_BALANCESTAND_H_

#include"State_Base.h"
#include<qpOASES.hpp>
// #include"libqpOASES"
#include"StateEstimatorContainer.h"
#include"BalanceController.h"
#include<vector>



class State_BalanceStand:public State_Base
{

    
public:
    State_BalanceStand(LegController* _leg,ControlParameters* _qua,StateEstimatorContainer* container):State_Base(_leg,_qua,container),
        Leg_Pos_Vector(4),
        Leg_Pos_World_Vector(4),
        Leg_Vel_Vector(4),
        Leg_Vel_World_Vector(4),
        Leg_Pos_Init(4),
        Leg_Pos_Init_World(4),
        Des_Joint_Vel(4),
        Des_Joint_Pos(4),
        Leg_Force(4),
        Leg_Torq(4)
    {
        _legController = _leg;
        _quaCP = _qua;
        _container = container;

        body_height_total = 0.0;
        body_velo_total = 0.0;
        x_pos=0.0;
        y_pos=0.0;
        x_vel=0.0;
        y_vel=0.0;

        minFootForce<<0.0,0.0,0.0,0.0;
        maxFootForce<<660.0,660.0,660.0,660.0;

        contact_foot<<1.0,1.0,1.0,1.0;
    }
    ~State_BalanceStand();

    // qpOASES::QProblem qp;

    void onEnter();

    void run();

    void onExit();

    State_Name checkTransition();

    TransitionData getTransitionData();

private:
    int iter;
    double _time;
    double start_time;

    Vec3 FL_Leg_Pos,HL_Leg_Pos,HR_Leg_Pos,FR_Leg_Pos;
    Vec3 FL_Leg_Pos_World,HL_Leg_Pos_World,HR_Leg_Pos_World,FR_Leg_Pos_World;
    Mat43 Leg_Pos_World_Mat;

    std::vector<Vec3> Leg_Pos_Init;
    std::vector<Vec3> Leg_Pos_Init_World;
    // std::vector<Vec3> Leg_Vel_Init;
    // std::vector<Vec3> Leg_Vel_Init_World;
    std::vector<Vec3> Leg_Pos_Vector;
    std::vector<Vec3> Leg_Pos_World_Vector;
    std::vector<Vec3> Leg_Vel_Vector;
    std::vector<Vec3> Leg_Vel_World_Vector;

    Mat3 body_matrix;

    Vec3 RPY,RPY_Init;
    Vec3 OmigaWorld;

    double x_set,y_set,h_set;
    double body_height,body_velo,body_height_total,body_velo_total;

    double x_pos,y_pos;
    double x_vel,y_vel;

    Vec4 minFootForce;
    Vec4 maxFootForce;

    Vec6 F_Vec;
    Vec4 contact_foot;

    std::vector<Vec3> Des_Joint_Pos;
    std::vector<Vec3> Des_Joint_Vel;

    std::vector<Vec3> Leg_Force;
    std::vector<Vec3> Leg_Torq;

    double fOpt[12];
};




#endif