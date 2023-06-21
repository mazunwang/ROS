#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#include"State_Base.h"
#include"State_Init.h"
#include"State_JointPD.h"
#include"State_StandUp.h"
#include"State_BalanceStand.h"
#include"State_COM_Move.h"
#include"State_UnloadForce.h"
#include"State_Swing.h"
#include"State_LoadForce.h"

struct State_Machine_List{
    State_Base* invalid;
    State_Init* init;
    State_JointPD* jointPD;
    State_StandUp* standUp;
    State_BalanceStand* balanceStand;
    State_COM_Move* cOM_Move;
    State_UnloadForce* unloadForce;
    State_Swing* swing;
    State_LoadForce* loadForce;
 };


class State_Machine{
public:
    State_Machine(Quadruped* quad,
                LegController* _leg,
                ControlParameters* cp,
                StateEstimatorContainer* container);
    void initialize();
    void State_Machine_run();
    State_Base* getNextState(State_Name statename);

    State_Machine_List statelist;
    State_Base* current_state;
    State_Base* next_state;
    State_Name next_state_name;
    State_Name current_state_name;

    StateEstimatorContainer* _container;

    TransitionData transtiondata;

    LegController* _legController;
    Quadruped* _quadog;
    ControlParameters* _quaCP;
};


#endif