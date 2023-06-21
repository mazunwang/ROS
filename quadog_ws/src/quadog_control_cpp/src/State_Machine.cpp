#include"State_Machine.h"

State_Machine::State_Machine(Quadruped* quad,LegController* _leg,ControlParameters* cp,StateEstimatorContainer* container){
    _legController = _leg;
    _quadog = quad;
    _quaCP = cp;
    _container = container;

    statelist.invalid = nullptr;
    statelist.init = new State_Init(_legController,cp,container);
    statelist.jointPD = new State_JointPD(_legController,cp,container);
    statelist.standUp = new State_StandUp(_legController,cp,container);
    statelist.balanceStand = new State_BalanceStand(_legController,cp,container);
    statelist.cOM_Move = new State_COM_Move(_legController,cp,container);
    statelist.unloadForce = new State_UnloadForce(_legController,cp,container);
    statelist.swing = new State_Swing(_legController,cp,container);
    statelist.loadForce = new State_LoadForce(_legController,cp,container);


    initialize();
}

void State_Machine::initialize(){
    current_state = statelist.init;
    current_state_name = State_Name::INIT;
    current_state->onEnter();
    next_state = current_state;
}

void State_Machine::State_Machine_run(){
    next_state_name = current_state->checkTransition();
    if(next_state_name==current_state_name) current_state->run();
    else{
        current_state->onExit();
        next_state = getNextState(next_state_name);
        next_state->onEnter();
        next_state->run();
        current_state = next_state;
        current_state_name = next_state_name;
    }
}

State_Base* State_Machine::getNextState(State_Name statename){
    switch(statename){
        case State_Name::INVALID:
            return statelist.invalid;
        case State_Name::INIT:
            return statelist.init;
        case State_Name::JOINT_PD:
            return statelist.jointPD;
        case State_Name::STAND_UP:
            return statelist.standUp;
        case State_Name::BALANCE_STAND:
            return statelist.balanceStand;
        case State_Name::COM_MOVE:
            return statelist.cOM_Move;
        case State_Name::UNLOADFORCE:
            return statelist.unloadForce;
        case State_Name::SWING:
            return statelist.swing;
        case State_Name::LOADFORCE:
            return statelist.loadForce;
        default:{
            assert(0);
            return statelist.invalid;
        }

    }
    return statelist.invalid;
}