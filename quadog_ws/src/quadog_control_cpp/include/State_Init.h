#ifndef _STATE_INIT_H_
#define _STATE_INIT_H_

#include "State_Base.h"

class State_Init:public State_Base{
public:
    State_Init(LegController* _leg,ControlParameters* _qua,StateEstimatorContainer* container):State_Base(_leg,_qua,container){
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

    double start_time;
    double _time;


};



#endif