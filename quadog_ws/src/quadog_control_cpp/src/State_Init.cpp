#include"State_Init.h"


void State_Init::onEnter(){
    this->iter = 0;
    start_time = _container->_stateEstimatorData->result->t;
    _time = 0.0;
}

void State_Init::run(){
    // cout<<"Init_State"<<endl;
    // for(int leg =0;leg<4;++leg) cout<<_legController->legName[leg]<<":"<<_legController->GetLegPos(leg).transpose()<<" "<<std::endl;;
    // cout<<endl;
    iter++;
    _time = _container->_stateEstimatorData->result->t;
    // cout<<_time<<endl;
    // cout<<iter<<endl;
}

void State_Init::onExit(){
    iter = 0;
    cout<<"INIT FINISHED"<<endl;
}

State_Name State_Init::checkTransition(){
    if(iter>(_quaCP->CONTROL_RATE*(_quaCP->INIT_PERIOD))) return State_Name::JOINT_PD;
    return State_Name::INIT;
}

TransitionData State_Init::getTransitionData(){
    return this->_transitonData;
}