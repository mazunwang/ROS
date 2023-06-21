#include"QuaDogController.h"


QuaDogController::QuaDogController(Quadruped* quad, LegController* _leg,ControlParameters* qua,StateEstimatorData* _data){
    quadog = quad;
    _legController =  _leg;
    quaCP = qua;
    this->_container = new StateEstimatorContainer(_data);
    StateControl = new State_Machine(quad,_leg,qua,_container);
    std::cout<<"---------QuaDogController Init Finished----------"<<std::endl;
}

void QuaDogController::initializeController(){

}

void QuaDogController::runController(){
    StateControl->State_Machine_run();
    _container->run();
}

