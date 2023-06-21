#ifndef _QUADOGCONTROLLER_H_
#define _QUADOGCONTROLLER_H_

#include"RobotController.h"
#include"State_Machine.h"
#include"StateEstimatorContainer.h"

class QuaDogController:public RobotController{
public:
    QuaDogController(Quadruped* quad, LegController* _leg,ControlParameters* qua,StateEstimatorData* _data);
    void runController();
    void initializeController();


protected:
    State_Machine* StateControl;
    StateEstimatorContainer* _container;

};


#endif
