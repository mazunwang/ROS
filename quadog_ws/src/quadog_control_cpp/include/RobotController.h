#ifndef _ROBOT_CONTROLLER_H_
#define _ROBOT_CONTROLLER_H_

#include "LegController.h"
#include"ControlParameters.h"

class RobotController{
    public:
        RobotController(){}
        virtual ~RobotController(){}
        virtual void initializeController() = 0;

        virtual void runController() = 0;

    protected:
        Quadruped* quadog;
        LegController* _legController;
        ControlParameters* quaCP;

};

#endif