#ifndef _STATE_BASE_H_
#define _STATE_BASE_H_

#include "ControlParameters.h"
#include "LegController.h"
#include "SwingTrajectory.h"
#include "BalanceController.h"
#include "StateEstimatorContainer.h"

using namespace std;

enum class State_Name{
    INVALID,
    INIT,
    JOINT_PD,
    STAND_UP,
    BALANCE_STAND,
    COM_MOVE,
    LOADFORCE,
    SWING,
    UNLOADFORCE
};

struct TransitionData{
    Mat34 Leg_Pos;
    Mat34 Leg_Pos_World;
    Mat34 Leg_Vel;
    Mat34 Leg_Vel_World;
};

static int legSeq[4] = {0,1,3,2};

class State_Base{
public:
    State_Base(LegController* leg,ControlParameters* _qua,StateEstimatorContainer* container)/*:_balanceController(BalanceController())*/{
        _legController = leg;
        _quaCP = _qua;
        _container = container;
        // _balanceController = new BalanceController();
        // _balanceController = BalanceController();
        // legSeq[4] = {0,1,2,3};
        for(int i=0;i<4;++i) legMoveNum[i] = legSeq[i];
    }
    virtual void onEnter() = 0;
    virtual void run() = 0;
    virtual State_Name checkTransition(){
        return State_Name::INVALID;
    }
    virtual void onExit() = 0;
    virtual TransitionData getTransitionData(){
        return _transitonData;
    }

    LegController* _legController;
    ControlParameters* _quaCP;
    StateEstimatorContainer* _container;
    BalanceController _balanceController;
    TransitionData _transitonData;

    State_Name current_state;
    State_Name next_state;

    SwingTrajectory* swingTraj;

    int legMoveNum[4];

    void JointPDControl(int leg,Vec3 Des_Joint_Pos,Vec3 Des_Joint_Vel);

    int getRefLegNum(int leg);


    // int legSeq[4];
};

#endif
