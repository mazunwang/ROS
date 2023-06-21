#include"State_StandUp.h"

void State_StandUp::onEnter(){
    // FL_Leg_Pos_Init = _legController->GetLegPos(0);
    // HL_Leg_Pos_Init = _legController->GetLegPos(1);
    // HR_Leg_Pos_Init = _legController->GetLegPos(2);
    // FR_Leg_Pos_Init = _legController->GetLegPos(3);
    for(int leg = 0;leg<4;++leg){
        this->Leg_Pos_Init[leg] = _legController->GetLegPos(leg);
        this->Joint_Pos_Init[leg] = _legController->legData[leg].JointPosition;
        this->Joint_Vel_Init[leg] = _legController->legData[leg].JointVelocity;
        this->Leg_Pos_Init[leg][2] = _quaCP->StandUp_Height;
        this->Target_Joint_Pos[leg] = _legController->LegIKCal(leg,this->Leg_Pos_Init[leg]);
        // cout<<leg<<"     "<<Leg_Pos_Init[leg].transpose()<<endl;
        // cout<<leg<<"   "<<Joint_Pos_Init[leg].transpose()<<endl;
        // cout<<leg<<"   "<<Target_Joint_Pos[leg].transpose()<<endl;
    }
    this->iter = 0;
    cout<<"----------Stand Up----------"<<endl;
}


void State_StandUp::run(){

    iter++;

    _time = (double)iter/_quaCP->CONTROL_RATE;

    for(int leg=0;leg<4;++leg){
        for(int joint = 0;joint<3;++joint){
            this->swingTraj->CubicPolynomialInterpolation(Joint_Pos_Init[leg][joint],Joint_Vel_Init[leg][joint],Target_Joint_Pos[leg][joint],0,_time,_quaCP->STANDUP_PERIOD,Des_Joint_Pos[leg][joint],Des_Joint_Vel[leg][joint]);
        }
        this->JointPDControl(leg,Des_Joint_Pos[leg],Des_Joint_Vel[leg]);

        // cout<<_time<<"  "<<leg<<" "<<Des_Joint_Pos[leg].transpose()<<endl;
    }
}

void State_StandUp::onExit(){
    iter = 0;
}

State_Name State_StandUp::checkTransition(){
    if((double)iter/_quaCP->CONTROL_RATE>=_quaCP->STANDUP_PERIOD) return State_Name::BALANCE_STAND;
    return State_Name::STAND_UP;
}

TransitionData State_StandUp::getTransitionData(){
    return this->_transitonData;
}