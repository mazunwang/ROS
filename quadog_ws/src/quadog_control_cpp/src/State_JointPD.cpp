#include "State_JointPD.h"

void State_JointPD::onEnter(){
    this->FL_Joint_Pos = _legController->legData[0].JointPosition;
    this->HL_Joint_Pos = _legController->legData[1].JointPosition;
    this->HR_Joint_Pos = _legController->legData[2].JointPosition;
    this->FR_Joint_Pos = _legController->legData[3].JointPosition;

    this->FL_Joint_Vel = _legController->legData[0].JointVelocity;
    this->HL_Joint_Vel = _legController->legData[1].JointVelocity;   
    this->HR_Joint_Vel = _legController->legData[2].JointVelocity;
    this->FR_Joint_Vel = _legController->legData[3].JointVelocity;

    this->FL_Joint_Target_Pos = _legController->LegIKCal(0,_quaCP->Joint_PD_FL_Leg_Pos);
    this->HL_Joint_Target_Pos = _legController->LegIKCal(1,_quaCP->Joint_PD_HL_Leg_Pos);
    this->HR_Joint_Target_Pos = _legController->LegIKCal(2,_quaCP->Joint_PD_HR_Leg_Pos);
    this->FR_Joint_Target_Pos = _legController->LegIKCal(3,_quaCP->Joint_PD_FR_Leg_Pos);

    cout<<"---------Joint_PD State Started---------"<<endl;
    // cout<<"FL_Joint_Pos"<<this->FL_Joint_Target_Pos.transpose()<<endl;

    // cout<<"FL_Pos:"<<_legController->GetLegPos(0,FL_Joint_Target_Pos).transpose()<<endl;
    // cout<<"HL_Pos:"<<_legController->GetLegPos(1,HL_Joint_Target_Pos).transpose()<<endl;
    // cout<<"HR_Pos:"<<_legController->GetLegPos(2,HR_Joint_Target_Pos).transpose()<<endl;
    // cout<<"FR_Pos:"<<_legController->GetLegPos(3,FR_Joint_Target_Pos).transpose()<<endl;
}

void State_JointPD::run(){
    // cout<<this->FL_Joint_Pos.transpose()<<endl;
    // Vec3 Init_Joint_Pos;
    // Init_Joint_Pos<<0,1.5,-1.6;

    _time = (double)iter/(_quaCP->CONTROL_RATE);
    ++iter;

    for(int joint = 0;joint<3;++joint){
        this->swingTraj->CubicPolynomialInterpolation(FL_Joint_Pos[joint],FL_Joint_Vel[joint],FL_Joint_Target_Pos[joint],0,_time,_quaCP->JOINT_PD_PERIOD,FL_Joint_Pos_Des[joint],FL_Joint_Vel_Des[joint]);
    }
    for(int joint = 0;joint<3;++joint){
        this->swingTraj->CubicPolynomialInterpolation(HL_Joint_Pos[joint],HL_Joint_Vel[joint],HL_Joint_Target_Pos[joint],0,_time,_quaCP->JOINT_PD_PERIOD,HL_Joint_Pos_Des[joint],HL_Joint_Vel_Des[joint]);
    }  
    for(int joint = 0;joint<3;++joint){
        this->swingTraj->CubicPolynomialInterpolation(HR_Joint_Pos[joint],HR_Joint_Vel[joint],HR_Joint_Target_Pos[joint],0,_time,_quaCP->JOINT_PD_PERIOD,HR_Joint_Pos_Des[joint],HR_Joint_Vel_Des[joint]);
    }
    for(int joint = 0;joint<3;++joint){
        this->swingTraj->CubicPolynomialInterpolation(FR_Joint_Pos[joint],FR_Joint_Vel[joint],FR_Joint_Target_Pos[joint],0,_time,_quaCP->JOINT_PD_PERIOD,FR_Joint_Pos_Des[joint],FR_Joint_Vel_Des[joint]);
    }

    // cout<<_time<<"|"<<FL_Joint_Pos_Des.transpose()<<endl;

    this->JointPDControl(0,FL_Joint_Pos_Des,FL_Joint_Vel_Des);
    this->JointPDControl(1,HL_Joint_Pos_Des,HL_Joint_Vel_Des);
    this->JointPDControl(2,HR_Joint_Pos_Des,HR_Joint_Vel_Des);
    this->JointPDControl(3,FR_Joint_Pos_Des,FR_Joint_Vel_Des);
}

void State_JointPD::onExit(){

}

State_Name State_JointPD::checkTransition(){
    if(iter>(_quaCP->CONTROL_RATE*(_quaCP->JOINT_PD_PERIOD))) return State_Name::STAND_UP;
    return State_Name::JOINT_PD;
}

TransitionData State_JointPD::getTransitionData(){
    return this->_transitonData;
}