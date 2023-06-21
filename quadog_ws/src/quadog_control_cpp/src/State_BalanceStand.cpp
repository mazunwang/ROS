#include"State_BalanceStand.h"

void State_BalanceStand::onEnter(){
    iter = 0;
    start_time = _container->_stateEstimatorData->result->t;
    body_matrix = _container->_stateEstimatorData->result->Body_Matrix;
    for(int leg = 0;leg<4;++leg){
        this->Leg_Pos_Init[leg] = _legController->GetLegPos(leg);
        this->Leg_Pos_Init_World[leg] = body_matrix * Leg_Pos_Init[leg];
        x_set+=this->Leg_Pos_Init_World[leg][0];
        y_set+=this->Leg_Pos_Init_World[leg][1];
        h_set+=this->Leg_Pos_Init_World[leg][2];

        Des_Joint_Pos[leg] = this->_legController->legData[leg].JointPosition; 
        Des_Joint_Vel[leg] << 0.,0.,0.;
    }
    x_set = -x_set/4.0;
    y_set = -y_set/4.0;
    h_set = -h_set/4.0;
    RPY_Init = _container->_stateEstimatorData->result->rpy;
    std::cout<<"----------BalanceStand Start----------"<<std::endl;
    std::cout<<" "<<x_set<<" "<<y_set<<" "<<h_set<<std::endl;
}

void State_BalanceStand::run(){
    iter++;

    body_matrix = _container->_stateEstimatorData->result->Body_Matrix;

    RPY = _container->_stateEstimatorData->result->rpy;
    OmigaWorld = _container->_stateEstimatorData->result->BodyOmigaWorld;

    body_height_total= 0.0;
    body_velo_total = 0.0;
    x_pos = 0.0;
    y_pos = 0.0;
    x_vel = 0.0;
    y_vel = 0.0;

    for(int leg = 0;leg<4;++leg){
        Leg_Pos_Vector[leg] = this->_legController->GetLegPos(leg);
        Leg_Vel_Vector[leg] = this->_legController->GetFootVelocity(leg);
        Leg_Pos_World_Vector[leg] = body_matrix*Leg_Pos_Vector[leg];
        Leg_Vel_World_Vector[leg] = body_matrix*Leg_Vel_Vector[leg];

        body_height_total+=Leg_Pos_World_Vector[leg][2];
        body_velo_total += Leg_Vel_World_Vector[leg][2];

        x_pos += Leg_Pos_World_Vector[leg][0];
        y_pos += Leg_Pos_World_Vector[leg][1];
        x_vel += Leg_Vel_World_Vector[leg][0];
        y_vel += Leg_Vel_World_Vector[leg][1];

        Leg_Pos_World_Mat.block<1,3>(leg,0)<<Leg_Pos_World_Vector[leg].transpose();

        // std::cout<< this->_legController->legName[leg]<<":"<<Leg_Pos_World_Mat.block<1,3>(leg,0) <<std::endl;
    }

    body_height = -body_height_total/4.0;
    body_velo = -body_velo_total/4.0;
    // cout<<body_height<< " "<<body_velo<<endl;

    x_pos = -x_pos/4.0;
    y_pos = -y_pos/4.0;
    x_vel = -x_vel/4.0;
    y_vel = -y_vel/4.0;

    F_Vec[0] = _quaCP->KP_F_X*(x_set-x_pos)+_quaCP->KD_F_X*(0-x_vel);
    F_Vec[1] = _quaCP->KP_F_Y*(y_set-y_pos)+_quaCP->KD_F_Y*(0-y_vel);
    F_Vec[2] = _quaCP->KP_F_Z*(h_set-body_height)+_quaCP->KD_F_Z*(0-body_velo)+_quaCP->FZ_FEEDFORWARD;
    F_Vec[3] = _quaCP->KP_TORQ_X*(0-RPY[0])+_quaCP->KD_TORQ_X*(0-OmigaWorld[0]);
    F_Vec[4] = _quaCP->KP_TORQ_Y*(0-RPY[1])+_quaCP->KD_TORQ_Y*(0-OmigaWorld[1]);
    F_Vec[5] = _quaCP->KP_TORQ_Z*(RPY_Init[2]-RPY[2])+_quaCP->KD_TORQ_Z*(0-OmigaWorld[2]);

    // cout<<"F_Vec:"<<F_Vec.transpose()<<endl;

    this->_balanceController.updata_qpSolver_parameters(Leg_Pos_World_Mat,F_Vec,contact_foot);

    // double fOpt[12];
    this->_balanceController.qp_solver(fOpt);

    for(int leg=0;leg<4;++leg){
        Leg_Force[leg]<<fOpt[3*leg+0],fOpt[3*leg+1],fOpt[3*leg+2];
        Leg_Torq[leg] = (this->_legController->GetJacobianMat(leg,body_matrix))*Leg_Force[leg];
        _legController->legCommand[leg].EffortCommand<<Leg_Torq[leg];
	// _legController->legCommand[leg].EffortCommand[1] = HipY_Torq;
	// _legController->legCommand[leg].EffortCommand[2] = Knee_Torq;
    }

    // std::cout<<iter<<" "<<this->_container->_stateEstimatorData->result->t<<std::endl;
    // for(int leg=0;leg<4;++leg){
        // for(int joint = 0;joint<3;++joint){
        //     this->swingTraj->CubicPolynomialInterpolation(Joint_Pos_Init[leg][joint],Joint_Vel_Init[leg][joint],Target_Joint_Pos[leg][joint],0,_time,_quaCP->STANDUP_PERIOD,Des_Joint_Pos[leg][joint],Des_Joint_Vel[leg][joint]);
        // }
        // this->JointPDControl(leg,Des_Joint_Pos[leg],Des_Joint_Vel[leg]);

        // cout<<_time<<"  "<<leg<<" "<<Des_Joint_Pos[leg].transpose()<<endl;
    // }

}

void State_BalanceStand::onExit(){

}

State_Name State_BalanceStand::checkTransition(){

    _time = _container->_stateEstimatorData->result->t;
    if(_time-start_time>_quaCP->BALANCESTAND_PERIOD) return State_Name::COM_MOVE;
    return State_Name::BALANCE_STAND;
}



TransitionData State_BalanceStand::getTransitionData(){
    return this->_transitonData;
}