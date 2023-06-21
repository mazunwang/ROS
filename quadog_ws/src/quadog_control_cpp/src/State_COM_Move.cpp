#include"State_COM_Move.h"

void State_COM_Move::onEnter(){
    contact_foot<<1.0,1.0,1.0,1.0;
    body_matrix = _container->_stateEstimatorData->result->Body_Matrix;
    start_time = _container->_stateEstimatorData->result->t;
    h_vel_Init = 0;
    h_pos_Init = 0;
    for(int leg = 0;leg<4;++leg){
        this->Leg_Pos_Init[leg] = _legController->GetLegPos(leg);
        this->Leg_Pos_Init_World[leg] = body_matrix * Leg_Pos_Init[leg];
        this->Leg_Vel_Init[leg] = _legController->GetFootVelocity(leg);
        this->Leg_Vel_Init_World[leg] = body_matrix*Leg_Vel_Init[leg];
        h_vel_Init += this->Leg_Vel_Init_World[leg][2];
        h_pos_Init += this->Leg_Pos_Init_World[leg][2];
    }
    h_pos_Init = -h_pos_Init/4.0;
    h_vel_Init = -h_vel_Init/4.0;

    h_set = _quaCP->WALK_HEIGHT;

    if(legNum>=4) legNum=0;
    curLegNum = legSeq[legNum];
    refLegNum = getRefLegNum(legNum);

    getXYInitState();
    RPY_Init = _container->_stateEstimatorData->result->rpy;

    std::cout<< "-----------" << this->_legController->legName[curLegNum] <<" Leg COM_Move State Start"<<"----------"<<std::endl;
    cout<<x_pos_Init<<" "<<y_pos_Init<<" "<<h_pos_Init<<endl;
}

void State_COM_Move::run(){
    h_pos = 0.0;
    h_vel = 0.0;

    body_matrix = _container->_stateEstimatorData->result->Body_Matrix;
    RPY = _container->_stateEstimatorData->result->rpy;
    OmigaWorld = _container->_stateEstimatorData->result->BodyOmigaWorld;

    for(int leg = 0;leg<4;++leg){
        Leg_Pos_Vector[leg] = this->_legController->GetLegPos(leg);
        Leg_Vel_Vector[leg] = this->_legController->GetFootVelocity(leg);
        Leg_Pos_World_Vector[leg] = body_matrix*Leg_Pos_Vector[leg];
        Leg_Vel_World_Vector[leg] = body_matrix*Leg_Vel_Vector[leg];

        h_pos+=Leg_Pos_World_Vector[leg][2];
        h_vel+= Leg_Vel_World_Vector[leg][2];
        Leg_Pos_World_Mat.block<1,3>(leg,0)<<Leg_Pos_World_Vector[leg].transpose();
        // std::cout<< this->_legController->legName[leg]<<":"<<Leg_Pos_World_Mat.block<1,3>(leg,0) <<std::endl;
    }
    h_pos = -h_pos/4.0;
    h_vel = -h_vel/4.0;
    getXYCurState();

    _time = _container->_stateEstimatorData->result->t - start_time;
    this->swingTraj->CubicPolynomialInterpolation(x_pos_Init,x_vel_Init,x_set,0,_time,_quaCP->COM_MOVE_PERIOD,x_des_pos,x_des_vel);
    this->swingTraj->CubicPolynomialInterpolation(y_pos_Init,y_vel_Init,y_set,0,_time,_quaCP->COM_MOVE_PERIOD,y_des_pos,y_des_vel);
    this->swingTraj->CubicPolynomialInterpolation(h_pos_Init,h_vel_Init,h_set,0,_time,_quaCP->COM_MOVE_PERIOD,h_des_pos,h_des_vel);   

    // cout<<"x_des_pos:"<<x_des_pos<<" "<<"y_des_pos"<<y_des_pos<<" h_des_pos:"<<h_des_pos<<"  " <<refLegNum<<endl; 
    // cout<<"x_pos:"<<x_pos<<" "<<"y_pos:"<<y_pos<<" h_pos:"<<h_pos<<endl;

    F_Vec[0] = _quaCP->KP_F_X*(x_des_pos-x_pos)+_quaCP->KD_F_X*(x_des_vel-x_vel);
    F_Vec[1] = _quaCP->KP_F_Y*(y_des_pos-y_pos)+_quaCP->KD_F_Y*(y_des_vel-y_vel);
    F_Vec[2] = _quaCP->KP_F_Z*(h_des_pos-h_pos)+_quaCP->KD_F_Z*(h_des_vel-h_vel)+_quaCP->FZ_FEEDFORWARD;
    F_Vec[3] = _quaCP->KP_TORQ_X*(0-RPY[0])+_quaCP->KD_TORQ_X*(0-OmigaWorld[0]);
    F_Vec[4] = _quaCP->KP_TORQ_Y*(0-RPY[1])+_quaCP->KD_TORQ_Y*(0-OmigaWorld[1]);
    F_Vec[5] = _quaCP->KP_TORQ_Z*(RPY_Init[2]-RPY[2])+_quaCP->KD_TORQ_Z*(0-OmigaWorld[2]);    

        // cout<<"F_Vec:"<<F_Vec.transpose()<<endl;
        // cout<<"h_pos:"<<h_pos<<"h_des_pos:"<<h_des_pos<<endl;

    this->_balanceController.updata_qpSolver_parameters(Leg_Pos_World_Mat,F_Vec,contact_foot);

    
    this->_balanceController.qp_solver(fOpt);

    for(int leg=0;leg<4;++leg){
        Leg_Force[leg]<<fOpt[3*leg+0],fOpt[3*leg+1],fOpt[3*leg+2];
        Leg_Torq[leg] = (this->_legController->GetJacobianMat(leg,body_matrix))*Leg_Force[leg];
        _legController->legCommand[leg].EffortCommand<<Leg_Torq[leg];
    }

    // if(legNum)
}

void State_COM_Move::onExit(){
    legNum++;
}

State_Name State_COM_Move::checkTransition(){
    if(_container->_stateEstimatorData->result->t - start_time>_quaCP->BALANCESTAND_PERIOD * 1.1) return State_Name::UNLOADFORCE;
    return State_Name::COM_MOVE;
}

TransitionData State_COM_Move::getTransitionData(){
    return this->_transitonData;
}


void State_COM_Move::getXYInitState(){
    x_pos_Init = -this->Leg_Pos_Init_World[refLegNum][0];
    x_vel_Init = -this->Leg_Vel_Init_World[refLegNum][0];
    y_pos_Init = -this->Leg_Pos_Init_World[refLegNum][1];
    y_vel_Init = -this->Leg_Vel_Init_World[refLegNum][1]; 
    if(curLegNum==0){
        x_set = 0.5*(Leg_Pos_Init_World[3][0]-Leg_Pos_Init_World[1][0])+0.5*(Leg_Pos_Init_World[2][0]-Leg_Pos_Init_World[1][0])*(_quaCP->lamda);
        y_set = 0.5*(Leg_Pos_Init_World[3][1]-Leg_Pos_Init_World[1][1])+0.5*(Leg_Pos_Init_World[2][1]-Leg_Pos_Init_World[1][1])*(_quaCP->lamda);
    }
    else if(curLegNum==1){
        x_set = 0.5*(Leg_Pos_Init_World[2][0]-Leg_Pos_Init_World[0][0])+0.5*(Leg_Pos_Init_World[3][0]-Leg_Pos_Init_World[0][0])*(_quaCP->lamda);
        y_set = 0.5*(Leg_Pos_Init_World[2][1]-Leg_Pos_Init_World[0][1])+0.5*(Leg_Pos_Init_World[3][1]-Leg_Pos_Init_World[0][1])*(_quaCP->lamda);
    }
    else if(curLegNum==2){
        x_set = 0.5*(Leg_Pos_Init_World[1][0]-Leg_Pos_Init_World[3][0])+0.5*(Leg_Pos_Init_World[0][0]-Leg_Pos_Init_World[3][0])*(_quaCP->lamda);
        y_set = 0.5*(Leg_Pos_Init_World[1][1]-Leg_Pos_Init_World[3][1])+0.5*(Leg_Pos_Init_World[0][1]-Leg_Pos_Init_World[3][1])*(_quaCP->lamda);
    }
    else if(curLegNum==3){
        x_set = 0.5*(Leg_Pos_Init_World[0][0]-Leg_Pos_Init_World[2][0])+0.5*(Leg_Pos_Init_World[1][0]-Leg_Pos_Init_World[2][0])*(_quaCP->lamda);
        y_set = 0.5*(Leg_Pos_Init_World[0][1]-Leg_Pos_Init_World[2][1])+0.5*(Leg_Pos_Init_World[1][1]-Leg_Pos_Init_World[2][1])*(_quaCP->lamda);
    }
    else{
        std::cout<<"LegNum Error"<<std::endl;
        assert(0);
    }
}

void State_COM_Move::getXYCurState(){
    x_pos = -this->Leg_Pos_World_Vector[refLegNum][0];
    x_vel = -this->Leg_Vel_World_Vector[refLegNum][0];
    y_pos = -this->Leg_Pos_World_Vector[refLegNum][1];
    y_vel = -this->Leg_Vel_World_Vector[refLegNum][1];
}