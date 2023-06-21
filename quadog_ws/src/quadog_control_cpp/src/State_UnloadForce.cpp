#include"State_UnloadForce.h"

void State_UnloadForce::onEnter(){
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
    h_set = _quaCP->WALK_HEIGHT;

    if(legNum>=4) legNum=0;

    curLegNum = legSeq[legNum];
    refLegNum = getRefLegNum(legNum);

    getXYInitState();

    RPY_Init = _container->_stateEstimatorData->result->rpy;
    h_pos_Init = -(h_pos_Init-Leg_Pos_Init_World[curLegNum][2])/3.0;
    h_vel_Init = -(h_vel_Init-Leg_Vel_Init_World[curLegNum][2])/3.0;

    alpha = _quaCP->alpha_set;

    std::cout<< "-----------" << this->_legController->legName[curLegNum] <<" Leg UnloadForce State Start"<<"----------"<<std::endl;
    cout<<x_pos_Init<<" "<<y_pos_Init<<" "<<h_pos_Init<<endl;
    // cout<<"refLeg"<<refLegNum<<endl;
}

void State_UnloadForce::run(){
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
    
    h_pos = -(h_pos-Leg_Pos_World_Vector[curLegNum][2])/3.0;
    h_vel = -(h_vel-Leg_Vel_World_Vector[curLegNum][2])/3.0;

    getXYCurState();
    _time = _container->_stateEstimatorData->result->t - start_time;
    this->swingTraj->CubicPolynomialInterpolation(x_pos_Init,x_vel_Init,x_set,0,_time,_quaCP->UNLOAD_PERIOD,x_des_pos,x_des_vel);
    this->swingTraj->CubicPolynomialInterpolation(y_pos_Init,y_vel_Init,y_set,0,_time,_quaCP->UNLOAD_PERIOD,y_des_pos,y_des_vel);
    this->swingTraj->CubicPolynomialInterpolation(h_pos_Init,h_vel_Init,h_set,0,_time,_quaCP->UNLOAD_PERIOD,h_des_pos,h_des_vel);    

    F_Vec[0] = _quaCP->KP_F_X*(x_des_pos-x_pos)+_quaCP->KD_F_X*(x_des_vel-x_vel);
    F_Vec[1] = _quaCP->KP_F_Y*(y_des_pos-y_pos)+_quaCP->KD_F_Y*(y_des_vel-y_vel);
    F_Vec[2] = _quaCP->KP_F_Z*(h_des_pos-h_pos)+_quaCP->KD_F_Z*(h_des_vel-h_vel)+_quaCP->FZ_FEEDFORWARD;
    F_Vec[3] = _quaCP->KP_TORQ_X*(0-RPY[0])+_quaCP->KD_TORQ_X*(0-OmigaWorld[0]);
    F_Vec[4] = _quaCP->KP_TORQ_Y*(0-RPY[1])+_quaCP->KD_TORQ_Y*(0-OmigaWorld[1]);
    F_Vec[5] = _quaCP->KP_TORQ_Z*(RPY_Init[2]-RPY[2])+_quaCP->KD_TORQ_Z*(0-OmigaWorld[2]);    

    // double alpha_des;
    // double aplah_vel;

    this->swingTraj->LinerInterPolation(alpha,0,_time,_quaCP->UNLOAD_PERIOD,alpha_des,aplah_vel);

    // for(int leg=0;leg<4;++leg){
        contact_foot[curLegNum] = alpha_des;
    // }

        // cout<<"F_Vec:"<<F_Vec.transpose()<<endl;
        // cout<<contact_foot.transpose()<<endl;
        // cout<<"h_pos:"<<h_pos<<"h_des_pos:"<<h_des_pos<<endl;
        // cout<<y_des_pos<<" "<<y_pos<<" "<<y_des_vel<<" "<<y_vel<<" "<<Leg_Pos_World_Vector[3].transpose()<<endl;
        // cout<<Leg_Pos_World_Vector[3].transpose()<<Leg_Pos_Vector[3].transpose()<<endl;

    this->_balanceController.updata_qpSolver_parameters(Leg_Pos_World_Mat,F_Vec,contact_foot);

    // double fOpt[12];
    this->_balanceController.qp_solver(fOpt);

    for(int leg=0;leg<4;++leg){
        Leg_Force[leg]<<fOpt[3*leg+0],fOpt[3*leg+1],fOpt[3*leg+2];
        Leg_Torq[leg] = (this->_legController->GetJacobianMat(leg,body_matrix))*Leg_Force[leg];
        _legController->legCommand[leg].EffortCommand<<Leg_Torq[leg];

        // cout<<_legController->legName[leg]<<Leg_Force[leg].transpose()<<endl;
        
    }
}

void State_UnloadForce::onExit(){
    legNum++;
}

State_Name State_UnloadForce::checkTransition(){
    if(_time>_quaCP->UNLOAD_PERIOD * 1.2) return State_Name::SWING;
    return State_Name::UNLOADFORCE;
}

TransitionData State_UnloadForce::getTransitionData(){
    return this->_transitonData;
}


void State_UnloadForce::getXYInitState(){
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

void State_UnloadForce::getXYCurState(){
    x_pos = -this->Leg_Pos_World_Vector[refLegNum][0];
    x_vel = -this->Leg_Vel_World_Vector[refLegNum][0];
    y_pos = -this->Leg_Pos_World_Vector[refLegNum][1];
    y_vel = -this->Leg_Vel_World_Vector[refLegNum][1];
    // cout<<x_pos<<" "<<y_pos<<" "<<x_set<<" "<<y_set<<endl;
}