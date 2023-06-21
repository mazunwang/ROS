#ifndef _QUADOGCONTROLPARAMETERS_H_
#define _QUADOGCONTROLPARAMETERS_H_

#include "ControlParameters.h"

ControlParameters getQuadogControlParameters(){
    
    ControlParameters* quadog = new ControlParameters();
    quadog->KP_HIP_X = 800.;
    quadog->KD_HIP_X = 5;
    quadog->KP_HIP_Y = 800.;
    quadog->KD_HIP_Y = 5;
    quadog->KP_KNEE = 800.;
    quadog->KD_KNEE = 5.;
    quadog->KP_F_X = 6000.;
    quadog->KD_F_X = 80.;
    quadog->KP_F_Y = 4000.;
    quadog->KD_F_Y = 50.;
    quadog->KP_F_Z = 2000.;
    quadog->KD_F_Z = 100.;
    quadog->KP_TORQ_X = 1000.;
    quadog->KD_TORQ_X = 25.;
    quadog->KP_TORQ_Y = 1000.;
    quadog->KD_TORQ_Y = 120.;
    quadog->KP_TORQ_Z = 1000.;
    quadog->KD_TORQ_Z = 25.;   

    quadog->SWING_PERIOD = 1.0;
    quadog->LOAD_PERIOD= 1.0;
    quadog->UNLOAD_PERIOD= 1.0;
    quadog->COM_MOVE_PERIOD= 1.0;
    quadog->INIT_PERIOD = 0.2;
    quadog->JOINT_PD_PERIOD = 0.5;
    quadog->STANDUP_PERIOD = 0.8;
    quadog->BALANCESTAND_PERIOD = 0.5;

    quadog->STEP_LENGTH = 0.15;
    quadog->STEP_HEIGHT = 0.08;

    quadog->CONTROL_RATE = 1000;

    quadog->Joint_PD_FL_Leg_Pos<<0.3175,0.173,-0.15;
    quadog->Joint_PD_HL_Leg_Pos<<-0.3175,0.173,-0.15;
    quadog->Joint_PD_HR_Leg_Pos<<-0.3175,-0.173,-0.15;
    quadog->Joint_PD_FR_Leg_Pos<<0.3175,-0.173,-0.15;

    quadog->StandUp_Height = -0.48;
    quadog->WALK_HEIGHT = 0.47;

    quadog->FZ_FEEDFORWARD = 640.0;

    quadog->lamda = 0.3;

    quadog->alpha_set = 0.8;

    quadog->Touching_Pos_Height = -0.48;

    quadog->Force_Threshold = 40.0;

    return *quadog;
}

#endif
