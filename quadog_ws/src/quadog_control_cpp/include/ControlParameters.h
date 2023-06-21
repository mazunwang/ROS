#ifndef _CONTROLPARAMETERS_H_
#define _CONTROLPARAMETERS_H_


#include<vector>
#include "Types.h"

class ControlParameters{

public:
    ControlParameters(){}
    ~ControlParameters(){}

    double KP_HIP_X;
    double KD_HIP_X;
    double KP_HIP_Y;
    double KD_HIP_Y;
    double KP_KNEE;
    double KD_KNEE;

    double KP_F_X;
    double KD_F_X;
    double KP_F_Y;
    double KD_F_Y;
    double KP_F_Z;
    double KD_F_Z;

    double KP_TORQ_X;
    double KD_TORQ_X;
    double KP_TORQ_Y;
    double KD_TORQ_Y;
    double KP_TORQ_Z;
    double KD_TORQ_Z;

    double FZ_FEEDFORWARD;

    double SWING_PERIOD;
    double LOAD_PERIOD;
    double UNLOAD_PERIOD;
    double COM_MOVE_PERIOD;
    double INIT_PERIOD;
    double JOINT_PD_PERIOD;
    double STANDUP_PERIOD;
    double BALANCESTAND_PERIOD;

    double STEP_LENGTH;
    double STEP_HEIGHT;

    int CONTROL_RATE;

    Vec3 Joint_PD_FL_Leg_Pos;
    Vec3 Joint_PD_HL_Leg_Pos;
    Vec3 Joint_PD_HR_Leg_Pos;
    Vec3 Joint_PD_FR_Leg_Pos;

    double StandUp_Height;

    double WALK_HEIGHT;

    double Touching_Pos_Height;

    double lamda;

    double alpha_set;

    double Force_Threshold;
};






#endif
