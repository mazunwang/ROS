#include"State_Base.h"


void State_Base::JointPDControl(int leg,Vec3 Des_Joint_Pos,Vec3 Des_Joint_Vel){
	double HipX_Torq = _quaCP->KP_HIP_X*(Des_Joint_Pos[0]-_legController->legData[leg].JointPosition[0])+_quaCP->KD_HIP_X*(Des_Joint_Vel[0]-_legController->legData[leg].JointVelocity[0]);
	double HipY_Torq = _quaCP->KP_HIP_Y*(Des_Joint_Pos[1]-_legController->legData[leg].JointPosition[1])+_quaCP->KD_HIP_Y*(Des_Joint_Vel[1]-_legController->legData[leg].JointVelocity[1]);
	double Knee_Torq = _quaCP->KP_KNEE*(Des_Joint_Pos[2]-_legController->legData[leg].JointPosition[2])+_quaCP->KD_KNEE*(Des_Joint_Vel[2]-_legController->legData[leg].JointVelocity[2]);

    _legController->legCommand[leg].EffortCommand[0] = HipX_Torq;
	_legController->legCommand[leg].EffortCommand[1] = HipY_Torq;
	_legController->legCommand[leg].EffortCommand[2] = Knee_Torq;

	if(leg==0){
		// cout<<"FL:"<<_legController->legData[leg].JointPosition.transpose()<<endl;
	}
	
}

int State_Base::getRefLegNum(int num){
    switch (legSeq[num])
    {
        case 0:
            return 1;
        case 1:
            return 0;
        case 2:
            return 3;
        case 3:
            return 2;
        default:{
            std::cout<<"Leg_Num Error"<<std::endl;
            assert(0);
            return -1;
        }
    }
}