#ifndef _LEGCONTROLLER_H_
#define _LEGCONTROLLER_H_

#include <string>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include "Types.h"
#include "quadog.h"
#include <cmath>
#include <ros/ros.h>

// #define M_PI 3.1415926

using namespace std;

static string legDef[] = {"FL","HL","HR","FR"};

struct LegControlData{
     Vec3 JointPosition;
     Vec3 JointVelocity;
     Vec3 JointEffort;
     LegControlData(){
         JointPosition<<0,0,0;
         JointVelocity<<0,0,0;
         JointEffort<<0,0,0;
     }
};

struct LegControlCommand{
    Vec3 EffortCommand;
    LegControlCommand(){
        EffortCommand<<0,0,0;
    }
};

class LegController{
    public:
        LegController(Quadruped* _qua){
            quad = _qua;
            for(int leg = 0;leg<4;++leg) legName[leg] = legDef[leg];
        }
        ~LegController(){}


        LegControlData legData[4];
        LegControlCommand legCommand[4];
        
        void LegDataUpdate(const sensor_msgs::JointState::ConstPtr& msg);

        void LegCommandInitial();

        void LegCommandPublish();

        Vec3 GetLegPos(int leg);
        Vec3 GetLegPos(string leg);
        Vec3 GetLegPos(int leg,Vec3 JointPos);
        Vec3 LegIKCal(int leg,Vec3 pos);
        Vec3 LegIKCal(string leg_name,Vec3);
        Mat3 GetJacobianMat(int leg,Mat3 rbody);
        Vec3 GetFootVelocity(int leg);
        Vec3 GetFootVelocity(string Leg_name);
        void LegDataPrint();
        Vec3 GetHipXPos(int leg);
        Vec3 GetHipYPos(int leg);


        string legName[4];
        Quadruped* quad;

    private:
        Vec3 FL_Leg_Pos,HL_Leg_Pos,HR_Leg_Pos,FR_Leg_Pos;

        // ros::NodeHandle n;
        // ros::Publisher pub1,pub2,pub3,pub4,pub5,pub6,pub7,pub8,pub9,pub10,pub11,pub12;

};


#endif
