#include <ros/ros.h>
#include <std_msgs/String.h>
#include<std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include<rosgraph_msgs/Clock.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "quadogControlParameters.h"
#include"QuaDogController.h"

using namespace std;

Quadruped quadog = buildQuadog();
ControlParameters _quaCP = getQuadogControlParameters();
 
LegController _legController(&quadog);

StateEstimate _result;
StateEstimatorData _data(&_legController,&_result);
QuaDogController* quadogController = new QuaDogController(&quadog,&_legController,&_quaCP,&_data);

void messageCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    _data.result->orientation << msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w;
    _data.result->BodyOmiga << msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z;
    _data.result->BodyAcc << msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
}
void rosGetTime(const rosgraph_msgs::Clock::ConstPtr& msg){
  _data.result->t=ros::Time::now().toSec();
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"subscribe_and_publish");       
    ros::NodeHandle n;              
    ros::Duration(0.2).sleep();        
    ros::Subscriber sub = n.subscribe("/imu", 10, messageCallback);
    // ros::Subscriber sub = n.subscribe("/imu", 10, &StateEstimatorContainer::messageCallback,_container);//！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
    ros::Subscriber sub2 = n.subscribe("/quadog/joint_states",10,&LegController::LegDataUpdate,&_legController);
    ros::Subscriber sub3 = n.subscribe("/clock",10,rosGetTime);

    ros::Publisher pub1 = n.advertise<std_msgs::Float64>("/quadog/joint1_effort_controller/command",10);
    ros::Publisher pub2 = n.advertise<std_msgs::Float64>("/quadog/joint2_effort_controller/command",10);
    ros::Publisher pub3 = n.advertise<std_msgs::Float64>("/quadog/joint3_effort_controller/command",10);
    ros::Publisher pub4 = n.advertise<std_msgs::Float64>("/quadog/joint4_effort_controller/command",10);
    ros::Publisher pub5 = n.advertise<std_msgs::Float64>("/quadog/joint5_effort_controller/command",10);
    ros::Publisher pub6 = n.advertise<std_msgs::Float64>("/quadog/joint6_effort_controller/command",10);
    ros::Publisher pub7 = n.advertise<std_msgs::Float64>("/quadog/joint7_effort_controller/command",10);
    ros::Publisher pub8 = n.advertise<std_msgs::Float64>("/quadog/joint8_effort_controller/command",10);
    ros::Publisher pub9 = n.advertise<std_msgs::Float64>("/quadog/joint9_effort_controller/command",10);
    ros::Publisher pub10 = n.advertise<std_msgs::Float64>("/quadog/joint10_effort_controller/command",10);
    ros::Publisher pub11 = n.advertise<std_msgs::Float64>("/quadog/joint11_effort_controller/command",10);
    ros::Publisher pub12 = n.advertise<std_msgs::Float64>("/quadog/joint12_effort_controller/command",10);

    ros::AsyncSpinner spinner(4);
    ros::Rate loop_rate(_quaCP.CONTROL_RATE);
    spinner.start();
    std::cout<<"Start Controller"<<std::endl;
    while(ros::ok()){
       std_msgs::Float64 msg1,msg2,msg3,msg4,msg5,msg6,msg7,msg8,msg9,msg10,msg11,msg12; 
       quadogController->runController();
       msg1.data = _legController.legCommand[0].EffortCommand[0];
       msg2.data = _legController.legCommand[0].EffortCommand[1];
       msg3.data = _legController.legCommand[0].EffortCommand[2];
       msg4.data = _legController.legCommand[1].EffortCommand[0];
       msg5.data = _legController.legCommand[1].EffortCommand[1];     
       msg6.data = _legController.legCommand[1].EffortCommand[2];
       msg7.data = _legController.legCommand[3].EffortCommand[0];
       msg8.data = _legController.legCommand[3].EffortCommand[1];
       msg9.data = _legController.legCommand[3].EffortCommand[2];
       msg10.data = _legController.legCommand[2].EffortCommand[0];  
       msg11.data = _legController.legCommand[2].EffortCommand[1];
       msg12.data = _legController.legCommand[2].EffortCommand[2];
       pub1.publish(msg1);
       pub2.publish(msg2);
       pub3.publish(msg3);
       pub4.publish(msg4);
       pub5.publish(msg5);
       pub6.publish(msg6);
       pub7.publish(msg7);
       pub8.publish(msg8);
       pub9.publish(msg9);
       pub10.publish(msg10);
       pub11.publish(msg11);
       pub12.publish(msg12);
       loop_rate.sleep();
    }
    ros::waitForShutdown();
    
    return 0;
}

