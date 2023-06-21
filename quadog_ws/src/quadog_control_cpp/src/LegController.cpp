#include "LegController.h"

void LegController::LegDataUpdate(const sensor_msgs::JointState::ConstPtr& msg){
    // for(int i=0;i<msg->name.size();++i) cout<<msg->name[i]<<" ";
    // cout<<endl;

    legData[0].JointPosition << msg->position[0],msg->position[1],msg->position[2];
    legData[1].JointPosition << msg->position[6],msg->position[7],msg->position[8];
    legData[2].JointPosition << msg->position[9],msg->position[10],msg->position[11];
    legData[3].JointPosition << msg->position[3],msg->position[4],msg->position[5];

    legData[0].JointVelocity << msg->velocity[0],msg->velocity[1],msg->velocity[2];
    legData[1].JointVelocity << msg->velocity[6],msg->velocity[7],msg->velocity[8];
    legData[2].JointVelocity << msg->velocity[9],msg->velocity[10],msg->velocity[11];
    legData[3].JointVelocity << msg->velocity[3],msg->velocity[4],msg->velocity[5];

    legData[0].JointEffort << msg->effort[0],msg->effort[1],msg->effort[2];
    legData[1].JointEffort << msg->effort[6],msg->effort[7],msg->effort[8];
    legData[2].JointEffort << msg->effort[9],msg->effort[10],msg->effort[11];
    legData[3].JointEffort << msg->effort[3],msg->effort[4],msg->effort[5];

    // LegDataPrint();
}

void LegController::LegCommandInitial(){
    // this->pub1 = 
}

void LegController::LegCommandPublish(){

}

void LegController::LegDataPrint(){
    for(int leg = 0;leg<4;++leg){
        cout<<legDef[leg]<<" "<<legData[leg].JointPosition.transpose()<<" "<<legData[leg].JointVelocity.transpose()<<" "<<legData[leg].JointEffort.transpose()<<" "<<endl;
    }
} 

Vec3 LegController::GetLegPos(int leg){
    double c1 = cos(legData[leg].JointPosition[0]);     
    double c2 = cos(legData[leg].JointPosition[1]);   
    double s2 = -sin(legData[leg].JointPosition[1]);
    double ck = cos(legData[leg].JointPosition[2]+legData[leg].JointPosition[1]);    
    double sk = -sin(legData[leg].JointPosition[2]+legData[leg].JointPosition[1]);
    double w_bias = quad->bodyWidth/2;
    double l_bias = quad->bodyLength/2;
    double l1 = quad->HipXLength;
    double l2 = quad->HipYLength;
    double l3 = quad->KneeLength;
    double x = 0,y = 0,z = 0;
    double s1 = 0;
    if(leg==0){
        s1 =  sin(legData[leg].JointPosition[0]);
        y = l1*c1+l2*c2*s1+l3*s1*ck+w_bias;
        x = l3*sk+l2*s2+l_bias;
    }
    else if(leg==1){
        s1 =  sin(legData[leg].JointPosition[0]);
        y = l1*c1+l2*c2*s1+l3*s1*ck+w_bias;
        x = l3*sk+l2*s2-l_bias;
    }
    else if(leg==3){
        s1 =  -sin(legData[leg].JointPosition[0]);
        y = -l1*c1-l2*c2*s1-l3*s1*ck-w_bias;
        x = l3*sk+l2*s2+l_bias;
    }
    else if(leg==2){
        s1 =  -sin(legData[leg].JointPosition[0]);
        y = -l1*c1-l2*c2*s1-l3*s1*ck-w_bias;
        x = l3*sk+l2*s2-l_bias;
    }
    else{
        std::cout<<"FK_Cal Leg_Name Error!!!"<<std::endl;
        assert(0);
    }
    z = l1*s1-l2*c1*c2-l3*c1*ck;
    Vec3 footPos;
    footPos<<x,y,z;
    return footPos;
}

Vec3 LegController::GetLegPos(string leg_name){
    int leg = 0;
    if(leg_name=="FL") leg=0;
    else if(leg_name=="HL") leg=1;
    else if(leg_name=="FR") leg = 3;
    else if(leg_name=="HL") leg=2;
    else{
        std::cout<<"FK_Cal Leg_Name Error!!!"<<std::endl;
        assert(0);
    }
    return GetLegPos(leg);
}

Vec3 LegController::GetLegPos(int leg,Vec3 JointPos){
    double c1 = cos(JointPos[0]);     
    double c2 = cos(JointPos[1]);   
    double s2 = -sin(JointPos[1]);
    double ck = cos(JointPos[2]+JointPos[1]);    
    double sk = -sin(JointPos[2]+JointPos[1]);
    double w_bias = quad->bodyWidth/2;
    double l_bias = quad->bodyLength/2;
    double l1 = quad->HipXLength;
    double l2 = quad->HipYLength;
    double l3 = quad->KneeLength;
    double x = 0,y = 0,z = 0;
    double s1 = 0;
    if(leg==0){
        s1 =  sin(JointPos[0]);
        y = l1*c1+l2*c2*s1+l3*s1*ck+w_bias;
        x = l3*sk+l2*s2+l_bias;
    }
    else if(leg==1){
        s1 =  sin(JointPos[0]);
        y = l1*c1+l2*c2*s1+l3*s1*ck+w_bias;
        x = l3*sk+l2*s2-l_bias;
    }
    else if(leg==3){
        s1 =  -sin(JointPos[0]);
        y = -l1*c1-l2*c2*s1-l3*s1*ck-w_bias;
        x = l3*sk+l2*s2+l_bias;
    }
    else if(leg==2){
        s1 =  -sin(JointPos[0]);
        y = -l1*c1-l2*c2*s1-l3*s1*ck-w_bias;
        x = l3*sk+l2*s2-l_bias;
    }
    else{
        std::cout<<"FK_Cal Leg_Name Error!!!"<<std::endl;
        assert(0);
    }
    z = l1*s1-l2*c1*c2-l3*c1*ck;
    Vec3 footPos;
    footPos<<x,y,z;
    return footPos;    
}

Vec3 LegController::LegIKCal(int leg,Vec3 pos_vec){
    double w_bias = quad->bodyWidth/2.0;
    double l_bias = quad->bodyLength/2.0;
    double x,y,z=pos_vec[2];
    double link_1 = quad->HipXLength;
    double link_2 = quad->HipYLength;
    double link_k = quad->KneeLength;
    double theta1,theta2,thetak;
    if(leg==0){
        x=pos_vec[0]-l_bias;
        y=pos_vec[1]-w_bias;
        theta1=acos(link_1/(sqrt(y*y+z*z)))-M_PI/2-atan(y/z);
    }
    else if(leg==1){
        x=pos_vec[0]+l_bias;
        y=pos_vec[1]-w_bias;
        theta1=acos(link_1/(sqrt(y*y+z*z)))-M_PI/2-atan(y/z);
    }
    else if(leg==2){
        x=pos_vec[0]+l_bias;
        y=-pos_vec[1]-w_bias;
        theta1=-acos(link_1/(sqrt(y*y+z*z)))+M_PI/2+atan(y/z);
    }
    else if(leg==3){
        x=pos_vec[0]-l_bias;
        y=-pos_vec[1]-w_bias;
        theta1=-acos(link_1/(sqrt(y*y+z*z)))+M_PI/2+atan(y/z);
    }
    else{
        std::cout<<"Leg_Num Error"<<std::endl;
        assert(0);
    }
    // cout<<x<<" "<<y<<" "<<z<<" "<<quad->bodyWidth<<" "<<quad->bodyLength<<endl;
    thetak=acos(-(x*x+y*y+z*z-link_1*link_1-link_2*link_2-link_k*link_k)/(2*link_k*link_2))-M_PI;
    theta2=-atan(x/(sqrt(y*y+z*z-link_1*link_1)))+asin(link_k/sqrt(x*x+y*y+z*z-link_1*link_1)*sin(M_PI+thetak));
    Vec3 theta_vec;
    theta_vec << theta1,theta2,thetak;
    return theta_vec;
}

Vec3 LegController::LegIKCal(string leg_name,Vec3 pos_vec){
    int leg = 0;
    if(leg_name=="FL") leg=0;
    else if(leg_name=="HL") leg=1;
    else if(leg_name=="FR") leg = 3;
    else if(leg_name=="HL") leg=2;
    else{
        std::cout<<"FK_Cal Leg_Name Error!!!"<<std::endl;
        assert(0);
    }
    return LegIKCal(leg,pos_vec);
}

Mat3 LegController::GetJacobianMat(int leg,Mat3 Body_Matrix){
    Mat3 JacobiMat;
    Mat3 Shoulder_Matrix;
    Mat3 Jacobian_output;
    double c1,s1,c2,s2,ck,sk;
    c1 = cos(legData[leg].JointPosition[0]);    
    s1 = sin(legData[leg].JointPosition[0]); 
    c2 = cos(legData[leg].JointPosition[1]);    
    s2 = sin(legData[leg].JointPosition[1]); 
    ck = cos(legData[leg].JointPosition[1]+legData[leg].JointPosition[2]);   
    sk = sin(legData[leg].JointPosition[1]+legData[leg].JointPosition[2]); 
    double l1 = quad->HipXLength;
    double l2 = quad->HipYLength;
    double l3 = quad->KneeLength;
    if(leg==0||leg==1){
        JacobiMat(0,1) = -l3*ck-l2*c2;
        JacobiMat(0,2) = -l1;
        JacobiMat(2,0) = l3*ck;
        JacobiMat(2,2) = -l3*sk;
        JacobiMat(1,0) = JacobiMat(2,0)+l2*c2;
        JacobiMat(1,2) = JacobiMat(2,2)-l2*s2;
        Shoulder_Matrix(0,0)=1;
        Shoulder_Matrix(1,1)=c1;
        Shoulder_Matrix(1,2)=-s1;
        Shoulder_Matrix(2,1)=s1;
        Shoulder_Matrix(2,2)=c1;
        Mat3 tmp_mat = Body_Matrix*Shoulder_Matrix;
        Jacobian_output = JacobiMat*tmp_mat.transpose();   
    }
    else if(leg==2||leg==3){
        JacobiMat(0,1) = -l3*ck-l2*c2;
        JacobiMat(0,2) = l1;
        JacobiMat(2,0) = l3*ck;
        JacobiMat(2,2) = -l3*sk;
        JacobiMat(1,0) = JacobiMat(2,0)+l2*c2;
        JacobiMat(1,2) = JacobiMat(2,2)-l2*s2;
        Shoulder_Matrix(0,0)=1;
        Shoulder_Matrix(1,1)=c1;
        Shoulder_Matrix(1,2)=-s1;
        Shoulder_Matrix(2,1)=s1;
        Shoulder_Matrix(2,2)=c1;
        Mat3 tmp_mat = Body_Matrix*Shoulder_Matrix;
        Jacobian_output = JacobiMat*tmp_mat.transpose();

    }
    else{
        std::cout<<"Leg_Name Error"<<std::endl;
        assert(0);
    }
    return Jacobian_output;
}

Vec3 LegController::GetFootVelocity(int leg){
    double c1 = cos(legData[leg].JointPosition[0])   ; 
    double c2 = cos(legData[leg].JointPosition[1]);    
    double s2 = -sin(legData[leg].JointPosition[1]) ;
    double ck = cos(legData[leg].JointPosition[2]+legData[leg].JointPosition[1]) ;   
    double sk = -sin(legData[leg].JointPosition[2]+legData[leg].JointPosition[1]) ;
    double w2 = -legData[leg].JointVelocity[1];
    double wk = -legData[leg].JointVelocity[2]-legData[leg].JointVelocity[1];
    double l1 = quad->HipXLength;
    double l2 = quad->HipYLength;
    double l3 = quad->KneeLength;
    double dx,dy,dz,w1,s1;
    if(leg==0||leg==1){
        s1 = sin(legData[leg].JointPosition[0]);
        w1 = legData[leg].JointVelocity[0];
        dy = -l1*s1*w1-l2*s2*s1*w2+l2*c2*c1*w1+l3*c2*ck*w1+l3*s1*sk*wk;
    }
    else if(leg==2||leg==3){
        s1 = -sin(legData[leg].JointPosition[0]) ;
        w1 = -legData[leg].JointVelocity[0];
        dy = -(-l1*s1*w1-l2*s2*s1*w2+l2*c2*c1*w1+l3*c2*ck*w1+l3*s1*sk*wk);
    }

    else{
        std::cout<<"FK_Vel Leg_Name Error!!!"<<std::endl;
        assert(0);
    } 
    dx = l3*ck*wk+l2*c2*w2;
    dz = l1*c1*w1+l2*c2*s2*w2+l2*s1*c2*w1+l3*s1*ck*w1+l3*c1*sk*wk;
    Vec3 foot_vel;
    foot_vel << dx, dy, dz;
    return foot_vel;
}

Vec3 LegController::GetFootVelocity(string leg_name){
    int leg = 0;
    if(leg_name=="FL") leg=0;
    else if(leg_name=="HL") leg=1;
    else if(leg_name=="FR") leg = 3;
    else if(leg_name=="HL") leg=2;
    else{
        std::cout<<"FK_Vel Leg_Name Error!!!"<<std::endl;
        assert(0);
    }
    return GetFootVelocity(leg);
}

Vec3 LegController::GetHipXPos(int leg){
    Vec3 res;
    if(leg==0){
        res<<this->quad->bodyLength/2.0,quad->bodyWidth/2.0,0.0;
    }
    else if(leg==1){
        res<<-this->quad->bodyLength/2.0,quad->bodyWidth/2.0,0.0;
    }
    else if(leg==2){
        res<<-this->quad->bodyLength/2.0,-quad->bodyWidth/2.0,0.0;
    }
    else if(leg==3){
        res<<this->quad->bodyLength/2.0,-quad->bodyWidth/2.0,0.0;
    }
    else{
        std::cout<<"leg error"<<std::endl;
        assert(0);
    }
    return res;
}

Vec3 LegController::GetHipYPos(int leg){
    Vec3 HipX_Pos;
    HipX_Pos = this->GetHipXPos(leg);
    double thetaHipX = legData[leg].JointPosition[0];
    Mat3 RotHipX;
    RotHipX<<1.0 , 0.0 ,0.0,
            0.0,cos(thetaHipX),-sin(thetaHipX),
            0.0,sin(thetaHipX),cos(thetaHipX);
    Vec3 HipX_Link;
    if(leg==0||leg==1){
        HipX_Link<<0.0,quad->HipXLength,0.0;
    }
    else if(leg==2||leg==3){
        HipX_Link<<0.0,-quad->HipXLength,0.0;
    }
    Vec3 res;
    res = HipX_Pos + RotHipX * HipX_Link;
    return res;
} 