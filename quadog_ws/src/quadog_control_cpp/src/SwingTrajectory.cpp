#include"SwingTrajectory.h"

double SwingTrajectory::CosineInterpolation(double x0,double xf,double t,double T){
    double desPos = x0;
    if(t>=T){
        desPos = xf;
    }
    else  desPos = (x0+xf)/2-(xf-x0)/2*cos(M_PI*t/T);
    return desPos;
}

double SwingTrajectory::CosineFirstDerivativeInterpolation(double x0,double xf,double t,double T){
    double desVel = 0;
    if(t>=T)     desVel = 0;
    else    desVel = M_PI*(xf-x0)/(2*T)*sin(M_PI*t/T);
    return desVel;
}

double SwingTrajectory::CubicPolynomialInterpolation(double x0,double v0,double xf,double vf,double t,double T){
    double desPos = x0;
    double a,b,c,d;
    if(t>=T)  desPos = xf;
    else{
      d=x0;
      c=v0;
      b=3.0/pow(T,2)*(xf-x0)-2.0/T*v0-1.0/T*vf;
      a=-2.0/pow(T,3)*(xf-x0)+1.0/pow(T,2.0)*(vf+v0);
      desPos = a*pow(t,3)+b*pow(t,2)+c*t+d;
    }
    return desPos;
}

double SwingTrajectory::CubicPolynomialFirstDerivativeInterpolation(double x0,double v0,double xf,double vf,double t,double T){
    double desVel = 0;
    double a,b,c,d;
    if(t>=T)     desVel = 0;
    else{
        d=x0;
        c=v0;
        b=3.0/pow(T,2)*(xf-x0)-2.0/T*v0-1.0/T*vf;
        a=-2.0/pow(T,3)*(xf-x0)+1.0/pow(T,2.0)*(vf+v0);
        desVel = 3*a*pow(t,2)+2*b*t+c;
    }
    return desVel;
}

double SwingTrajectory::CubicBezierInterpolation(double x0,double xf,double t,double T){
    double desPos=0;

    return desPos;
}

double SwingTrajectory::CubicBezierFirstDerivativeInterpolation(double x0,double xf,double t,double T){
    double desVel = 0;

    return desVel;
}

double SwingTrajectory::CubicBezierSecondDerivativeInterpolation(double x0,double xf,double t,double T){
    double desAcc = 0;

    return desAcc;
}

void SwingTrajectory::CosineInterpolation(double x0,double xf,double t,double T,double& desPos,double& desVel){
    
    if(t>=T){
        desPos = xf;
        desVel = 0;
    }
    else{
        desPos = (x0+xf)/2-(xf-x0)/2*cos(M_PI*t/T);
        desVel = M_PI*(xf-x0)/(2*T)*sin(M_PI*t/T);
    }
}

void SwingTrajectory::CubicPolynomialInterpolation(double x0,double v0,double xf,double vf,double t,double T,double& desPos,double& desVel){
    double a,b,c,d;
    if(t>=T){
        desPos = xf;
        desVel = 0;
    }
    else{
        d=x0;
        c=v0;
        b=3.0/pow(T,2)*(xf-x0)-2.0/T*v0-1.0/T*vf;
        a=-2.0/pow(T,3)*(xf-x0)+1.0/pow(T,2.0)*(vf+v0);
        desPos = a*pow(t,3)+b*pow(t,2)+c*t+d;
        desVel = 3*a*pow(t,2)+2*b*t+c;
    }
}

void SwingTrajectory::CubicBezierInterpolation(double x0,double xf,double t,double T,double& desPos,double& desVel,double& desAcc){
    return;
}

void SwingTrajectory::LinerInterPolation(double x0,double xf,double t,double T,double& desPos,double& desVel){
    if(t>=T){
        desPos = xf;
        desVel = 0;
    }
    else{
        desPos = x0+(xf-x0)*t/T;
        desVel = (xf-x0)/T;
    }
}

void SwingTrajectory::SwingLegCubicInterpolation(Vec3 init_pos,Vec3 init_vel,Vec3 target_pos,Vec3 target_vel,double swing_height,double t,double T,Mat3 body_Matrix,Vec3& Des_pos ){
    Des_pos[0]=this->CubicPolynomialInterpolation(init_pos[0],init_vel[0],target_pos[0],target_vel[0],t,T);
    Des_pos[1]=this->CubicPolynomialInterpolation(init_pos[1],init_vel[1],target_pos[1],target_vel[1],t,T);
    if(t<T/2.0){
        Des_pos[2] = this->CubicPolynomialInterpolation(init_pos[2],init_vel[2],init_pos[2]+swing_height,0,t,T/2.0);
    }
    else{
        Des_pos[2] = this->CubicPolynomialInterpolation(init_pos[2]+swing_height,0,target_pos[2],target_vel[2],t-T/2.0,T/2.0);
    }
    Des_pos = body_Matrix.transpose()*Des_pos;
}