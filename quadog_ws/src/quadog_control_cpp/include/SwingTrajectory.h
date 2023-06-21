#ifndef _SWINGTRAJECTORY_H_
#define _SWINGTRAJECTORY_H_

#include "Types.h"
#include<cmath>

class SwingTrajectory
{
public:
    double CosineInterpolation(double x0,double xf,double t,double T);
    double CosineFirstDerivativeInterpolation(double x0,double xf,double t,double T);

    double CubicPolynomialInterpolation(double x0,double v0,double xf,double vf,double t,double T);
    double CubicPolynomialFirstDerivativeInterpolation(double x0,double v0,double xf,double vf,double t,double T);
    
    double CubicBezierInterpolation(double x0,double xf,double t,double T);
    double CubicBezierFirstDerivativeInterpolation(double x0,double xf,double t,double T);
    double CubicBezierSecondDerivativeInterpolation(double x0,double xf,double t,double T);

    void CosineInterpolation(double x0,double xf,double t,double T,double& desPos,double& desVel);
    void CubicPolynomialInterpolation(double x0,double v0,double xf,double vf,double t,double T,double& desPos,double& desVel);
    void CubicBezierInterpolation(double x0,double xf,double t,double T,double& desPos,double& desVel,double& desAcc);

    void LinerInterPolation(double x0,double xf,double t,double T,double& desPos,double& desVel);

    void SwingLegCubicInterpolation(Vec3 init_pos,Vec3 init_vel,Vec3 target_pos,Vec3 target_vel,double swing_height,double t,double T,Mat3 body_Matrix,Vec3& Des_pos );
public:
    SwingTrajectory(){

    }
    ~SwingTrajectory(){
        
    }

    
};


#endif