#ifndef _BALANCECONTROLLER_H_
#define _BALANCECONTROLLER_H_

#include"Types.h"
#include <qpOASES.hpp>
#include"ControlParameters.h"
#include<iostream>

static const int NUM_VARIABLES_QP = 12;
static const int NUM_CONSTRAINTS_QP = 20;
static const int NUM_CONTACT_POINTS = 4;
static const int NUM_VARIABLES_PER_FOOT = 3;
static const int NUM_CONSTRAINTS_PER_FOOT = 5;

static const double NEGATIVE_NUMBER = -1000000.0;
static const double POSITIVE_NUMBER = 1000000.0;
// using namespace std;
using namespace qpOASES;
using namespace Eigen;

class BalanceController{
    // USING_NAMESPACE_QPOASES
public:
    BalanceController(/*ControlParameters* _qua*/);
    ~BalanceController(){}

    // ControlParameters* _quaCP;

    void print_QPData();

    void set_coef_friction(double u);
    void set_alpha(double a);
    void set_S_Control(Mat6 S);
    void set_Q_Control(Mat6 Q);

    void set_Force(Vec4 minF,Vec4 maxF);

    void updata_qpSolver_parameters(Mat43 Leg_Pos_World,Vec6 F_Vec,Vec4 contact_state);

    void qp_solver(double* fOpt);

    // QProblem qp;


private:

    QProblem qp;

    real_t H_qpOASES[NUM_VARIABLES_QP * NUM_VARIABLES_QP];
    real_t A_qpOASES[NUM_CONSTRAINTS_QP * NUM_VARIABLES_QP];
    real_t g_qpOASES[NUM_VARIABLES_QP];
    real_t lb_qpOASES[NUM_VARIABLES_QP];
    real_t ub_qpOASES[NUM_VARIABLES_QP];
    real_t lbA_qpOASES[NUM_CONSTRAINTS_QP];
    real_t ubA_qpOASES[NUM_CONSTRAINTS_QP];
    real_t xOpt_qpOASES[NUM_VARIABLES_QP];
    real_t yOpt_qpOASES[NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP];


    Eigen::MatrixXd H_mat;
    Eigen::MatrixXd g_mat;
    Eigen::MatrixXd A_mat;
    Eigen::MatrixXd lbA_mat;
    Eigen::MatrixXd ubA_mat;
    Eigen::MatrixXd lb_mat;
    Eigen::MatrixXd ub_mat;

    Eigen::MatrixXd S_Control;//第一项权重
    Eigen::MatrixXd Q_Control;//与前一状态之差的权重
    Eigen::MatrixXd A_Control;
    Eigen::MatrixXd W_Control;//自身大小的权重

    double coef_friction;
    double alpha;
    Vec4 contact_state;
    Vec3 direction_normal_flatGround;

    Vec3 FL_Leg_Pos_World;
    Vec3 HL_Leg_Pos_World;
    Vec3 HR_Leg_Pos_World;
    Vec3 FR_Leg_Pos_World;

    Vec4   minNormalForces_feet;
    Vec4   maxNormalForces_feet;

    Mat3 Hat_Vec(Vec3 leg_pos);

    int_t nWSR_qpOASES;
    real_t cpu_time;

    







    void copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd& source, int nRows,int nCols);
    void print_real_t(real_t* matrix, int nRows, int nCols);

    // void calc_H_qpOASES();
    // void calc_A_qpOASES();
    // void calc_g_qpOASES();
    // void calc_lb_ub_qpOASES();
    // void calc_lbA_ubA_qpOASES();

};


#endif