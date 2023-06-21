#include"BalanceController.h"

using namespace qpOASES;

BalanceController::BalanceController(/*ControlParameters* _qua*/):qp(NUM_VARIABLES_QP,NUM_CONSTRAINTS_QP){
        // cout<<"ADFAFD"<<std::endl;
        Options options;
        options.printLevel = PL_NONE;
        qp.setOptions(options);
        qp.setPrintLevel(PL_NONE);
        // //  _quaCP = _qua;

        // cout<<"there!"<<std::endl;

        H_mat.resize(NUM_VARIABLES_QP, NUM_VARIABLES_QP);
        A_mat.resize(NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
        g_mat.resize(NUM_VARIABLES_QP, 1);
        lbA_mat.resize(NUM_VARIABLES_QP, 1);
        ubA_mat.resize(NUM_VARIABLES_QP, 1);
        lb_mat.resize(NUM_VARIABLES_QP, 1);
        ub_mat.resize(NUM_VARIABLES_QP, 1);
        // cout<<"there2"<<std::endl;

        S_Control.resize(6,6);
        Q_Control.resize(12,12);
        A_Control.resize(6,12);
        W_Control.resize(12,12);

        // A_Control.setZero();

        S_Control.setIdentity();
        Q_Control.setIdentity();
        W_Control.setIdentity();


        coef_friction = 0.5;
        contact_state << 1,1,1,1;
        alpha = 0.01;
        direction_normal_flatGround<<0,0,1;

        minNormalForces_feet << 10, 10, 10, 10;
        maxNormalForces_feet << 360, 360, 360, 360;

        nWSR_qpOASES = 10000;
        cpu_time = 0.01;

        // std::cout<<"BalanceController Init"<<std::endl;

}

void BalanceController::set_coef_friction(double u){
        coef_friction = u;
}

void BalanceController::set_alpha(double a){
        alpha = a;
}

void BalanceController::set_S_Control(Mat6 S){
        S_Control = S;
}

void BalanceController::set_Q_Control(Mat6 Q){
        Q_Control = Q;
}

void BalanceController::set_Force(Vec4 minF,Vec4 maxF){
        minNormalForces_feet = minF;
        maxNormalForces_feet = maxF;
}

Mat3 BalanceController::Hat_Vec(Vec3 leg_pos){
        Mat3 res;
        res<<0,-leg_pos[2],leg_pos[1],
             leg_pos[2],0,-leg_pos[0],
             -leg_pos[1],leg_pos[0],0;
        return res;
}

void BalanceController::updata_qpSolver_parameters(Mat43 Leg_Pos_World,Vec6 F_Vec,Vec4 contact_state){
        // FL_Leg_Pos_World = Leg_Pos_World.block<1,3>(0,0);
        // HL_Leg_Pos_World = Leg_Pos_World.block<1,3>(1,0);
        // HR_Leg_Pos_World = Leg_Pos_World.block<1,3>(2,0);
        // FR_Leg_Pos_World = Leg_Pos_World.block<1,3>(3,0);

        // cout<<A_Control<<std::endl;
        Mat3 eye3 = Eigen::Matrix<double,3,3>::Identity();
        // std::cout<<"there"<<std::std::endl;

        for(int i=0;i<4;++i){
                // cout<<A_Control<<std::endl;
                A_Control.block<3,3>(0,3*i)<<eye3;
                A_Control.block<3,3>(3,3*i)<<Hat_Vec(Leg_Pos_World.block<1,3>(i,0).transpose());
        }
        // // A_Control.block<3,3>(0,0)<<eye3;
        // // A_Control.block<3,3>(0,3)<<eye3

        H_mat = A_Control.transpose()*S_Control*A_Control + alpha * W_Control;
        g_mat = -A_Control.transpose()*S_Control*F_Vec;

        Vec3 t1x;
        t1x<<1,0,0;
        Vec3 t2y;
        t2y<<0,1,0;
        for(int i=0;i<4;++i){
                // cout<<A_mat<<std::endl;
                // cout<<std::endl;
                A_mat.block<1,3>(5*i+0,3*i) <<-coef_friction * direction_normal_flatGround.transpose() +
               t1x.transpose();
                A_mat.block<1,3>(5*i+1,3*i) <<-coef_friction * direction_normal_flatGround.transpose() +
               t2y.transpose();
                A_mat.block<1,3>(5*i+2,3*i) <<coef_friction * direction_normal_flatGround.transpose() +
               t2y.transpose();
                A_mat.block<1,3>(5*i+3,3*i) <<coef_friction * direction_normal_flatGround.transpose() +
               t1x.transpose();
                A_mat.block<1,3>(5*i+4,3*i) <<direction_normal_flatGround.transpose();
        }

        for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
                for (int j = 0; j < NUM_VARIABLES_PER_FOOT; j++) {
                        lb_qpOASES[NUM_VARIABLES_PER_FOOT * i + j] =
                                contact_state(i) * NEGATIVE_NUMBER;
                        ub_qpOASES[NUM_VARIABLES_PER_FOOT * i + j] =
                                contact_state(i) * POSITIVE_NUMBER;
                }
        }

        for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
                lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i] =
                        contact_state(i) * NEGATIVE_NUMBER;
                lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 1] =
                        contact_state(i) * NEGATIVE_NUMBER;
                lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 2] = 0;
                lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 3] = 0;
                lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 4] =
                        contact_state(i) * minNormalForces_feet(i);

                ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i] = 0;
                ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 1] = 0;
                ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 2] =
                        contact_state(i) * POSITIVE_NUMBER;
                ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 3] =
                        contact_state(i) * POSITIVE_NUMBER;
                ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 4] =
                        contact_state(i) * maxNormalForces_feet(i);
        }

}

void BalanceController::qp_solver(double* fOpt){
        
        copy_Eigen_to_real_t(A_qpOASES,A_mat, NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
        copy_Eigen_to_real_t(H_qpOASES, H_mat, NUM_VARIABLES_QP, NUM_VARIABLES_QP);       
        copy_Eigen_to_real_t(g_qpOASES, g_mat, NUM_VARIABLES_QP, 1);        
        returnValue qpfinished;
        // qpfinished = qp.init(H_qpOASES,g_qpOASES,A_qpOASES,lb_qpOASES,ub_qpOASES,lbA_qpOASES,ubA_qpOASES,nWSR_qpOASES,&cpu_time);
        // QProblem example( 12,20 );
        // qpfinished = example.init(H_qpOASES,g_qpOASES,A_qpOASES,lb_qpOASES,ub_qpOASES,lbA_qpOASES,ubA_qpOASES,nWSR_qpOASES);

        nWSR_qpOASES = 100;//attention here!!!
        qpfinished = qp.init(H_qpOASES,g_qpOASES,A_qpOASES,lb_qpOASES,ub_qpOASES,lbA_qpOASES,ubA_qpOASES,nWSR_qpOASES,&cpu_time);
        // print_QPData();
        if(qpfinished==SUCCESSFUL_RETURN){
                qp.getPrimalSolution(xOpt_qpOASES);
                qp.getDualSolution(yOpt_qpOASES);
                for(int i=0;i<12;++i){
                        fOpt[i] = xOpt_qpOASES[i]; 
                        // std::cout<<fOpt[i]<<std::endl;
                }
                // std::cout<<std::endl;
        } 
        else if(qpfinished==RET_INIT_FAILED){
                std::cout<<"QP Init Error!!!"<<std::endl;
        }
        else{
                std::cout<<"QPSOLVERERROR"<<std::endl;
        }

}




void BalanceController::copy_Eigen_to_real_t(real_t* target,
                                             Eigen::MatrixXd& source, int nRows,
                                             int nCols) {
  int count = 0;

  for (int i = 0; i < nRows; i++) {
    for (int j = 0; j < nCols; j++) {
      target[count] = source(i, j);
      count++;
    }
  }
}

void BalanceController::print_real_t(real_t* matrix, int nRows, int nCols) {
  int count = 0;
  for (int i = 0; i < nRows; i++) {
    for (int j = 0; j < nCols; j++) {
      std::cout << matrix[count] << "\t";
      count++;
    }
    std::cout << "\n";
  }
}

void BalanceController::print_QPData() {
  std::cout << "\n\n";
  std::cout << "\n\nH = ";
  print_real_t(H_qpOASES, NUM_VARIABLES_QP, NUM_VARIABLES_QP);
  std::cout << "\n\nA = ";
  print_real_t(A_qpOASES, NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
  std::cout << "\n\ng = ";
  print_real_t(g_qpOASES, NUM_VARIABLES_QP, 1);
  std::cout << "\n\nlb = ";
  print_real_t(lb_qpOASES, NUM_VARIABLES_QP, 1);
  std::cout << "\n\nub = ";
  print_real_t(ub_qpOASES, NUM_VARIABLES_QP, 1);
  std::cout << "\n\nlbA = ";
  print_real_t(lbA_qpOASES, NUM_CONSTRAINTS_QP, 1);
  std::cout << "\n\nubA = ";
  print_real_t(ubA_qpOASES, NUM_CONSTRAINTS_QP, 1);
}