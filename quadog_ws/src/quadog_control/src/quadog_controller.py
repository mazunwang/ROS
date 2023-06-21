#!/usr/bin/env python
#coding=utf-8

# import sys
# import copy
import rospy
import numpy as np
import math
# import moveit_commander
# import moveit_msgs.msg
from cvxopt import solvers, matrix
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock


class QuaDogController(object):
  def __init__(self):

    super(QuaDogController, self).__init__()

    rospy.init_node('quadog_controller_node',
                    anonymous=True)
    self.joint1_tor_pub = rospy.Publisher('/quadog/joint1_effort_controller/command', Float64, queue_size=10)
    self.joint2_tor_pub = rospy.Publisher('/quadog/joint2_effort_controller/command', Float64, queue_size=10)
    self.joint3_tor_pub = rospy.Publisher('/quadog/joint3_effort_controller/command', Float64, queue_size=10)
    self.joint4_tor_pub = rospy.Publisher('/quadog/joint4_effort_controller/command', Float64, queue_size=10)
    self.joint5_tor_pub = rospy.Publisher('/quadog/joint5_effort_controller/command', Float64, queue_size=10)
    self.joint6_tor_pub = rospy.Publisher('/quadog/joint6_effort_controller/command', Float64, queue_size=10)
    self.joint7_tor_pub = rospy.Publisher('/quadog/joint7_effort_controller/command', Float64, queue_size=10)
    self.joint8_tor_pub = rospy.Publisher('/quadog/joint8_effort_controller/command', Float64, queue_size=10)
    self.joint9_tor_pub = rospy.Publisher('/quadog/joint9_effort_controller/command', Float64, queue_size=10)
    self.joint10_tor_pub = rospy.Publisher('/quadog/joint10_effort_controller/command', Float64, queue_size=10)
    self.joint11_tor_pub = rospy.Publisher('/quadog/joint11_effort_controller/command', Float64, queue_size=10)
    self.joint12_tor_pub = rospy.Publisher('/quadog/joint12_effort_controller/command', Float64, queue_size=10)
     
    self.step_num = 40 
    self.rate = 1000
   
    self.body_width = 0.272
    self.body_length = 0.635

    self.l1 = 0.037 
    self.l2 = 0.3
    self.l3 = 0.3

    self.center_correct = 0.075 
    self.v_linear = 0.0
    self.v_angular = 0.0

    self.state = 'INIT_STATE'
    self.orientation = np.array([0.0, 0.0, 0.0])
    self.body_acc_vec = np.array([0.0, 0.0, 0.0]).T
    self.body_angular_vel = np.array([0.0, 0.0, 0.0])
    self.cnt = 0
    self.body_matrix = np.zeros((3,3))

    Kp_tmp = 800
    Kd_tmp = 10
    self.t=0

    self.FL_HipX_Kp = Kp_tmp
    self.FL_HipX_Kd = Kd_tmp
    self.FL_HipY_Kp = Kp_tmp
    self.FL_HipY_Kd = Kd_tmp
    self.FL_Knee_Kp = Kp_tmp
    self.FL_Knee_Kd = Kd_tmp
    self.HL_HipX_Kp = Kp_tmp
    self.HL_HipX_Kd = Kd_tmp
    self.HL_HipY_Kp = Kp_tmp
    self.HL_HipY_Kd = Kd_tmp
    self.HL_Knee_Kp = Kp_tmp
    self.HL_Knee_Kd = Kd_tmp
    self.FR_HipX_Kp = Kp_tmp
    self.FR_HipX_Kd = Kd_tmp
    self.FR_HipY_Kp = Kp_tmp
    self.FR_HipY_Kd = Kd_tmp
    self.FR_Knee_Kp = Kp_tmp
    self.FR_Knee_Kd = Kd_tmp
    self.HR_HipX_Kp = Kp_tmp
    self.HR_HipX_Kd = Kd_tmp
    self.HR_HipY_Kp = Kp_tmp
    self.HR_HipY_Kd = Kd_tmp
    self.HR_Knee_Kp = Kp_tmp
    self.HR_Knee_Kd = Kd_tmp

    self.HL_HipX_Pos = 0
    self.HL_HipY_Pos = 0
    self.HL_Knee_Pos = 0
    self.FL_HipX_Pos = 0
    self.FL_HipY_Pos = 0
    self.FL_Knee_Pos = 0
    self.HR_HipX_Pos = 0
    self.HR_HipY_Pos = 0
    self.HR_Knee_Pos = 0
    self.FR_HipX_Pos = 0
    self.FR_HipY_Pos = 0
    self.FR_Knee_Pos = 0

    self.HL_HipX_Vel = 0
    self.HL_HipY_Vel = 0
    self.HL_Knee_Vel = 0
    self.FL_HipX_Vel = 0
    self.FL_HipY_Vel = 0
    self.FL_Knee_Vel = 0
    self.HR_HipX_Vel = 0
    self.HR_HipY_Vel = 0
    self.HR_Knee_Vel = 0
    self.FR_HipX_Vel = 0
    self.FR_HipY_Vel = 0
    self.FR_Knee_Vel = 0

    self.HL_HipX_Torq = 0
    self.HL_HipY_Torq = 0
    self.HL_Knee_Torq = 0
    self.FL_HipX_Torq = 0
    self.FL_HipY_Torq = 0
    self.FL_Knee_Torq = 0
    self.HR_HipX_Torq = 0
    self.HR_HipY_Torq = 0
    self.HR_Knee_Torq = 0
    self.FR_HipX_Torq = 0
    self.FR_HipY_Torq = 0
    self.FR_Knee_Torq = 0

    self.flag='true'

    self.Init_Leg_Pos = np.zeros((4,3))#逆时针 lf,lb,rb,rf
    self.Init_Joint_Pos = np.zeros((4,3))

    self.Init_LF_Leg_Pos = np.array([0,0,0])
    
    self.Stance_Torq = np.zeros((4,3))
    
    self.time_start = rospy.get_time()
    self.time_start_ros = rospy.get_time()

    print('四足机器人初始化成功')
    return
     
  def model_setup(self):
    print("选择机器人的控制模式，'t'为键盘控制，'a'为自动导航")
    model_type = input()
    return model_type  

  def Init_Value(self):
    theta_lf = np.array([self.FL_HipX_Pos,self.FL_HipY_Pos,self.FL_Knee_Pos])
    theta_rf = np.array([self.FR_HipX_Pos,self.FR_HipY_Pos,self.FR_Knee_Pos])
    theta_lb = np.array([self.HL_HipX_Pos,self.HL_HipY_Pos,self.HL_Knee_Pos])
    theta_rb = np.array([self.HR_HipX_Pos,self.HR_HipY_Pos,self.HR_Knee_Pos])
    self.Init_Joint_Pos[0,0:3]=theta_lf
    self.Init_Joint_Pos[1,0:3]=theta_lb
    self.Init_Joint_Pos[2,0:3]=theta_rb
    self.Init_Joint_Pos[3,0:3]=theta_rf
    pos_lf = self.leg_FK_Cal(theta_lf,'Lf')
    pos_rf = self.leg_FK_Cal(theta_rf,'Rf')
    pos_lb = self.leg_FK_Cal(theta_lb,'Lb')
    pos_rb = self.leg_FK_Cal(theta_rb,'Rb')
    self.Init_Leg_Pos[0,0],self.Init_Leg_Pos[0,1],self.Init_Leg_Pos[0,2] = pos_lf[0],pos_lf[1],pos_lf[2]
    self.Init_Leg_Pos[1,0],self.Init_Leg_Pos[1,1],self.Init_Leg_Pos[1,2] = pos_lb[0],pos_lb[1],pos_lb[2]
    self.Init_Leg_Pos[2,0],self.Init_Leg_Pos[2,1],self.Init_Leg_Pos[2,2] = pos_rb[0],pos_rb[1],pos_rb[2]
    self.Init_Leg_Pos[3,0],self.Init_Leg_Pos[3,1],self.Init_Leg_Pos[3,2] = pos_rf[0],pos_rf[1],pos_rf[2]
    return

  def Para_Change(self,Kp,Kd):
    Kp_tmp = Kp
    Kd_tmp = Kd
    self.FL_HipX_Kp = Kp_tmp
    self.FL_HipX_Kd = Kd_tmp
    self.FL_HipY_Kp = Kp_tmp
    self.FL_HipY_Kd = Kd_tmp
    self.FL_Knee_Kp = Kp_tmp
    self.FL_Knee_Kd = Kd_tmp
    self.HL_HipX_Kp = Kp_tmp
    self.HL_HipX_Kd = Kd_tmp
    self.HL_HipY_Kp = Kp_tmp
    self.HL_HipY_Kd = Kd_tmp
    self.HL_Knee_Kp = Kp_tmp
    self.HL_Knee_Kd = Kd_tmp
    self.FR_HipX_Kp = Kp_tmp
    self.FR_HipX_Kd = Kd_tmp
    self.FR_HipY_Kp = Kp_tmp
    self.FR_HipY_Kd = Kd_tmp
    self.FR_Knee_Kp = Kp_tmp
    self.FR_Knee_Kd = Kd_tmp
    self.HR_HipX_Kp = Kp_tmp
    self.HR_HipX_Kd = Kd_tmp
    self.HR_HipY_Kp = Kp_tmp
    self.HR_HipY_Kd = Kd_tmp
    self.HR_Knee_Kp = Kp_tmp
    self.HR_Knee_Kd = Kd_tmp
    return


  def Move_To_Cubic_Planning(self,x0,v0,xf,vf,t,T):
    desPos = x0
    desVel = 0
    if(t>=T):
      desPos = xf
      desVel = 0
    else:
      d=x0
      c=v0
      b=3.0/math.pow(T,2)*(xf-x0)-2.0/T*v0-1.0/T*vf
      a=-2.0/math.pow(T,3)*(xf-x0)+1.0/math.pow(T,2.0)*(vf+v0)
      desPos = a*pow(t,3)+b*pow(t,2)+c*t+d
      desVel = 3*a*pow(t,2)+2*b*t+c
    return desPos,desVel

  def Move_To_Target_Pos_EndPoint_Planning(self,target_Pos,t,T,Leg_Name):
    Init_End_Pos = np.array([0.0,0.0,0.0])
    Init_End_Vel = np.array([0.0,0.0,0.0])
    if(Leg_Name=='Lf'):
      Init_End_Pos = self.Init_Leg_Pos[0,0:3]
    elif(Leg_Name=='Lb'):
      Init_End_Pos = self.Init_Leg_Pos[1,0:3]
    elif(Leg_Name=='Rb'):
      Init_End_Pos = self.Init_Leg_Pos[2,0:3]
    elif(Leg_Name=='Rf'):
      Init_End_Pos = self.Init_Leg_Pos[3,0:3]
    else:
      print('Move_Leg_Name Error')
    desPos = np.array([0.0,0.0,0.0])
    desVel = np.array([0.0,0.0,0.0])
    for i in range(3):
      desPos[i],desVel[i]=self.Move_To_Cubic_Planning(Init_End_Pos[i],0.0,target_Pos[i],0,t,T)
    return desPos,desVel
  
  def Move_To_Target_Pos_Joint_Planning(self,target_Pos,t,T,Leg_Name):
    Init_Angle_Vec = np.array([0.0,0.0,0.0])
    if(Leg_Name=='Lf'):
      Init_Angle_Vec = self.Init_Joint_Pos[0,0:3]
    elif(Leg_Name=='Lb'):
      Init_Angle_Vec = self.Init_Joint_Pos[1,0:3]
    elif(Leg_Name=='Rb'):
      Init_Angle_Vec = self.Init_Joint_Pos[2,0:3]
    elif(Leg_Name=='Rf'):
      Init_Angle_Vec = self.Init_Joint_Pos[3,0:3]
    else:
      print('Move_Leg_Name Error')
    TargetJointPos = self.leg_IK_Cal(target_Pos,Leg_Name)
    #print(TargetJointPos)
    desJointAngle = np.array([0.0,0.0,0.0])
    desJointVel = np.array([0.0,0.0,0.0])
    for i in range(3):
      desJointAngle[i],desJointVel[i]=self.Move_To_Cubic_Planning(Init_Angle_Vec[i],0.0,TargetJointPos[i],0,t,T)
    #Torq = self.Joint_Pos_Control(desJointAngle,desVel,Leg_Name)
    #print(desJointAngle)
    return desJointAngle,desJointVel

  def pre_state_plan(self):
    gait_data = np.zeros((1, 12))
    _t=self.t-self.time_start
    T=0.8
    #self.cnt+=1
    #print(_t,rospy.get_time()-self.time_start_ros,self.cnt)
    #print(t)
    #self.Init_Value()
    #print(self.Init_Leg_Pos)
    #print(self.Init_Joint_Pos)
    #print(time.time()*1000)
    #print(rospy.get_time())
    lf_target_pos = np.array([0.3175,0.173,-0.15])
    lb_target_pos = np.array([-0.3175,0.173,-0.15])
    rb_target_pos = np.array([-0.3175,-0.173,-0.15])
    rf_target_pos = np.array([0.3175,-0.173,-0.15])
    if(_t<=1.2*T and self.flag=='true'):
      lf_target_pos = np.array([0.3175,0.173,-0.15])
      lb_target_pos = np.array([-0.3175,0.173,-0.15])
      rb_target_pos = np.array([-0.3175,-0.173,-0.15])
      rf_target_pos = np.array([0.3175,-0.173,-0.15])
    elif(self.flag=='false' and _t<1.1*T):
      lf_target_pos = np.array([0.3175,0.173,-0.45])
      lb_target_pos = np.array([-0.3175,0.173,-0.45])
      rb_target_pos = np.array([-0.3175,-0.173,-0.45])
      rf_target_pos = np.array([0.3175,-0.173,-0.45])
      #print('IMU',self.orientation)
    elif(self.flag=='false' ):
      self.time_start = self.t
      self.Init_Value()
      self.flag = 'false'
      self.state = 'STANCE_STATE'
      self.Stance_Torq[0,0:3] = self.FL_HipX_Torq,self.FL_HipY_Torq,self.FL_Knee_Torq
      self.Stance_Torq[1,0:3] = self.HL_HipX_Torq,self.HL_HipY_Torq,self.HL_Knee_Torq
      self.Stance_Torq[2,0:3] = self.HR_HipX_Torq,self.HR_HipY_Torq,self.HR_Knee_Torq
      self.Stance_Torq[3,0:3] = self.FR_HipX_Torq,self.FR_HipY_Torq,self.FR_Knee_Torq
      print('STANCE_STATE')
      print('IMU',self.orientation)
    else:
      #self.time_start = rospy.get_time()
      self.time_start = self.t
      self.Init_Value()
      self.flag = 'false'
      lf_target_pos = np.array([0.3175,0.173,-0.45])
      lb_target_pos = np.array([-0.3175,0.173,-0.45])
      rb_target_pos = np.array([-0.3175,-0.173,-0.45])
      rf_target_pos = np.array([0.3175,-0.173,-0.45])
      #self.state = 'STANCE_STATE'
      #print('Stance State!!!')
      print(self.Init_Leg_Pos)
    theta_lf = np.array([self.FL_HipX_Pos,self.FL_HipY_Pos,self.FL_Knee_Pos])
    theta_rf = np.array([self.FR_HipX_Pos,self.FR_HipY_Pos,self.FR_Knee_Pos])
    theta_lb = np.array([self.HL_HipX_Pos,self.HL_HipY_Pos,self.HL_Knee_Pos])
    theta_rb = np.array([self.HR_HipX_Pos,self.HR_HipY_Pos,self.HR_Knee_Pos])

    anglurvel_lf = np.array([self.FL_HipX_Vel,self.FL_HipY_Vel,self.FL_Knee_Vel])
    anglurvel_rf = np.array([self.FR_HipX_Vel,self.FR_HipY_Vel,self.FR_Knee_Vel])
    anglurvel_lb = np.array([self.HL_HipX_Vel,self.HL_HipY_Vel,self.HL_Knee_Vel])
    anglurvel_rb = np.array([self.HR_HipX_Vel,self.HR_HipY_Vel,self.HR_Knee_Vel])

    pos_lf = self.leg_FK_Cal(theta_lf,'Lf')
    pos_rf = self.leg_FK_Cal(theta_rf,'Rf')
    pos_lb = self.leg_FK_Cal(theta_lb,'Lb')
    pos_rb = self.leg_FK_Cal(theta_rb,'Rb')

    vel_lf = self.Get_Velocity(theta_lf,anglurvel_lf,'Lf')
    vel_rf = self.Get_Velocity(theta_rf,anglurvel_rf,'Rf')
    vel_lb = self.Get_Velocity(theta_lb,anglurvel_lb,'Lb')
    vel_rb = self.Get_Velocity(theta_rb,anglurvel_rb,'Rb')
    
    LF_Pos_Foot_World = np.dot(self.body_matrix,pos_lf.T) 
    LB_Pos_Foot_World = np.dot(self.body_matrix,pos_lb.T) 
    RB_Pos_Foot_World = np.dot(self.body_matrix,pos_rb.T) 
    RF_Pos_Foot_World = np.dot(self.body_matrix,pos_rf.T) 

    LF_Vel_Foot_World = np.dot(self.body_matrix,vel_lf.T)
    LB_Vel_Foot_World = np.dot(self.body_matrix,vel_lb.T)
    RB_Vel_Foot_World = np.dot(self.body_matrix,vel_rb.T)
    RF_Vel_Foot_World = np.dot(self.body_matrix,vel_rf.T)
    print(RB_Vel_Foot_World)
    lf_desJointAngle,lf_desJointVel = self.Move_To_Target_Pos_Joint_Planning(lf_target_pos,_t,T,'Lf')
    lb_desJointAngle,lb_desJointVel = self.Move_To_Target_Pos_Joint_Planning(lb_target_pos,_t,T,'Lb')
    Rb_desJointAngle,rb_desJointVel = self.Move_To_Target_Pos_Joint_Planning(rb_target_pos,_t,T,'Rb')
    Rf_desJointAngle,rf_desJointVel = self.Move_To_Target_Pos_Joint_Planning(rf_target_pos,_t,T,'Rf')
    gait_data[0,0:3] = self.Joint_Pos_Control(lf_desJointAngle,lf_desJointVel,'Lf')
    gait_data[0,3:6] = self.Joint_Pos_Control(lb_desJointAngle,lb_desJointVel,'Lb')
    gait_data[0,6:9] = self.Joint_Pos_Control(Rb_desJointAngle,rb_desJointVel,'Rb')
    gait_data[0,9:12] = self.Joint_Pos_Control(Rf_desJointAngle,rf_desJointVel,'Rf')
    """
    lf_desEndPos = self.Move_To_Target_Pos_EndPoint_Planning(lf_target_pos,_t,T,'Lf')
    lb_desEndPos = self.Move_To_Target_Pos_EndPoint_Planning(lb_target_pos,_t,T,'Lb')
    rb_desEndPos = self.Move_To_Target_Pos_EndPoint_Planning(rb_target_pos,_t,T,'Rb')
    rf_desEndPos = self.Move_To_Target_Pos_EndPoint_Planning(rf_target_pos,_t,T,'Rf')
    #print(lf_desEndPos)
    #print(self.Init_Leg_Pos)
    lf_desJointAngle = self.leg_IK_Cal(lf_desEndPos,'Lf')
    lb_desJointAngle = self.leg_IK_Cal(lb_desEndPos,'Lb')
    rb_desJointAngle = self.leg_IK_Cal(rb_desEndPos,'Rb')
    rf_desJointAngle = self.leg_IK_Cal(rf_desEndPos,'Rf')
    gait_data[0,0:3] = self.Joint_Pos_Control(lf_desJointAngle,desVel,'Lf')
    gait_data[0,3:6] = self.Joint_Pos_Control(lb_desJointAngle,desVel,'Lb')
    gait_data[0,6:9] = self.Joint_Pos_Control(rb_desJointAngle,desVel,'Rb')
    gait_data[0,9:12] = self.Joint_Pos_Control(rf_desJointAngle,desVel,'Rf')
    """
    #print(self.leg_FK_Cal([self.FL_HipX_Pos,self.FL_HipY_Pos,self.FL_Knee_Pos],'Lf'))
    #print(self.leg_FK_Cal([self.HL_HipX_Pos,self.HL_HipY_Pos,self.HL_Knee_Pos],'Lb'))
    #print(self.leg_FK_Cal([self.HR_HipX_Pos,self.HR_HipY_Pos,self.HR_Knee_Pos],'Rb'))
    #print(self.leg_FK_Cal([self.FR_HipX_Pos,self.FR_HipY_Pos,self.FR_Knee_Pos],'Rf'))
    #gait_data = np.zeros((1, 12))
    return gait_data

  def Swing_Leg_Traj(self,Init_Pos,Target_Pos,t,T,Leg_Name):
    h=-0.40
    if(t<=T):
      x=Init_Pos[0]*(1-t/T)+Target_Pos[0]*(t/T)
      y=Init_Pos[1]*(1-t/T)+Target_Pos[1]*(t/T)
      if(t/T<=0.5):
        z=(h+Init_Pos[2])/2+(Init_Pos[2]-h)/2*math.cos(t/T*2*3.1415)
      else:
        z=(h+Target_Pos[2])/2+(Target_Pos[2]-h)/2*math.cos(t/T*2*3.1415)
    else:
      x,y,z=Target_Pos[0],Target_Pos[1],Target_Pos[2]
    #print(x,y,z)
    TargetJointPos = self.leg_IK_Cal(np.array([x,y,z]),Leg_Name)
    return TargetJointPos
      


  def stance_state_plan(self):
    gait_data = np.zeros((1, 12))


    _t=self.t-self.time_start
    T=1.0
    #self.cnt+=1
    #print(self.t,self.cnt)
    x_bias = 0.00
    y_bias = 0.00
    Touching_Pos = np.array([0.3175+0.08,0.173,-0.45])
    lf_target_pos = np.array([self.Init_Leg_Pos[0][0]+x_bias,self.Init_Leg_Pos[0][1]+y_bias,-0.45])
    lb_target_pos = np.array([self.Init_Leg_Pos[1][0]+x_bias,self.Init_Leg_Pos[1][1]+y_bias,-0.45])
    rb_target_pos = np.array([self.Init_Leg_Pos[2][0]+x_bias,self.Init_Leg_Pos[2][1]+y_bias,-0.45])
    rf_target_pos = np.array([self.Init_Leg_Pos[3][0]+x_bias,self.Init_Leg_Pos[3][1]+y_bias,-0.45])
    lf_desJointAngle,lf_desJointVel = self.Move_To_Target_Pos_Joint_Planning(lf_target_pos,_t,T,'Lf')
    lb_desJointAngle,lb_desJointVel = self.Move_To_Target_Pos_Joint_Planning(lb_target_pos,_t,T,'Lb')
    Rb_desJointAngle,rb_desJointVel = self.Move_To_Target_Pos_Joint_Planning(rb_target_pos,_t,T,'Rb')
    Rf_desJointAngle,rf_desJointVel = self.Move_To_Target_Pos_Joint_Planning(rf_target_pos,_t,T,'Rf')
    gait_data[0,0:3] = self.Joint_Pos_Control(lf_desJointAngle,lf_desJointVel,'Lf')+self.Stance_Torq[0,0:3]
    gait_data[0,3:6] = self.Joint_Pos_Control(lb_desJointAngle,lb_desJointVel,'Lb')+self.Stance_Torq[1,0:3]
    gait_data[0,6:9] = self.Joint_Pos_Control(Rb_desJointAngle,rb_desJointVel,'Rb')+self.Stance_Torq[2,0:3]
    gait_data[0,9:12] = self.Joint_Pos_Control(Rf_desJointAngle,rf_desJointVel,'Rf')+self.Stance_Torq[3,0:3]
    if(_t>=1.2*T):
      LF_TargetJointPos = self.Swing_Leg_Traj(self.Init_LF_Leg_Pos,Touching_Pos,_t-1.2*T,T,'Lf')
      #gait_data[0,0:3] = self.Joint_Pos_Control(LF_TargetJointPos,desVel,'Lf')
    else:
      #self.time_start = self.t
      self.Init_LF_Leg_Pos = self.leg_FK_Cal(np.array([self.FL_HipX_Pos,self.FL_HipY_Pos,self.FL_Knee_Pos]),'Lf')
      #print(_t,self.Init_LF_Leg_Pos)
      #print('start step')
      #print(self.Init_Leg_Pos)
      #self.Stance_Torq[0:4,2] *=1.3333
    LF_Torq_Vec = np.array([[self.FL_HipX_Torq],[self.FL_HipY_Torq],[self.FL_Knee_Torq]])
    LF_Jacobi = self.Leg_Jacobi_Mat_Cal('Lf')
    #print(np.dot(np.linalg.inv(LF_Jacobi),LF_Torq_Vec))

    LF_Jacobi = self.Leg_Jacobi_Mat_Cal('Lf')
    RF_Jacobi = self.Leg_Jacobi_Mat_Cal('Rf')
    LB_Jacobi = self.Leg_Jacobi_Mat_Cal('Lb')
    RB_Jacobi = self.Leg_Jacobi_Mat_Cal('Rb')

    LF_Torq_Vec = np.array([[self.FL_HipX_Torq],[self.FL_HipY_Torq],[self.FL_Knee_Torq]])
    LB_Torq_Vec = np.array([[self.HL_HipX_Torq],[self.HL_HipY_Torq],[self.HL_Knee_Torq]])
    RB_Torq_Vec = np.array([[self.HR_HipX_Torq],[self.HR_HipY_Torq],[self.HR_Knee_Torq]])
    RF_Torq_Vec = np.array([[self.FR_HipX_Torq],[self.FR_HipY_Torq],[self.FR_Knee_Torq]])

    FL_HipX_Vec = np.array([self.body_length/2,self.body_width/2,0])
    FR_HipX_Vec = np.array([self.body_length/2,-self.body_width/2,0])
    HL_HipX_Vec = np.array([-self.body_length/2,self.body_width/2,0])
    HR_HipX_Vec = np.array([-self.body_length/2,-self.body_width/2,0])

    FL_HipX_Vec_World = np.dot(self.body_matrix,FL_HipX_Vec.T)
    FR_HipX_Vec_World = np.dot(self.body_matrix,FR_HipX_Vec.T)
    HL_HipX_Vec_World = np.dot(self.body_matrix,HL_HipX_Vec.T)
    HR_HipX_Vec_World = np.dot(self.body_matrix,HR_HipX_Vec.T)

    A_mat = matrix(np.array([[1.0,0,0,1.0,0,0,1.0,0,0,1.0,0,0],
                             [0,1.0,0,0,1.0,0,0,1.0,0,0,1.0,0],
                             [0,0,1.0,0,0,1.0,0,0,1.0,0,0,1.0],
                             [0,-FL_HipX_Vec_World[2],FL_HipX_Vec_World[1], 0,-HL_HipX_Vec_World[2],HL_HipX_Vec_World[1], 0,-HR_HipX_Vec_World[2],HR_HipX_Vec_World[1], 0,-FR_HipX_Vec_World[2],FR_HipX_Vec_World[1]],
                             [FL_HipX_Vec_World[2],0,-FL_HipX_Vec_World[0], HL_HipX_Vec_World[2],0,-HL_HipX_Vec_World[0], HR_HipX_Vec_World[2],0,-HR_HipX_Vec_World[0], FR_HipX_Vec_World[2],0,-FR_HipX_Vec_World[0]],
                             [-FL_HipX_Vec_World[1],FL_HipX_Vec_World[0],0, -HL_HipX_Vec_World[1],HL_HipX_Vec_World[0],0, -HR_HipX_Vec_World[1],HR_HipX_Vec_World[0],0, -FR_HipX_Vec_World[1],FR_HipX_Vec_World[0],0]]))

    theta_lf = np.array([self.FL_HipX_Pos,self.FL_HipY_Pos,self.FL_Knee_Pos])
    theta_rf = np.array([self.FR_HipX_Pos,self.FR_HipY_Pos,self.FR_Knee_Pos])
    theta_lb = np.array([self.HL_HipX_Pos,self.HL_HipY_Pos,self.HL_Knee_Pos])
    theta_rb = np.array([self.HR_HipX_Pos,self.HR_HipY_Pos,self.HR_Knee_Pos])

    anglurvel_lf = np.array([self.FL_HipX_Vel,self.FL_HipY_Vel,self.FL_Knee_Vel])
    anglurvel_rf = np.array([self.FR_HipX_Vel,self.FR_HipY_Vel,self.FR_Knee_Vel])
    anglurvel_lb = np.array([self.HL_HipX_Vel,self.HL_HipY_Vel,self.HL_Knee_Vel])
    anglurvel_rb = np.array([self.HR_HipX_Vel,self.HR_HipY_Vel,self.HR_Knee_Vel])

    pos_lf = self.leg_FK_Cal(theta_lf,'Lf')
    pos_rf = self.leg_FK_Cal(theta_rf,'Rf')
    pos_lb = self.leg_FK_Cal(theta_lb,'Lb')
    pos_rb = self.leg_FK_Cal(theta_rb,'Rb')

    vel_lf = self.Get_Velocity(theta_lf,anglurvel_lf,'Lf')
    vel_rf = self.Get_Velocity(theta_rf,anglurvel_rf,'Rf')
    vel_lb = self.Get_Velocity(theta_lb,anglurvel_lb,'Lb')
    vel_rb = self.Get_Velocity(theta_rb,anglurvel_rb,'Rb')
    
    LF_Pos_Foot_World = np.dot(self.body_matrix,pos_lf.T) 
    LB_Pos_Foot_World = np.dot(self.body_matrix,pos_lb.T) 
    RB_Pos_Foot_World = np.dot(self.body_matrix,pos_rb.T) 
    RF_Pos_Foot_World = np.dot(self.body_matrix,pos_rf.T) 

    LF_Vel_Foot_World = np.dot(self.body_matrix,vel_lf.T)
    LB_Vel_Foot_World = np.dot(self.body_matrix,vel_lb.T)
    RB_Vel_Foot_World = np.dot(self.body_matrix,vel_rb.T)
    RF_Vel_Foot_World = np.dot(self.body_matrix,vel_rf.T)

    now_height = -(LF_Pos_Foot_World[2]+LB_Pos_Foot_World[2]+RB_Pos_Foot_World[2]+RF_Pos_Foot_World[2])/4
    now_v_height = (LF_Vel_Foot_World[2]+LB_Vel_Foot_World[2]+RB_Vel_Foot_World[2]+RF_Vel_Foot_World[2])/4

    x_pos = (LF_Pos_Foot_World[0]+RF_Pos_Foot_World[0])/2
    x_vel = (LF_Vel_Foot_World[0]+RF_Vel_Foot_World[0])/2
 
    y_pos = (LF_Pos_Foot_World[1]+LB_Pos_Foot_World[1])/2
    y_vel = (LF_Vel_Foot_World[1]+LB_Vel_Foot_World[1])/2

    y_set = (self.Init_Leg_Pos[0][1]+self.Init_Leg_Pos[1][1])/2
    x_set = (self.Init_Leg_Pos[0][0]+self.Init_Leg_Pos[3][0])/2
    

    Fx = 1000*(x_set-x_pos)+80*(0-x_vel)
    Fy = -1000*(y_set-y_pos)-80*(0-y_vel)
    Fz = 300*(0.45-now_height)+20*(0-now_v_height)+630
    tao_x = 500*(0-self.orientation[0])+25*(0-self.body_angular_vel[0])
    tao_y = 500*(0-self.orientation[1])+25*(0-self.body_angular_vel[1])
    tao_z = 500*(0-self.orientation[2])+25*(0-self.body_angular_vel[2])
    
    F_Vec = matrix(np.array([[Fx],[Fy],[Fz],[tao_x],[tao_y],[tao_z]]))
    #print(y_set,y_pos,y_vel,Fy)
    #print(F_Vec)

    p_mat = matrix(np.identity(12))
    q_mat = matrix(np.zeros([12,1]))
    qp_sol = solvers.qp(p_mat,q_mat,None,None,A_mat,F_Vec)
    #print(qp_sol['x'])

    FL_F_Vec = qp_sol['x'][0:3]
    HL_F_Vec = qp_sol['x'][3:6]
    HR_F_Vec = qp_sol['x'][6:9]
    FR_F_Vec = qp_sol['x'][9:12]
    print('FL')
    print(FL_F_Vec)
    print('HL')
    print(HL_F_Vec)
    print('HR')
    print(HR_F_Vec)
    print('FR')
    print(FR_F_Vec)

    LF_Leg_Torq = np.dot(LF_Jacobi,FL_F_Vec)
    LB_Leg_Torq = np.dot(LB_Jacobi,HL_F_Vec)
    RB_Leg_Torq = np.dot(RB_Jacobi,HR_F_Vec)
    RF_Leg_Torq = np.dot(RF_Jacobi,FR_F_Vec)
    #print(LF_Leg_Torq,LB_Leg_Torq,RB_Leg_Torq,RF_Leg_Torq,gait_data)
 
    
    gait_data[0,0:3] = LF_Leg_Torq[0:3,0]
    gait_data[0,3:6] = LB_Leg_Torq[0:3,0]
    gait_data[0,6:9] = RB_Leg_Torq[0:3,0]
    gait_data[0,9:12] = RF_Leg_Torq[0:3,0]
    
    
    #print(x_pos,y_pos,now_height)
    #print(Fx,Fy,Fz)
    
    #print(np.dot(self.body_matrix,self.body_acc_vec))
    #print(np.dot(np.linalg.inv(LF_Jacobi),LF_Torq_Vec)+np.dot(np.linalg.inv(LB_Jacobi),LB_Torq_Vec)+np.dot(np.linalg.inv(RB_Jacobi),RB_Torq_Vec)+np.dot(np.linalg.inv(RF_Jacobi),RF_Torq_Vec))
    #print(np.dot(np.linalg.inv(LF_Jacobi),LF_Torq_Vec))
    #print(np.linalg.inv(LB_Jacobi))
    #print(np.linalg.inv(RB_Jacobi))

    """lf_desEndPos = self.Move_To_Target_Pos_EndPoint_Planning(lf_target_pos,_t,T,'Lf')
    lb_desEndPos = self.Move_To_Target_Pos_EndPoint_Planning(lb_target_pos,_t,T,'Lb')
    rb_desEndPos = self.Move_To_Target_Pos_EndPoint_Planning(rb_target_pos,_t,T,'Rb')
    rf_desEndPos = self.Move_To_Target_Pos_EndPoint_Planning(rf_target_pos,_t,T,'Rf')
    lf_desJointAngle = self.leg_IK_Cal(lf_desEndPos,'Lf')
    lb_desJointAngle = self.leg_IK_Cal(lb_desEndPos,'Lb')
    rb_desJointAngle = self.leg_IK_Cal(rb_desEndPos,'Rb')
    rf_desJointAngle = self.leg_IK_Cal(rf_desEndPos,'Rf')
    gait_data[0,0:3] = self.Joint_Pos_Control(lf_desJointAngle,desVel,'Lf')+self.Stance_Torq[0,0:3]
    gait_data[0,3:6] = self.Joint_Pos_Control(lb_desJointAngle,desVel,'Lb')+self.Stance_Torq[1,0:3]
    gait_data[0,6:9] = self.Joint_Pos_Control(rb_desJointAngle,desVel,'Rb')+self.Stance_Torq[2,0:3]
    gait_data[0,9:12] = self.Joint_Pos_Control(rf_desJointAngle,desVel,'Rf')+self.Stance_Torq[3,0:3]
    #print(self.leg_FK_Cal([self.FL_HipX_Pos,self.FL_HipY_Pos,self.FL_Knee_Pos],'Lf'))
    #print(self.leg_FK_Cal([self.HL_HipX_Pos,self.HL_HipY_Pos,self.HL_Knee_Pos],'Lb'))
    #print(self.leg_FK_Cal([self.HR_HipX_Pos,self.HR_HipY_Pos,self.HR_Knee_Pos],'Rb'))
    #print(self.leg_FK_Cal([self.FR_HipX_Pos,self.FR_HipY_Pos,self.FR_Knee_Pos],'Rf'))
    #print(self.Get_Velocity([self.FL_HipX_Pos,self.FL_HipY_Pos,self.FL_Knee_Pos],[self.FL_HipX_Vel,self.FL_HipY_Vel,self.FL_Knee_Vel],'Lf'))
    """
    return gait_data

  def Leg_Jacobi_Mat_Cal(self,Leg_Name):
    M = np.zeros((3,3))
    Shoulder_Matrix = np.zeros((3,3))
    Jacobian = np.zeros((3,3))
    if(Leg_Name=='Lf' or Leg_Name=='lf'):
      c1 = math.cos(self.FL_HipX_Pos)    
      s1 = math.sin(self.FL_HipX_Pos) 
      c2 = math.cos(self.FL_HipY_Pos)    
      s2 = math.sin(self.FL_HipY_Pos) 
      ck = math.cos(self.FL_HipY_Pos+self.FL_Knee_Pos)    
      sk = math.sin(self.FL_HipY_Pos+self.FL_Knee_Pos) 
      M[0][1] = -self.l3*ck-self.l2*c2
      M[0][2] = -self.l1
      M[2][0] = self.l3*ck
      M[2][2] = -self.l3*sk
      M[1][0] = M[2][0]+self.l2*c2
      M[1][2] = M[2][2]-self.l2*s2
      Shoulder_Matrix[0][0]=1
      Shoulder_Matrix[1][1]=c1
      Shoulder_Matrix[1][2]=-s1
      Shoulder_Matrix[2][1]=s1
      Shoulder_Matrix[2][2]=c1
      tmp_matrix = np.dot(self.body_matrix,Shoulder_Matrix)
      Jacobian = np.dot(M,tmp_matrix.T)
    elif(Leg_Name=='Lb' or Leg_Name=='lb'):
      c1 = math.cos(self.HL_HipX_Pos)    
      s1 = math.sin(self.HL_HipX_Pos) 
      c2 = math.cos(self.HL_HipY_Pos)    
      s2 = math.sin(self.HL_HipY_Pos) 
      ck = math.cos(self.HL_HipY_Pos+self.HL_Knee_Pos)    
      sk = math.sin(self.HL_HipY_Pos+self.HL_Knee_Pos) 
      M[0][1] = -self.l3*ck-self.l2*c2
      M[0][2] = -self.l1
      M[2][0] = self.l3*ck
      M[2][2] = -self.l3*sk
      M[1][0] = M[2][0]+self.l2*c2
      M[1][2] = M[2][2]-self.l2*s2
      Shoulder_Matrix[0][0]=1
      Shoulder_Matrix[1][1]=c1
      Shoulder_Matrix[1][2]=-s1
      Shoulder_Matrix[2][1]=s1
      Shoulder_Matrix[2][2]=c1
      tmp_matrix = np.dot(self.body_matrix,Shoulder_Matrix)
      Jacobian = np.dot(M,tmp_matrix.T)
    elif(Leg_Name=='Rf' or Leg_Name=='rf'):
      c1 = math.cos(self.FR_HipX_Pos)    
      s1 = math.sin(self.FR_HipX_Pos) 
      c2 = math.cos(self.FR_HipY_Pos)    
      s2 = math.sin(self.FR_HipY_Pos) 
      ck = math.cos(self.FR_HipY_Pos+self.FR_Knee_Pos)    
      sk = math.sin(self.FR_HipY_Pos+self.FR_Knee_Pos) 
      M[0][1] = self.l3*ck+self.l2*c2
      M[0][2] = -self.l1
      M[2][0] = self.l3*ck
      M[2][2] = -self.l3*sk
      M[1][0] = M[2][0]+self.l2*c2
      M[1][2] = M[2][2]-self.l2*s2
      Shoulder_Matrix[0][0]=1
      Shoulder_Matrix[1][1]=c1
      Shoulder_Matrix[1][2]=s1
      Shoulder_Matrix[2][1]=-s1
      Shoulder_Matrix[2][2]=c1
      tmp_matrix = np.dot(self.body_matrix,Shoulder_Matrix)
      Jacobian = np.dot(M,tmp_matrix.T)
    elif(Leg_Name=='Rb' or Leg_Name=='rb'):
      c1 = math.cos(self.HR_HipX_Pos)    
      s1 = math.sin(self.HR_HipX_Pos) 
      c2 = math.cos(self.HR_HipY_Pos)    
      s2 = math.sin(self.HR_HipY_Pos) 
      ck = math.cos(self.HR_HipY_Pos+self.HR_Knee_Pos)    
      sk = math.sin(self.HR_HipY_Pos+self.HR_Knee_Pos) 
      M[0][1] = self.l3*ck+self.l2*c2
      M[0][2] = -self.l1
      M[2][0] = self.l3*ck
      M[2][2] = -self.l3*sk
      M[1][0] = M[2][0]+self.l2*c2
      M[1][2] = M[2][2]-self.l2*s2
      Shoulder_Matrix[0][0]=1
      Shoulder_Matrix[1][1]=c1
      Shoulder_Matrix[1][2]=s1
      Shoulder_Matrix[2][1]=-s1
      Shoulder_Matrix[2][2]=c1
      tmp_matrix = np.dot(self.body_matrix,Shoulder_Matrix)
      Jacobian = np.dot(M,tmp_matrix.T)
    else:
      print('Leg_Name Error')

    return Jacobian

  def State_Output(self):
    theta_vec = np.array([self.FL_HipX_Pos,self.FL_HipY_Pos,self.FL_Knee_Pos])
    Leg_Name = 'Rb'
    foot_pos = self.leg_FK_Cal(theta_vec,Leg_Name)
    print(foot_pos)
    theta_pos = self.leg_IK_Cal(foot_pos,Leg_Name)
    print(theta_pos)
    return
  
  def leg_FK_Cal(self, theta_vec,Leg_Name):
    c1 = math.cos(theta_vec[0])    
    s1 = math.sin(theta_vec[0]) 
    c2 = math.cos(theta_vec[1])    
    s2 = math.sin(-theta_vec[1]) 
    ck = math.cos(theta_vec[2]+theta_vec[1])    
    sk = -math.sin(theta_vec[2]+theta_vec[1]) 
    w_bias = self.body_width/2
    l_bias = self.body_length/2
    x,y,z = 0, 0 ,0
    if(Leg_Name=='Lf' or Leg_Name=='lf'):
      y = self.l1*c1+self.l2*c2*s1+self.l3*s1*ck+w_bias
      x = self.l3*sk+self.l2*s2+l_bias
    elif(Leg_Name=='Lb' or Leg_Name=='lb'):
      y = self.l1*c1+self.l2*c2*s1+self.l3*s1*ck+w_bias
      x = self.l3*sk+self.l2*s2-l_bias
    elif(Leg_Name=='Rf' or Leg_Name=='rf'):
      y = -self.l1*c1-self.l2*c2*s1-self.l3*s1*ck-w_bias
      x = self.l3*sk+self.l2*s2+l_bias
    elif(Leg_Name=='Rb' or Leg_Name=='rb'):
      y = -self.l1*c1-self.l2*c2*s1-self.l3*s1*ck-w_bias
      x = self.l3*sk+self.l2*s2-l_bias
    else:
      print('FK_Cal Leg_Name Error!!!')
    z = self.l1*s1-self.l2*c1*c2-self.l3*c1*ck
    foot_pos = np.array([x, y, z])
    return foot_pos
  
  def Get_Velocity(self,theta_vec,vel_vec,Leg_Name):
    c1 = math.cos(theta_vec[0])    
    s1 = math.sin(theta_vec[0]) 
    c2 = math.cos(theta_vec[1])    
    s2 = -math.sin(theta_vec[1]) 
    ck = math.cos(theta_vec[2]+theta_vec[1])    
    sk = -math.sin(theta_vec[2]+theta_vec[1]) 
    w1 = vel_vec[0]
    w2 = -vel_vec[1]
    wk = -vel_vec[2]-vel_vec[1]
    dx,dy,dz = 0, 0 ,0
    if(Leg_Name=='Lf' or Leg_Name=='lf' or Leg_Name=='Lb' or Leg_Name=='lb'):
      dy = -self.l1*s1*w1-self.l2*s2*s1*w2+self.l2*c2*c1*w1+self.l3*c2*ck*w1+self.l3*s1*sk*wk
    elif(Leg_Name=='Rf' or Leg_Name=='rf' or Leg_Name=='Rb' or Leg_Name=='rb'):
      dy = -(-self.l1*s1*w1-self.l2*s2*s1*w2+self.l2*c2*c1*w1+self.l3*c2*ck*w1+self.l3*s1*sk*wk)
    else:
      print('FK_Vel Leg_Name Error!!!')
    dx = self.l3*ck*wk+self.l2*c2*w2
    dz = self.l1*c1*w1+self.l2*c2*s2*w2+self.l2*s1*c2*w1+self.l3*s1*ck*w1+self.l3*c1*sk*wk
    foot_vel = np.array([dx, dy, dz])
    return foot_vel
  
  def leg_IK_Cal(self,pos_vec,Leg_Name):
    w_bias = self.body_width/2
    l_bias = self.body_length/2
    z=pos_vec[2]
    link_1,link_2,link_k = self.l1,self.l2,self.l3
    if(Leg_Name=='Lf' or Leg_Name=='lf'):
      x=pos_vec[1]-w_bias
      y=-pos_vec[0]+l_bias
    elif(Leg_Name=='Lb' or Leg_Name=='lb'):
      x=pos_vec[1]-w_bias
      y=-pos_vec[0]-l_bias
    elif(Leg_Name=='Rf' or Leg_Name=='rf'):
      x=-pos_vec[1]-w_bias
      y=-pos_vec[0]+l_bias
    elif(Leg_Name=='Rb' or Leg_Name=='rb'):
      x=-pos_vec[1]-w_bias
      y=-pos_vec[0]-l_bias
    else:
      theta1,theta2,thetak = 0,0,0
      print('leg_IK_Cal Leg_Name Error')
    theta1=math.acos(link_1/((x*x+z*z)**0.5))+math.pi/2-math.atan2(x,z)
    thetak=math.acos((x*x+y*y+z*z-link_1*link_1-link_2*link_2-link_k*link_k)/(2*link_k*link_2))
    theta2=-math.atan2(y,(x*x+z*z-link_1*link_1))-math.asin((link_k/((x*x+y*y+z*z-link_1*link_1)**0.5))*math.sin(thetak))
    theta_vec = np.array([theta1,-theta2,-thetak])
    return theta_vec

  def Init_Gait_Plan(self):
      des_joint_pos = np.array([0.34, -0.65, 1.30])
      des_joint_vel = np.array([0.0, 0.0, 0.0])
      gait_data = np.zeros((1, 12))
      torq_cal = self.Joint_Pos_Control(des_joint_pos,des_joint_vel,'lf')
      gait_data[0, 0], gait_data[0, 1 ], gait_data[0, 2 ] = torq_cal[0],torq_cal[1],torq_cal[2]
      torq_cal = self.Joint_Pos_Control(des_joint_pos,des_joint_vel,'rf')
      gait_data[0, 3], gait_data[0, 4 ], gait_data[0, 5 ] = torq_cal[0],torq_cal[1],torq_cal[2]
      torq_cal = self.Joint_Pos_Control(des_joint_pos,des_joint_vel,'lb')
      gait_data[0, 6], gait_data[0, 7 ], gait_data[0, 8 ] = torq_cal[0],torq_cal[1],torq_cal[2]
      torq_cal = self.Joint_Pos_Control(des_joint_pos,des_joint_vel,'rb')
      gait_data[0, 9], gait_data[0, 10], gait_data[0, 11] = torq_cal[0],torq_cal[1],torq_cal[2]
      return gait_data

  def Joint_Pos_Control(self,des_pos,des_vel,Leg_Name):   
    torq_output = np.array([0.0, 0.0, 0.0])
    if(Leg_Name=='lf' or Leg_Name=='Lf'):
      torq1 = self.FL_HipX_Kp*(des_pos[0] - self.FL_HipX_Pos) + self.FL_HipX_Kd*(des_vel[0] - self.FL_HipX_Vel)
      torq2 = self.FL_HipY_Kp*(des_pos[1] - self.FL_HipY_Pos) + self.FL_HipY_Kd*(des_vel[1] - self.FL_HipY_Vel)
      torq3 = self.FL_Knee_Kp*(des_pos[2] - self.FL_Knee_Pos) + self.FL_Knee_Kd*(des_vel[2] - self.FL_Knee_Vel)
    elif (Leg_Name=='lb' or Leg_Name=='Lb'):
      torq1 = self.HL_HipX_Kp*(des_pos[0] - self.HL_HipX_Pos) + self.HL_HipX_Kd*(des_vel[0] - self.HL_HipX_Vel)
      torq2 = self.HL_HipY_Kp*(des_pos[1] - self.HL_HipY_Pos) + self.HL_HipY_Kd*(des_vel[1] - self.HL_HipY_Vel)
      torq3 = self.HL_Knee_Kp*(des_pos[2]- self.HL_Knee_Pos) + self.HL_Knee_Kd*(des_vel[2] - self.HL_Knee_Vel)
    elif(Leg_Name=='rf' or Leg_Name=='Rf'):
      torq1 = self.FR_HipX_Kp*(des_pos[0] - self.FR_HipX_Pos) + self.FR_HipX_Kd*(des_vel[0] - self.FR_HipX_Vel)
      torq2 = self.FR_HipY_Kp*(des_pos[1] - self.FR_HipY_Pos) + self.FR_HipY_Kd*(des_vel[1] - self.FR_HipY_Vel)
      torq3 = self.FR_Knee_Kp*(des_pos[2] - self.FR_Knee_Pos) + self.FR_Knee_Kd*(des_vel[2] - self.FR_Knee_Vel)
    elif (Leg_Name=='rb' or Leg_Name=='Rb'):
      torq1 = self.HR_HipX_Kp*(des_pos[0] - self.HR_HipX_Pos) + self.HR_HipX_Kd*(des_vel[0] - self.HR_HipX_Vel)
      torq2 = self.HR_HipY_Kp*(des_pos[1] - self.HR_HipY_Pos) + self.HR_HipY_Kd*(des_vel[1] - self.HR_HipY_Vel)
      torq3 = self.HR_Knee_Kp*(des_pos[2] - self.HR_Knee_Pos) + self.HR_Knee_Kd*(des_vel[2] - self.HR_Knee_Vel)
    else:
      torq1,torq2,torq3 = 0,0,0
      print("Leg_Name Error!!!")
    torq_output[0] = torq1
    torq_output[1] = torq2
    torq_output[2] = torq3
    return torq_output
    
  def teleop_control(self):
    rospy.Subscriber('/quadog/joint_states',JointState, self.GetJointState , queue_size=10)
    rospy.Subscriber('/quadog/action_chatter', String, self.body_action_cb)
    rospy.Subscriber('/imu', Imu, self.orientation_cb, queue_size = 10)
    rospy.Subscriber('/clock',Clock,self.Sim_time,queue_size = 10)
    self.time_start = self.t
    self.time_start_ros = rospy.get_time()
    #time.sleep(1)
    while(1):
      if self.state =='INIT_STATE':
        self.Init_Value()
        #self.t=rospy.get_time()
        """self.State_Output()
	gait_data = self.Init_Gait_Plan()
        self.body_action_pub(gait_data)"""
        if((self.t-self.time_start)>=0.2):
          self.state ='PRE_STATE'
          self.time_start=self.t
          self.time_start_ros = rospy.get_time()
          print('pre_state')
      elif self.state == 'PRE_STATE':
        gait_data = self.pre_state_plan()
        self.body_action_pub(gait_data)
      elif self.state =='STANCE_STATE':
        gait_data = self.stance_state_plan()
        self.body_action_pub(gait_data)
      elif self.state == 'walk':
        gait_data = self.gait_plan(self.v_linear, self.v_angular)
        self.body_action_pub(gait_data)
      elif self.state == 'keep':
        gait_data = self.gait_plan(0, 0)
        self.body_action_pub(gait_data)
      elif self.state == 'overturn':
        self.stand_up()
     # print(self.state)

    rospy.spin()

  def Sim_time(self,time_msg):
    #print(time_msg.clock.to_sec())
    self.t=time_msg.clock.to_sec()


  def GetJointState(self , jointstate_msgs):
    #print(jointstate_msgs)
    self.HL_HipX_Pos = jointstate_msgs.position[6]
    self.HL_HipY_Pos = jointstate_msgs.position[7]
    self.HL_Knee_Pos = jointstate_msgs.position[8]
    self.FL_HipX_Pos = jointstate_msgs.position[0]
    self.FL_HipY_Pos = jointstate_msgs.position[1]
    self.FL_Knee_Pos = jointstate_msgs.position[2]
    self.HR_HipX_Pos = jointstate_msgs.position[9]
    self.HR_HipY_Pos = jointstate_msgs.position[10]
    self.HR_Knee_Pos = jointstate_msgs.position[11]
    self.FR_HipX_Pos = jointstate_msgs.position[3]
    self.FR_HipY_Pos = jointstate_msgs.position[4]
    self.FR_Knee_Pos = jointstate_msgs.position[5]

    self.HL_HipX_Vel = jointstate_msgs.velocity[6]
    self.HL_HipY_Vel = jointstate_msgs.velocity[7]
    self.HL_Knee_Vel = jointstate_msgs.velocity[8]
    self.FL_HipX_Vel = jointstate_msgs.velocity[0]
    self.FL_HipY_Vel = jointstate_msgs.velocity[1]
    self.FL_Knee_Vel = jointstate_msgs.velocity[2]
    self.HR_HipX_Vel = jointstate_msgs.velocity[9]
    self.HR_HipY_Vel = jointstate_msgs.velocity[10]
    self.HR_Knee_Vel = jointstate_msgs.velocity[11]
    self.FR_HipX_Vel = jointstate_msgs.velocity[3]
    self.FR_HipY_Vel = jointstate_msgs.velocity[4]
    self.FR_Knee_Vel = jointstate_msgs.velocity[5]

    self.HL_HipX_Torq = jointstate_msgs.effort[6]
    self.HL_HipY_Torq = jointstate_msgs.effort[7]
    self.HL_Knee_Torq = jointstate_msgs.effort[8]
    self.FL_HipX_Torq = jointstate_msgs.effort[0]
    self.FL_HipY_Torq = jointstate_msgs.effort[1]
    self.FL_Knee_Torq = jointstate_msgs.effort[2]
    self.HR_HipX_Torq = jointstate_msgs.effort[9]
    self.HR_HipY_Torq = jointstate_msgs.effort[10]
    self.HR_Knee_Torq = jointstate_msgs.effort[11]
    self.FR_HipX_Torq = jointstate_msgs.effort[3]
    self.FR_HipY_Torq = jointstate_msgs.effort[4]
    self.FR_Knee_Torq = jointstate_msgs.effort[5]

    return
    
  def body_action_cb(self, action_msgs):

    self.state = 'walk'
    if (action_msgs.data == 'w'):
      self.v_linear = 0.1
      self.v_angular = 0.0
    elif (action_msgs.data == 's'):
      self.v_linear = -0.1
      self.v_angular = 0.0
    elif (action_msgs.data == 'a'):
      self.v_linear = 0.0
      self.v_angular = 0.05
    elif (action_msgs.data == 'd'):
      self.v_linear = 0.0
      self.v_angular = -0.05
    elif (action_msgs.data == 'k'):
      self.state = 'keep'
      self.v_linear = 0.0
      self.v_angular = 0.0
    return
  
  def orientation_cb(self, imu_msgs):
    x = imu_msgs.orientation.x
    y = imu_msgs.orientation.y
    z = imu_msgs.orientation.z
    w = imu_msgs.orientation.w
    w_x = imu_msgs.angular_velocity.x
    w_y = imu_msgs.angular_velocity.y
    w_z = imu_msgs.angular_velocity.z
    a_x = imu_msgs.linear_acceleration.x
    a_y = imu_msgs.linear_acceleration.y
    a_z = imu_msgs.linear_acceleration.z
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    p = math.asin(2 * (w * y - z * x))
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    self.orientation[0] = r
    self.orientation[1] = p
    self.orientation[2] = y
    roll_matrix = np.array([[1.0,0.0,0.0],[0.0,math.cos(r),-math.sin(r)],[0.0,math.sin(r),math.cos(r)]])
    pitch_matrix = np.array([[math.cos(p),0.0,math.sin(p)],[0.0,1.0,0.0],[-math.sin(p),0.0,math.cos(p)]])
    yaw_matrix = np.array([[math.cos(y),-math.sin(y),0.0],[math.sin(y),math.cos(y),0.0],[0.0,0.0,1.0]])
    #print(pitch_matrix * roll_matrix)
    self.body_matrix = np.dot(np.dot(yaw_matrix,pitch_matrix) ,roll_matrix) 
    self.body_acc_vec = np.array([[a_x],[a_y],[a_z]])
    self.body_angular_vel = np.array([w_x,w_y,w_z])
    #print(self.body_matrix)
    #if ((abs(r) > 0.8) and (abs(r) < 2.8)) or ((abs(p) > 0.8) and (abs(p) < 2.8)):
     # self.state = 'overturn'
    #print(self.orientation)   
    return
  
  def body_action_pub(self, gait_np_data):
    data_length = gait_np_data.shape[0]
    pause = rospy.Rate(self.rate)
    gait_data = Float32MultiArray()
    gait_data.data = gait_np_data
    for i in range(data_length):
      self.joint1_tor_pub.publish(gait_data.data[i, 0])
      self.joint2_tor_pub.publish(gait_data.data[i, 1])
      self.joint3_tor_pub.publish(gait_data.data[i, 2])
      self.joint4_tor_pub.publish(gait_data.data[i, 3])
      self.joint5_tor_pub.publish(gait_data.data[i, 4])
      self.joint6_tor_pub.publish(gait_data.data[i, 5])
      self.joint7_tor_pub.publish(gait_data.data[i, 9])
      self.joint8_tor_pub.publish(gait_data.data[i, 10])
      self.joint9_tor_pub.publish(gait_data.data[i, 11])
      self.joint10_tor_pub.publish(gait_data.data[i, 6])
      self.joint11_tor_pub.publish(gait_data.data[i, 7])
      self.joint12_tor_pub.publish(gait_data.data[i, 8])
      pause.sleep()
    return
    
def main():
  try:
      pc = QuaDogController()

      model_type = pc.model_setup()
      while not rospy.is_shutdown():

        if model_type == 't':
          pc.teleop_control()
        elif model_type == 'a':
          pc.auto_navigation()
  except rospy.ROSInterruptException:
      pass



if __name__ == '__main__':
  main()
    
    
