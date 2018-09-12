/*
  Manipulator Training 2018 Spring
  Sample Program
  written by Rikuto SATO(2018/4)
*/


#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>

#include "pa10/pa10_params.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"

#define pi 3.1415926
#define T 31.0
class PA10Controller
{
public:
  void StartMoving();
  PA10Controller(); //constructor


private:
  double theta_0[6];
  double thetad_0[6];
  double thetadd_0[6];
  double theta_f[6];
  double thetad_f[6];
  double thetadd_f[6];
  double a_0[6];
  double a_1[6];
  double a_2[6];
  double a_3[6];
  double a_4[6];
  double a_5[6];
  double l[4] = {317.0, 450.0, 480.0, 70.0};
  ros::NodeHandle node; //node handler
  ros::Publisher joint_pub; //define publisher
  void initJointState(sensor_msgs::JointState *joint_state); // see init folder
  void targetJointState(sensor_msgs::JointState *joint_state,double targets[6]);
  /*Example definition of kinematics compute functions*/
  std::vector<double> ForwardKinematics(sensor_msgs::JointState joint); //forward kinematics
  sensor_msgs::JointState InverseKinematics(std::vector<double> target); //inverse kinematics

  void MoveJoint();
  void PathGenerate();

  sensor_msgs::JointState cur_joint; //current joint state
  sensor_msgs::JointState target_joint; // target joint state
  bool ReachGoalFlag; //true:Reached goal, false: Not reached goal
  double ticks;
};


//Constructor
  PA10Controller::PA10Controller()
  {
    //initialize
    ReachGoalFlag = false;

    //Initializing speed and Acceleration targets.
    for (int i = 0; i < 6; ++i) {
      thetad_0[i] = 0;
      thetadd_0[i]= 0;
      thetad_f[i]= 0;
      thetadd_f[i]= 0;
    }
    //Target angles for first few tries.
    //define ROS node
    joint_pub = node.advertise<sensor_msgs::JointState>("/pa10/joint_states",10);
  }


void PA10Controller::initJointState(sensor_msgs::JointState *joint_state)
{
  joint_state->name.resize(9);
  joint_state->position.resize(9);
  joint_state->name[0] = "joint_1";
  joint_state->name[1] = "joint_2";
  joint_state->name[2] = "joint_3";
  joint_state->name[3] = "joint_4";
  joint_state->name[4] = "joint_5";
  joint_state->name[5] = "joint_6";
  joint_state->name[6] = "joint_7";
  joint_state->name[7] = "gripper_finger";
  joint_state->name[8] = "gripper_finger_mimic_joint";

  //Initialize Position (All angles are 0.0[rad])
  for(int i=0; i<7; ++i)joint_state->position[i] = 0.0;

  return;
}
void PA10Controller::targetJointState(sensor_msgs::JointState *joint_state, double targets[6])
{
  joint_state->name.resize(9);
  joint_state->position.resize(9);
  joint_state->name[0] = "joint_1";
  joint_state->name[1] = "joint_2";
  joint_state->name[2] = "joint_3";
  joint_state->name[3] = "joint_4";
  joint_state->name[4] = "joint_5";
  joint_state->name[5] = "joint_6";
  joint_state->name[6] = "joint_7";
  joint_state->name[7] = "gripper_finger";
  joint_state->name[8] = "gripper_finger_mimic_joint";

  //Initialize Position (All angles are 0.0[rad])
  for(int i=0; i<7; ++i)joint_state->position[i] = targets[i]*pi/180;

  return;
}

void MatMult(double A[4][4], double B[4][4], double Ti[4][4])
{
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Ti[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j] +A[i][3]*B[3][j];
    }
  }
}
std::vector<double> PA10Controller::ForwardKinematics(sensor_msgs::JointState joint)
{
  double Ti[4][4];
  double tempRes[4][4];

  double alpha[6] = {-90, 90, -90, 90, -90, 0};
  double theta[6] = {joint.position[0],joint.position[1],joint.position[2],joint.position[3],joint.position[4],joint.position[5]};

  for (int i = 0; i < 6; i++) {
    while (abs(theta[i])>180) {
      if (theta[i]>180) {
        theta[i] = theta[i]-360;
      }else{
        theta[i] = theta[i] + 360;
      }
    }
  }

  double A1[4][4] = {
    {cos(theta[0]*pi/180),0,-sin(theta[0]*pi/180),0},
    {sin(theta[0]*pi/180),0,cos(theta[0]*pi/180),0},
    {0,-1,0,l[0]},
    {0,0,0,1}
  };
  double A2[4][4] = {
    {cos(theta[1]*pi/180),0,sin(theta[1]*pi/180),0},
    {sin(theta[1]*pi/180),0,-cos(theta[1]*pi/180),0},
    {0,1,0,0},
    {0,0,0,1}
  };
  double A3[4][4] = {
    {1,0,0,0},
    {0,0,1,0},
    {0,-1,0,l[1]},
    {0,0,0,1}
  };
  double A4[4][4] = {
    {cos(theta[2]*pi/180),0,sin(theta[2]*pi/180),0},
    {sin(theta[2]*pi/180),0,-cos(theta[2]*pi/180),0},
    {0,1,0,0},
    {0,0,0,1}
  };
  double A5[4][4] = {
    {cos(theta[3]*pi/180),0,-sin(theta[3]*pi/180),0},
    {sin(theta[3]*pi/180),0,cos(theta[3]*pi/180),0},
    {0,-1,0,l[2]},
    {0,0,0,1}
  };
  double A6[4][4] = {
    {cos(theta[4]*pi/180),0,sin(theta[4]*pi/180),0},
    {sin(theta[4]*pi/180),0,-cos(theta[4]*pi/180),0},
    {0,1,0,0},
    {0,0,0,1}
  };
  double A7[4][4] = {
    {cos(theta[5]*pi/180),-sin(theta[5]*pi/180),0,0},
    {sin(theta[5]*pi/180),cos(theta[5]*pi/180),0,0},
    {0,0,1,l[3]},
    {0,0,0,1}
  };
  MatMult(A1, A2, Ti);
  memcpy(tempRes,Ti,sizeof(Ti));
  MatMult(tempRes, A3, Ti);
  memcpy(tempRes,Ti,sizeof(Ti));
  MatMult(tempRes, A4, Ti);
  memcpy(tempRes,Ti,sizeof(Ti));
  MatMult(tempRes, A5, Ti);
  memcpy(tempRes,Ti,sizeof(Ti));
  MatMult(tempRes, A6, Ti);
  memcpy(tempRes,Ti,sizeof(Ti));
  MatMult(tempRes, A7, Ti);
  std::vector<double> target(16);
  int k = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      target[k] = Ti[i][j];
      k++;
    }
  }
  return target;
}


sensor_msgs::JointState PA10Controller::InverseKinematics(std::vector<double> target)
{
  double endP[3];
  double pStar[3];
  double Ti[4][4];
  double tempRes[4][4];
  int k = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Ti[i][j] = target[k];
      k++;
    }
  }
  for (int i = 0; i < 3; i++) {
    endP[i] = Ti[i][3];
    pStar[i] = -l[3]*Ti[i][2] + Ti[i][3];
  }
  double sum[2];
  if (abs(pStar[1])<1e-12) {
    sum[0] = 0;
    sum[1] = 0;
  }else{
    sum[0] = ((-pStar[0]/pStar[1])+(1/pStar[1])*sqrt(pow(pStar[0],2)+pow(pStar[1],2)));
    sum[1] = ((-pStar[0]/pStar[1])-(1/pStar[1])*sqrt(pow(pStar[0],2)+pow(pStar[1],2)));
  }

  double thetaArr[2] = {(180/pi)*2*atan(sum[0]),(180/pi)*2*atan(sum[1])};
  double theta1;

  theta1 = thetaArr[0];

  double A = cos(theta1*pi/180)*pStar[0]+sin(theta1*pi/180)*pStar[1];
  double B = pStar[2]-l[0];
  double C = (A*A + B*B + l[1]*l[1] - (l[2]*l[2]))/(2*l[1]);

  double constSum = A*A + B*B - C*C;
  if (constSum < 0 & abs(constSum) < 1e-6 ) {
    constSum = 0;
  }
  if (constSum >= 0) {
    if (B+C != 0) {
      sum[0] = (A+sqrt(constSum))/(C+B);
      sum[1] = (A-sqrt(constSum))/(C+B);
    }else {
      sum[0] = (C-B)/(A+sqrt(constSum));
      sum[1] = (C-B)/(A-sqrt(constSum));
    }
  }


  thetaArr[0] = (180/pi)*2*atan(sum[0]);
  thetaArr[1] = (180/pi)*2*atan(sum[1]);


  double theta2;


  if (abs(thetaArr[0]) < abs(thetaArr[1])) {
    theta2 = thetaArr[0];
  }else if (abs(thetaArr[0]) > abs(thetaArr[1])) {
    theta2 = thetaArr[1];
  }else if (thetaArr > 0) {
    theta2 = thetaArr[0];
  }else{
    theta2 = thetaArr[1];
  }

  double theta3;
  double P3x = A - l[1]*sin(theta2*pi/180);
  double P3y = B - l[1]*cos(theta2*pi/180);


  if (theta2> 0 & A > 0) {
    if (abs(A)>abs(l[1]*sin(theta2*pi/180))) {
      theta3 = (180/pi)*acos((B-l[1]*cos(theta2*pi/180))/l[2])-theta2;
    }else{
      theta3 = 360- theta2 - (180/pi)*acos((B-l[1]*cos(theta2*pi/180))/l[2]);
    }
  }else if (theta2<0 & A>0) {
    theta3 = (180/pi)*acos((B-l[1]*cos(theta2*pi/180))/l[2])-theta2;
  }else if (theta2>0 & A<0) {
    theta3 = -(180/pi)*acos((B-l[1]*cos(theta2*pi/180))/l[2])-theta2;
  }else{
    if (abs(A)>abs(l[1]*sin(theta2*pi/180))) {
      theta3 = -(180/pi)*acos((B-l[1]*cos(theta2*pi/180))/l[2])-theta2;
    }else{
      theta3 = -360+((180/pi)*acos((B-l[1]*cos(theta2*pi/180))/l[2])-theta2);
    }
  }


  double A1i[4][4]={
    {cos(theta1*pi/180), sin(theta1*pi/180), 0, 0},
    {0,0,-1,l[0]},
    {-sin(theta1*pi/180), cos(theta1*pi/180), 0, 0},
    {0,0,0,1}
  };
  double A2i[4][4]={
    {cos(theta2*pi/180), sin(theta2*pi/180), 0, 0},
    {0,0,1,0},
    {sin(theta2*pi/180), -cos(theta2*pi/180), 0, 0},
    {0,0,0,1}
  };
  double A3i[4][4]={
    {1, 0, 0, 0},
    {0,0,-1,l[1]},
    {0, 1, 0, 0},
    {0, 0, 0, 1}
  };
  double A4i[4][4]={
    {cos(theta3*pi/180), sin(theta3*pi/180), 0, 0},
    {0,0,1,0},
    {sin(theta3*pi/180), -cos(theta3*pi/180), 0, 0},
    {0,0,0,1}
  };

  double Tip[4][4];
  MatMult(A1i, Ti, Tip);
  memcpy(tempRes,Tip,sizeof(Ti));
  MatMult(A2i, tempRes, Tip);
  memcpy(tempRes,Tip,sizeof(Ti));
  MatMult(A3i, tempRes, Tip);
  memcpy(tempRes,Tip,sizeof(Ti));
  MatMult(A4i, tempRes, Tip);
  memcpy(tempRes,Tip,sizeof(Ti));

  double theta4;
  if (abs(Tip[1][2]) < 1e-12) {
    theta4 = 0;
  } else{
    theta4 = 180/pi*atan(Tip[1][2]/Tip[0][2]);
  }

  double A5i[4][4]={
    {cos(theta4*pi/180), sin(theta4*pi/180), 0, 0},
    {0,0,-1,l[2]},
    {-sin(theta4*pi/180), cos(theta4*pi/180), 0, 0},
    {0,0,0,1}
  };

  double Tipcheck[4][4];
  MatMult(A5i, tempRes, Tipcheck);
  memcpy(tempRes,Tipcheck,sizeof(Ti));
  //PrintMat(Tipcheck);

  double theta5;
  if (abs(Tip[0][2]*cos(pi/180*theta4) + Tip[1][2]*sin(pi/180*theta4))<1e-12) {
    theta5 = 0;
  }else{
    theta5   = 180/pi*atan((Tip[0][2]*cos(pi/180*theta4) + Tip[1][2]*sin(pi/180*theta4))/Tip[2][2]);
  }

  if (Tipcheck[1][3]>1e-10) {
    if (theta5 < 0) {
      theta5 = theta5 + 180;
    }else if (theta5 > 0) {
      theta5 = theta5 - 180;
    }
  }

  double A6i[4][4]={
    {cos(theta5*pi/180), sin(theta5*pi/180), 0, 0},
    {0,0,1,0},
    {sin(theta5*pi/180), -cos(theta5*pi/180), 0, 0},
    {0,0,0,1}
  };

  double Tipcheck2[4][4];
  MatMult(A6i, tempRes, Tipcheck2);

  double theta6;
  if (abs((-Tip[0][0]*sin(pi/180*theta4) + Tip[1][0]*cos(pi/180*theta4)))<1e-12) {
    theta6 = 0;
  }else{
    theta6 = 180/pi*atan((-Tip[0][0]*sin(pi/180*theta4) + Tip[1][0]*cos(pi/180*theta4))/(-Tip[0][1]*sin(pi/180*theta4) + Tip[1][1]*cos(pi/180*theta4)));
  }

  if (Tipcheck2[0][0] <= 0) {
    if (theta6 <= 0) {
      theta6 = theta6 + 180;
    }else if (theta6 > 0) {
      theta6 = theta6 - 180;
    }
  }

}


void PA10Controller::PathGenerate()
{
  for (int i = 0; i < 6; ++i) {
    theta_0[i] = cur_joint.position[i];
    theta_f[i] = target_joint.position[i];
    a_0[i] = theta_0[i];
    a_1[i] = thetad_0[i];
    a_2[i] = thetadd_0[i];
    a_3[i] = (1/(2*pow(T,3)))*(20*(theta_f[i]-theta_0[i])-(8*thetad_f[i] + 12*thetad_0[i])*T - (3*thetadd_f[i]-thetadd_0[i])*T*T);
    a_4[i] = (1/(2*pow(T,4)))*(30*(theta_0[i]-theta_f[i])+(14*thetad_f[i] + 16*thetad_0[i])*T + (3*thetadd_f[i]-2*thetadd_0[i])*T*T);
    a_5[i] = (1/(2*pow(T,5)))*(12*(theta_f[i]-theta_0[i])-6*(thetad_f[i] + thetad_0[i])*T - (thetadd_f[i]-thetadd_0[i])*T*T);
  }
}

void PA10Controller::MoveJoint()
{

  for(int i=0; i<7; ++i)
    {
      if (i<2) {
        cur_joint.position[i]=a_0[i] + a_1[i]*pow(ticks,1) + a_2[i]*pow(ticks,2) + a_3[i]*pow(ticks,3) + a_4[i]*pow(ticks,4) + a_5[i]*pow(ticks,5);
      }else if (i > 2) {
        cur_joint.position[i]=a_0[i-1] + a_1[i-1]*pow(ticks,1) + a_2[i-1]*pow(ticks,2) + a_3[i-1]*pow(ticks,3) + a_4[i-1]*pow(ticks,4) + a_5[i-1]*pow(ticks,5);
      }
    }
  //publish joint states
  joint_pub.publish(cur_joint);
}


void PA10Controller::StartMoving()
{
  ros::Rate loop_rate(10); //set loop rate 10[Hz]
  ROS_INFO("Start Moving");
  ticks=0.0;

  //initiallize joint names
  initJointState(&cur_joint);
  double targets[6] = {90, 0, 90, -90, 30, 0};
  targetJointState(&target_joint, targets);
  PathGenerate();

  while(ros::ok() && ReachGoalFlag == false)
    {
      MoveJoint();
      ros::spinOnce();
      loop_rate.sleep();//sleep 1 loop rate(0.1sec)
      if(ticks>T) ReachGoalFlag=true;
      ticks++;
    }

  ROS_INFO("Finished");
}


//main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_pub");

  PA10Controller pa10controller;

  pa10controller.StartMoving();

  return 0;
}
