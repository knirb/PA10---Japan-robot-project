#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "pa10/pa10_params.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"

#define pi 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679
#define T 100.0
using namespace std;
class PA10Controller
{

public:
  void StartMoving();
  PA10Controller(); //constructor

private:

  /*double theta_0[6];
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
  double a_5[6];*/

  std::vector< std::vector<double> > a_0;
  std::vector< std::vector<double> > a_1;
  std::vector< std::vector<double> > a_2;
  std::vector< std::vector<double> > a_3;
  std::vector< std::vector<double> > a_4;
  std::vector< std::vector<double> > a_5;

  int nParts;
  int nPoints;

  std::vector<double> fKinStart;
  std::vector< std::vector<double> > theta;
  std::vector< std::vector<double> > thetad;
  std::vector< std::vector<double> > thetadd;
  std::vector< std::vector<double> > trajPoints;
  std::vector<sensor_msgs::JointState> trajAngles;
  std::vector<double> P;


  double l[4] = {317.0, 450.0, 480.0, 70.0};
  double start[6] = {0, 0, 90, 0, 0, 0};
  double target[6] = {0, 0, 90, 0, 0, 0};
  ros::NodeHandle node; //node handler
  ros::Publisher joint_pub; //define publisher
  void initJointState(sensor_msgs::JointState *joint_state); // see init folder
  void targetJointState(sensor_msgs::JointState *joint_state);
  void fillJointState(sensor_msgs::JointState *joint_state, std::vector<double> angles);
  std::vector<double> ForwardKinematics(sensor_msgs::JointState joint); //forward kinematics
  sensor_msgs::JointState InverseKinematics(std::vector<double> targetAngles); //inverse kinematics

  void MoveJoint(int j);
  void PathGenerate();
  void GeneratePoints();
  void GenerateAngles();

  sensor_msgs::JointState cur_joint; //current joint state
  sensor_msgs::JointState target_joint; // target joint state
  bool ReachGoalFlag; //true:Reached goal, false: Not reached goal
  double ticks;
};

PA10Controller::PA10Controller()
{
    //initializing some variables
    ReachGoalFlag = false;

    nParts = 1;
    nPoints = nParts + 1;

    fKinStart = std::vector<double>(16);
    trajPoints = std::vector< std::vector<double> >(nPoints, std::vector<double>(16));
    trajAngles = std::vector<sensor_msgs::JointState>(nPoints);
    theta = std::vector< std::vector<double> >(nPoints, std::vector<double>(6));
    thetad = std::vector< std::vector<double> >(nPoints, std::vector<double>(6));
    thetadd = std::vector< std::vector<double> >(nPoints, std::vector<double>(6));

    a_0 = std::vector< std::vector<double> >(nParts, std::vector<double>(6));
    a_1 = std::vector< std::vector<double> >(nParts, std::vector<double>(6));
    a_2 = std::vector< std::vector<double> >(nParts, std::vector<double>(6));
    a_3 = std::vector< std::vector<double> >(nParts, std::vector<double>(6));
    a_4 = std::vector< std::vector<double> >(nParts, std::vector<double>(6));
    a_5 = std::vector< std::vector<double> >(nParts, std::vector<double>(6));
    P = std::vector<double>(3);

    theta = std::vector< std::vector<double> >(nPoints, std::vector<double>(6));
    thetad = std::vector< std::vector<double> >(nPoints, std::vector<double>(6));
    thetadd = std::vector< std::vector<double> >(nPoints, std::vector<double>(6));

    //Initializing speed and Acceleration targets.
    for (int i = 0; i < 6; ++i) {
      thetad[0][i] = 0;
      thetadd[0][i]= 0;
      thetad[nParts][i]= 0;
      thetadd[nParts][i]= 0;
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
  for(int i=0; i<7; ++i){
    if (i<2) {
      joint_state->position[i] = start[i]*pi/180;;
    }else if (i==2){
      joint_state->position[i] = 0;
    }else{
      joint_state->position[i] = start[i-1]*pi/180;
    }
  }

  return;
}

void PA10Controller::targetJointState(sensor_msgs::JointState *joint_state)
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

  for(int i=0; i<7; ++i){
    if (i<2) {
      joint_state->position[i] = target[i]*pi/180;
    }else if (i==2){
      joint_state->position[i] = 0;
    }else{
      joint_state->position[i] = target[i-1]*pi/180;
    }
  }
  return;
}

void PA10Controller::fillJointState(sensor_msgs::JointState *joint_state, std::vector<double> angles)
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

  for(int i=0; i<7; ++i){
    if (i<2) {
      joint_state->position[i] = angles[i]*pi/180;;
    }else if (i==2){
      joint_state->position[i] = 0;
    }else{
      joint_state->position[i] = angles[i-1]*pi/180;
    }
  }
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

int sgn(double A){
  if (A > 0) return 1;
  if (A < 0) return -1;
  return 0;
}

std::vector<double> PA10Controller::ForwardKinematics(sensor_msgs::JointState joint)
{
  double Ti[4][4];
  double tempRes[4][4];
  double theta[6] = {joint.position[0],joint.position[1],joint.position[3],joint.position[4],joint.position[5],joint.position[6]}; //Checked to be correct
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
    {cos(theta[0]),0,-sin(theta[0]),0},
    {sin(theta[0]),0,cos(theta[0]),0},
    {0,-1,0,l[0]},
    {0,0,0,1}
  };
  double A2[4][4] = {
    {cos(theta[1]),0,sin(theta[1]),0},
    {sin(theta[1]),0,-cos(theta[1]),0},
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
    {cos(theta[2]),0,sin(theta[2]),0},
    {sin(theta[2]),0,-cos(theta[2]),0},
    {0,1,0,0},
    {0,0,0,1}
  };
  double A5[4][4] = {
    {cos(theta[3]),0,-sin(theta[3]),0},
    {sin(theta[3]),0,cos(theta[3]),0},
    {0,-1,0,l[2]},
    {0,0,0,1}
  };
  double A6[4][4] = {
    {cos(theta[4]),0,sin(theta[4]),0},
    {sin(theta[4]),0,-cos(theta[4]),0},
    {0,1,0,0},
    {0,0,0,1}
  };
  double A7[4][4] = {
    {cos(theta[5]),-sin(theta[5]),0,0},
    {sin(theta[5]),cos(theta[5]),0,0},
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
  std::vector<double> targets(16);
  int k = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (abs(Ti[i][j]) > 1e-5) {
        targets[k] = Ti[i][j];
      }else{
        targets[k] = 0;
      }

      k++;
    }
  }

  return targets;
}

sensor_msgs::JointState PA10Controller::InverseKinematics(std::vector<double> tA)
{
  double endP[3];
  double pStar[3];
  double Ti[4][4];
  double tempRes[4][4];
  int k = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Ti[i][j] = tA[k];
      ////cout<<Ti[i][j]<<  " ";
      k++;
    }
    ////cout<<endl;
  }
  ////cout<<endl;

  for (int i = 0; i < 3; i++) {
    endP[i] = Ti[i][3];
    ////cout<<"Ti[i][3]: "<<Ti[i][3]<<endl;
    ////cout<<"Ti[i][2]: "<<Ti[i][2]<<endl;
    pStar[i] = -l[3]*Ti[i][2] + Ti[i][3];
    ////cout<<"pStar[i]: "<<pStar[i]<<endl;
  }
  double sum[2];
  double atanSum[2];
  /*if (abs(pStar[1])<1e-12) {
    sum[0] = 0;
    sum[1] = 0;
    atanSum[0] = atan(0);
    atanSum[1] = atan(0);

  }else{*/
    sum[0] = ((-pStar[0]/pStar[1])+(1/pStar[1])*sqrt(pow(pStar[0],2)+pow(pStar[1],2)));
    sum[1] = ((-pStar[0]/pStar[1])-(1/pStar[1])*sqrt(pow(pStar[0],2)+pow(pStar[1],2)));
    atanSum[0] = atan2((-pStar[0])+sqrt(pow(pStar[0],2)+pow(pStar[1],2)),pStar[1]);
    atanSum[1] = atan2((-pStar[0])-sqrt(pow(pStar[0],2)+pow(pStar[1],2)),pStar[1]);

  //}
  ////cout<<"sum[0], sum[1]: "<<sum[0]<<", "<<sum[1]<<endl;
  double thetaArr[2] = {(180/pi)*2*atanSum[0],(180/pi)*2*atanSum[1]};
  double theta1;

  theta1 = thetaArr[0];
  if (theta1>180) {
    theta1 = theta1-360;
  }else if (theta1<(-180)) {
    theta1 = theta1+360;
  }

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


  thetaArr[0] = (180/pi)*2*atan2((A+sqrt(constSum)),(C+B));
  thetaArr[1] = (180/pi)*2*atan2((A-sqrt(constSum)),(C+B));


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
  /*if (abs(Tip[1][2]) < 1e-12) {
    theta4 = 0;
  } else{*/
    theta4 = 180/pi*atan2(Tip[1][2],Tip[0][2]);
  //}

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
  /*if (abs(Tip[0][2]*cos(pi/180*theta4) + Tip[1][2]*sin(pi/180*theta4))<1e-12) {
    theta5 = 0;
  }else{*/
    theta5   = 180/pi*atan2((Tip[0][2]*cos(pi/180*theta4) + Tip[1][2]*sin(pi/180*theta4)),Tip[2][2]);
    //SUPER ODD BUG FIX!
    /*if (abs(Tip[2][2])<1e-12 && Tip[0][2]<0 && Tip[2][2] <0) {
      theta5 = -theta5;

  }}*/

  /*if (Tipcheck[1][3]>1e-10) {
    if (theta5 < 0) {
      theta5 = theta5 + 180;
    }else if (theta5 > 0) {
      theta5 = theta5 - 180;
    }
  }*/

  double A6i[4][4]={
    {cos(theta5*pi/180), sin(theta5*pi/180), 0, 0},
    {0,0,1,0},
    {sin(theta5*pi/180), -cos(theta5*pi/180), 0, 0},
    {0,0,0,1}
  };

  double Tipcheck2[4][4];
  MatMult(A6i, tempRes, Tipcheck2);

  double theta6;
  /*if (abs((-Tip[0][0]*sin(pi/180*theta4) + Tip[1][0]*cos(pi/180*theta4)))<1e-12) {
    theta6 = 0;
  }else{*/
    theta6 = 180/pi*atan2((-Tip[0][0]*sin(pi/180*theta4) + Tip[1][0]*cos(pi/180*theta4)),(-Tip[0][1]*sin(pi/180*theta4) + Tip[1][1]*cos(pi/180*theta4)));
  //}

  /*if (Tipcheck2[0][0] <= 0) {
    if (theta6 <= 0) {
      theta6 = theta6 + 180;
    }else if (theta6 > 0) {
      theta6 = theta6 - 180;
    }
  }*/
  std::vector<double> angles(6);
  ////cout<<" "<< theta1<<" "<< theta2<<" "<< theta3<<" "<< theta4<<" "<< theta5<<" "<< theta6<<endl;
  angles[0] = theta1;
  angles[1] = theta2;
  angles[2] = theta3;
  angles[3] = theta4;
  angles[4] = theta5;
  angles[5] = theta6;



  /*//cout<<"Theta1: "<<theta1<<endl;
  //cout<<"Theta2: "<<theta2<<endl;
  //cout<<"Theta3: "<<theta3<<endl;
  //cout<<"Theta4: "<<theta4<<endl;
  //cout<<"Theta5: "<<theta5<<endl;
  //cout<<"Theta6: "<<theta6<<endl<<endl;*/
  sensor_msgs::JointState createdState;
  fillJointState(&createdState, angles);
  return createdState;



}

void PA10Controller::GeneratePoints()
{

    // T matrices for Start, end + delta vector to be divided into nParts points.

    std::vector<double> fKinEnd(16);
    std::vector<double> deltaSE(16);

    for (int i = 0; i < 16; i++) {
      deltaSE[i] = 0;
    }

    fKinStart = ForwardKinematics(cur_joint);
    fKinEnd = ForwardKinematics(target_joint);

    deltaSE[3] = (fKinEnd[3]-fKinStart[3]);
    deltaSE[7] = (fKinEnd[7]-fKinStart[7]);
    deltaSE[11] = (fKinEnd[11]-fKinStart[11]);
    ////cout<<deltaSE[3]<<", "<<deltaSE[7]<<", "<<deltaSE[11]<<endl;
    for (int i = 0; i < nPoints; ++i) {
      for (int j = 0; j < fKinStart.size(); j++) {
        trajPoints[i][j] = fKinStart[j] + deltaSE[j]/nParts*i;
        if (j == 3 || j == 7 || j == 11) {
          ////cout<<fKinStart[j]+ deltaSE[j]/nParts*i<<", "<<endl;
        }
      }
    //  //cout<<fKinStart[3]+ deltaSE[3]/nParts*i<<", "<<fKinStart[7]+ deltaSE[7]/nParts*i<<", "<<fKinStart[11]+ deltaSE[11]/nParts*i<<endl;
    }


}

void PA10Controller::GenerateAngles()
{
  for (int i = 0; i < nPoints; ++i) {
    ////cout<<trajPoints[i]
    trajAngles[i] = InverseKinematics(trajPoints[i]);
    for (int j = 0; j < 7; j++) {
      ////cout<<"theta["<<j+1<<"]: "<<trajAngles[i].position[j]*180/pi<<" ";
    }
    ////cout<<endl;
  }
}

void PA10Controller::PathGenerate()
{
  GeneratePoints();
  //cout<<"Generated Points"<<endl;

  double Ti = T/nParts;

  for (int i = 0; i < nPoints; ++i) {
    for (int j = 0; j < 3; j++) {
        theta[i][j] = trajPoints[i][4*(j+1) - 1];
        //cout<<theta[i][j]<<endl;
    }
  }
  theta[0][1] = 0;
  theta[1][1] = 1;
  //cout<<"theta done"<<endl;
  for (int i = 1; i < nParts; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (sgn((theta[i][j]-theta[i-1][j])) == sgn((theta[i+1][j]-theta[i][j])/Ti) ) {
        thetad[i][j]= 0;
      }else{
        thetad[i][j]= 0;
      }
      thetadd[i][j] = 0;
    }
  }
  //cout<<"thetad and dd done"<<endl;
  for (int i = 0; i < nParts; ++i) {
    for (int j = 0; j < 3; j++) {
      //cout<<"started loop"<<endl;
      a_0[i][j] = theta[i][j];
      a_1[i][j] = thetad[i][j];
      a_2[i][j] = thetadd[i][j]/2;
      a_3[i][j] = (1/(2*pow(Ti,3)))*(20*(theta[i+1][j]-theta[i][j])-(8*thetad[i+1][j] + 12*thetad[i][j])*Ti - (3*thetadd[i+1][j]-thetadd[i][j])*Ti*Ti);
      a_4[i][j] = (1/(2*pow(Ti,4)))*(30*(theta[i][j]-theta[i+1][j])+(14*thetad[i+1][j] + 16*thetad[i][j])*Ti + (3*thetadd[i+1][j]-2*thetadd[i][j])*Ti*Ti);
      a_5[i][j] = (1/(2*pow(Ti,5)))*(12*(theta[i+1][j]-theta[i][j])-6*(thetad[i+1][j] + thetad[i][j])*Ti - (thetadd[i+1][j]-thetadd[i][j])*Ti*Ti);
      //cout<<"ended loop"<<endl;
    }
    //cout<<"a's calculated"<<endl;
  }
}

void PA10Controller::MoveJoint(int j)
{
  double ftx = a_0[0][0] + a_1[0][0]*pow(ticks,1) + a_2[0][0]*pow(ticks,2) + a_3[0][0]*pow(ticks,3) + a_4[0][0]*pow(ticks,4) + a_5[0][0]*pow(ticks,5);
  double fty = a_0[0][1] + a_1[0][1]*pow(ticks,1) + a_2[0][1]*pow(ticks,2) + a_3[0][1]*pow(ticks,3) + a_4[0][1]*pow(ticks,4) + a_5[0][1]*pow(ticks,5);
  double ftz = a_0[0][2] + a_1[0][2]*pow(ticks,1) + a_2[0][2]*pow(ticks,2) + a_3[0][2]*pow(ticks,3) + a_4[0][2]*pow(ticks,4) + a_5[0][2]*pow(ticks,5);
  //cout<<"Ft's calculated "<<endl;
  std::vector<double> curPos;
  curPos = std::vector<double>(16);
  for (int i = 0; i < 16; i++) {
    curPos[i] = trajPoints[0][i];
  }
  double rad = 300;
  curPos[3] = ftx;
  curPos[7] = curPos[7] + rad*cos(fty*2*pi);
  curPos[11] = curPos[11] - rad*sin(fty*2*pi);;
  //cout<<"position found: "<< curPos[3]<<", "<<curPos[7]<< ", " << curPos[11]<<endl;
  sensor_msgs::JointState curAngles;
  curAngles = InverseKinematics(curPos);
  //cout<<"angles found: ";

  if (j == nParts) {
    j--;
  }

  for(int i=0; i<7; ++i)
  {
    if (i<2) {
       cur_joint.position[i] = curAngles.position[i];
       //cout<<curAngles.position[i]<<", ";


    }else if (i > 2) {
      cur_joint.position[i] = curAngles.position[i];
      //cout<<curAngles.position[i]<<", ";
    }
  }
  //cout<<"angles set"<<endl;
    ////cout<<endl;
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

  targetJointState(&target_joint);
  PathGenerate();
  int count = 0;
  int part = 0;
  while(ros::ok() && ReachGoalFlag == false)
    {
      //cout<<"tick: "<<ticks<<endl;
      ////cout<<"ticks: "<<ticks<<" part: "<<part<<endl;
      MoveJoint(part);
      //cout<<"Moved Joint"<<endl;
      ros::spinOnce();
      loop_rate.sleep();//sleep 1 loop rate(0.1sec)

      ticks++;
      if(ticks>T){ ReachGoalFlag=true;}

      count++;
      if (count >=T/nParts) {
        count = 0;
        part++;
      }
    }

  ROS_INFO("Finished");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_pub");

  PA10Controller pa10controller;

  pa10controller.StartMoving();

  return 0;
}
