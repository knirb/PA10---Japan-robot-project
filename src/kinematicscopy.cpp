#include <iostream>
#include <stdio.h>      /* printf */
#include <math.h>       /* sin,tan,cos,atan */
#include <string.h>     // memcpy

#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679
using namespace std;


void MatMult(double A[4][4], double B[4][4], double Ti[4][4])
{
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Ti[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j] +A[i][3]*B[3][j];
    }
  }
}

void PrintMat(double mat[4][4]){
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      if (abs(mat[i][j]) < 1e-05) {
        cout<<"0"<<" ";
      }else{
      cout<<mat[i][j]<<" ";
      }
    }
    cout<<endl;
  }
  cout<<endl;
}


int main ()
{
  double Ti[4][4];
  double tempRes[4][4];
  double l[4] = {317.0, 450.0, 480.0, 70.0};
  double alpha[6] = {-90, 90, -90, 90, -90, 0};
  double theta[6] = {0,-170,100,0,0,0};

  for (int i = 0; i < 6; i++) {
    while (abs(theta[i])>180) {
      if (theta[i]>180) {
        theta[i] = theta[i]-360;
        cout<< theta[i]<<endl;
      }else{
        theta[i] = theta[i] + 360;
      }
    }
  }

  double A1[4][4] = {
    {cos(theta[0]*PI/180),0,-sin(theta[0]*PI/180),0},
    {sin(theta[0]*PI/180),0,cos(theta[0]*PI/180),0},
    {0,-1,0,l[0]},
    {0,0,0,1}
  };
  double A2[4][4] = {
    {cos(theta[1]*PI/180),0,sin(theta[1]*PI/180),0},
    {sin(theta[1]*PI/180),0,-cos(theta[1]*PI/180),0},
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
    {cos(theta[2]*PI/180),0,sin(theta[2]*PI/180),0},
    {sin(theta[2]*PI/180),0,-cos(theta[2]*PI/180),0},
    {0,1,0,0},
    {0,0,0,1}
  };
  double A5[4][4] = {
    {cos(theta[3]*PI/180),0,-sin(theta[3]*PI/180),0},
    {sin(theta[3]*PI/180),0,cos(theta[3]*PI/180),0},
    {0,-1,0,l[2]},
    {0,0,0,1}
  };
  double A6[4][4] = {
    {cos(theta[4]*PI/180),0,sin(theta[4]*PI/180),0},
    {sin(theta[4]*PI/180),0,-cos(theta[4]*PI/180),0},
    {0,1,0,0},
    {0,0,0,1}
  };
  double A7[4][4] = {
    {cos(theta[5]*PI/180),-sin(theta[5]*PI/180),0,0},
    {sin(theta[5]*PI/180),cos(theta[5]*PI/180),0,0},
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

  //PrintMat(Ti);

  double endP[3];
  double pStar[3];
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

  double thetaArr[2] = {(180/PI)*2*atan(sum[0]),(180/PI)*2*atan(sum[1])};
  double theta1;

  theta1 = thetaArr[0];
  cout <<"theta1:  " <<theta1<<endl<<endl;

  double A = cos(theta1*PI/180)*pStar[0]+sin(theta1*PI/180)*pStar[1];
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
  }else{
    cout << "abs(C) was too big"<<endl;
    cout << "pow(C,2) = "<< pow(C,2) <<endl;
  }


  thetaArr[0] = (180/PI)*2*atan(sum[0]);
  thetaArr[1] = (180/PI)*2*atan(sum[1]);


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
  double P3x = A - l[1]*sin(theta2*PI/180);
  double P3y = B - l[1]*cos(theta2*PI/180);


  if (theta2> 0 & A > 0) {
    if (abs(A)>abs(l[1]*sin(theta2*PI/180))) {
      theta3 = (180/PI)*acos((B-l[1]*cos(theta2*PI/180))/l[2])-theta2;
    }else{
      theta3 = 360- theta2 - (180/PI)*acos((B-l[1]*cos(theta2*PI/180))/l[2]);
    }
  }else if (theta2<0 & A>0) {
    theta3 = (180/PI)*acos((B-l[1]*cos(theta2*PI/180))/l[2])-theta2;
  }else if (theta2>0 & A<0) {
    theta3 = -(180/PI)*acos((B-l[1]*cos(theta2*PI/180))/l[2])-theta2;
  }else{
    if (abs(A)>abs(l[1]*sin(theta2*PI/180))) {
      theta3 = -(180/PI)*acos((B-l[1]*cos(theta2*PI/180))/l[2])-theta2;
    }else{
      theta3 = -360+((180/PI)*acos((B-l[1]*cos(theta2*PI/180))/l[2])-theta2);
    }
  }


  double A1i[4][4]={
    {cos(theta1*PI/180), sin(theta1*PI/180), 0, 0},
    {0,0,-1,l[0]},
    {-sin(theta1*PI/180), cos(theta1*PI/180), 0, 0},
    {0,0,0,1}
  };
  double A2i[4][4]={
    {cos(theta2*PI/180), sin(theta2*PI/180), 0, 0},
    {0,0,1,0},
    {sin(theta2*PI/180), -cos(theta2*PI/180), 0, 0},
    {0,0,0,1}
  };
  double A3i[4][4]={
    {1, 0, 0, 0},
    {0,0,-1,l[1]},
    {0, 1, 0, 0},
    {0, 0, 0, 1}
  };
  double A4i[4][4]={
    {cos(theta3*PI/180), sin(theta3*PI/180), 0, 0},
    {0,0,1,0},
    {sin(theta3*PI/180), -cos(theta3*PI/180), 0, 0},
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
    theta4 = 180/PI*atan(Tip[1][2]/Tip[0][2]);
  }

  double A5i[4][4]={
    {cos(theta4*PI/180), sin(theta4*PI/180), 0, 0},
    {0,0,-1,l[2]},
    {-sin(theta4*PI/180), cos(theta4*PI/180), 0, 0},
    {0,0,0,1}
  };

  double Tipcheck[4][4];
  MatMult(A5i, tempRes, Tipcheck);
  memcpy(tempRes,Tipcheck,sizeof(Ti));
  //PrintMat(Tipcheck);

  double theta5;
  if (abs(Tip[0][2]*cos(PI/180*theta4) + Tip[1][2]*sin(PI/180*theta4))<1e-12) {
    theta5 = 0;
  }else{
    theta5   = 180/PI*atan((Tip[0][2]*cos(PI/180*theta4) + Tip[1][2]*sin(PI/180*theta4))/Tip[2][2]);
  }

  if (Tipcheck[1][3]>1e-10) {
    if (theta5 < 0) {
      theta5 = theta5 + 180;
    }else if (theta5 > 0) {
      theta5 = theta5 - 180;
    }
  }

  double A6i[4][4]={
    {cos(theta5*PI/180), sin(theta5*PI/180), 0, 0},
    {0,0,1,0},
    {sin(theta5*PI/180), -cos(theta5*PI/180), 0, 0},
    {0,0,0,1}
  };

  double Tipcheck2[4][4];
  MatMult(A6i, tempRes, Tipcheck2);

  double theta6;
  if (abs((-Tip[0][0]*sin(PI/180*theta4) + Tip[1][0]*cos(PI/180*theta4)))<1e-12) {
    theta6 = 0;
  }else{
    theta6 = 180/PI*atan((-Tip[0][0]*sin(PI/180*theta4) + Tip[1][0]*cos(PI/180*theta4))/(-Tip[0][1]*sin(PI/180*theta4) + Tip[1][1]*cos(PI/180*theta4)));
  }

  if (Tipcheck2[0][0] <= 0) {
    if (theta6 <= 0) {
      theta6 = theta6 + 180;
    }else if (theta6 > 0) {
      theta6 = theta6 - 180;
    }
  }


  double thetac[6] = {theta1, theta2+360, theta3, theta4, theta5, theta6};
  double Ac1[4][4] = {
    {cos(thetac[0]*PI/180),0,-sin(thetac[0]*PI/180),0},
    {sin(thetac[0]*PI/180),0,cos(thetac[0]*PI/180),0},
    {0,-1,0,l[0]},
    {0,0,0,1}
  };
  double Ac2[4][4] = {
    {cos(thetac[1]*PI/180),0,sin(thetac[1]*PI/180),0},
    {sin(thetac[1]*PI/180),0,-cos(thetac[1]*PI/180),0},
    {0,1,0,0},
    {0,0,0,1}
  };
  double Ac3[4][4] = {
    {1,0,0,0},
    {0,0,1,0},
    {0,-1,0,l[1]},
    {0,0,0,1}
  };
  double Ac4[4][4] = {
    {cos(thetac[2]*PI/180),0,sin(thetac[2]*PI/180),0},
    {sin(thetac[2]*PI/180),0,-cos(thetac[2]*PI/180),0},
    {0,1,0,0},
    {0,0,0,1}
  };
  double Ac5[4][4] = {
    {cos(thetac[3]*PI/180),0,-sin(thetac[3]*PI/180),0},
    {sin(thetac[3]*PI/180),0,cos(thetac[3]*PI/180),0},
    {0,-1,0,l[2]},
    {0,0,0,1}
  };
  double Ac6[4][4] = {
    {cos(thetac[4]*PI/180),0,sin(thetac[4]*PI/180),0},
    {sin(thetac[4]*PI/180),0,-cos(thetac[4]*PI/180),0},
    {0,1,0,0},
    {0,0,0,1}
  };
  double Ac7[4][4] = {
    {cos(thetac[5]*PI/180),-sin(thetac[5]*PI/180),0,0},
    {sin(thetac[5]*PI/180),cos(thetac[5]*PI/180),0,0},
    {0,0,1,l[3]},
    {0,0,0,1}
  };

  double Tic[4][4];
  MatMult(Ac1, Ac2, Tic);
  memcpy(tempRes,Tic,sizeof(Tic));
  MatMult(tempRes, Ac3, Tic);
  memcpy(tempRes,Tic,sizeof(Tic));
  MatMult(tempRes, Ac4, Tic);
  memcpy(tempRes,Tic,sizeof(Tic));
  MatMult(tempRes, Ac5, Tic);
  memcpy(tempRes,Tic,sizeof(Tic));
  MatMult(tempRes, Ac6, Tic);
  memcpy(tempRes,Tic,sizeof(Tic));
  MatMult(tempRes, Ac7, Tic);

  cout<<"Ti: "<< endl;
  PrintMat(Ti);

  cout<<"calculated Ti: "<< endl;
  PrintMat(Tic);

}
