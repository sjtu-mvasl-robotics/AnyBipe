#include "jacobian.h"

std::vector<double> state_vel_track(std::vector<double> q, std::vector<double> dq, std::vector<double> q_init, std::vector<double> l, bool left_leg)
{
  double Jacobian[3][3]={0.0};
  double l1=l[0];
  double l2=l[1];
  auto q_in0=q_init[0];
  auto q_in1=q_init[1];
  auto q_in2=q_init[2];
  auto q_in3=q_init[3];
  auto q_in4=q_init[4];
  auto q_in5=q_init[5];
  std::vector<double> state_vel;

  if (left_leg)
  {
    double theta1=q_in0-q[0];///q_in0:q0_joint_initial_angle,q[0]:q0_joint_angle
    double theta2=q_in1+q[1];///q_in1:q1_joint_initial_angle,q[1]:q1_joint_angle
    double theta3=q_in2-q[2];///q_in2:q2_joint_initial_angle,q[2]:q2_joint_angle

    double dtheta1=-dq[0];///relationship of dq and dtheta
    double dtheta2=dq[1];
    double dtheta3=-dq[2];


    Jacobian[0][0] = 0.0;
    Jacobian[0][1] = l1*cos(theta2)+l2*cos(theta2+theta3);
    Jacobian[0][2] = l2*cos(theta2+theta3);

    Jacobian[1][0] = l1*cos(theta2)*cos(theta1)+l2*cos(theta2+theta3)*cos(theta1);
    Jacobian[1][1] = l1*(-sin(theta2))*sin(theta1)+l2*(-sin(theta2+theta3))*sin(theta1);
    Jacobian[1][2] = l2*(-sin(theta2+theta3))*sin(theta1);

    Jacobian[2][0] = l1*cos(theta2)*(-sin(theta1))+l2*cos(theta2+theta3)*(-sin(theta1));
    Jacobian[2][1] = l1*(-sin(theta2))*cos(theta1)+l2*(-sin(theta2+theta3))*cos(theta1);
    Jacobian[2][2] = l2*(-sin(theta2+theta3))*cos(theta1);

    double x_dot_left=dtheta1*Jacobian[0][0]+dtheta2*Jacobian[0][1]+dtheta3*Jacobian[0][2];//the left foot linear velocity related to the hip joint in x direction
    double y_dot_left=dtheta1*Jacobian[1][0]+dtheta2*Jacobian[1][1]+dtheta3*Jacobian[1][2];//the left foot linear velocity related to the hip joint in y direction
    double z_dot_left=dtheta1*Jacobian[2][0]+dtheta2*Jacobian[2][1]+dtheta3*Jacobian[2][2];//the left foot linear velocity related to the hip joint in z direction
    state_vel.push_back(x_dot_left);
    state_vel.push_back(y_dot_left);
    state_vel.push_back(z_dot_left);

  }

  else
  {
    double theta1=q_in3-q[3];///q_in3:q3_joint_initial_angle,q[3]:q3_joint_angle
    double theta2=q_in4-q[4];///q_in4:q4_joint_initial_angle,q[4]:q4_joint_angle
    double theta3=q_in5+q[5];///q_in5:q5_joint_initial_angle,q[5]:q5_joint_angle

    double dtheta1=-dq[3];///relationship of dq and dtheta
    double dtheta2=-dq[4];
    double dtheta3=dq[5];


    Jacobian[0][0] = 0.0;
    Jacobian[0][1] = l1*cos(theta2)+l2*cos(theta2+theta3);
    Jacobian[0][2] = l2*cos(theta2+theta3);

    Jacobian[1][0] = l1*cos(theta2)*cos(theta1)+l2*cos(theta2+theta3)*cos(theta1);
    Jacobian[1][1] = l1*(-sin(theta2))*sin(theta1)+l2*(-sin(theta2+theta3))*sin(theta1);
    Jacobian[1][2] = l2*(-sin(theta2+theta3))*sin(theta1);

    Jacobian[2][0] = l1*cos(theta2)*(-sin(theta1))+l2*cos(theta2+theta3)*(-sin(theta1));
    Jacobian[2][1] = l1*(-sin(theta2))*cos(theta1)+l2*(-sin(theta2+theta3))*cos(theta1);
    Jacobian[2][2] = l2*(-sin(theta2+theta3))*cos(theta1);

    double x_dot_right=dtheta1*Jacobian[0][0]+dtheta2*Jacobian[0][1]+dtheta3*Jacobian[0][2];//the right foot linear velocity related to the hip joint in x direction
    double y_dot_right=dtheta1*Jacobian[1][0]+dtheta2*Jacobian[1][1]+dtheta3*Jacobian[1][2];//the right foot linear velocity related to the hip joint in y direction
    double z_dot_right=dtheta1*Jacobian[2][0]+dtheta2*Jacobian[2][1]+dtheta3*Jacobian[2][2];//the right foot linear velocity related to the hip joint in z direction
    state_vel.push_back(x_dot_right);
    state_vel.push_back(y_dot_right);
    state_vel.push_back(z_dot_right);
  }
  
  return state_vel;
}

std::vector<double> feet_distance_track(std::vector<double> q, std::vector<double> q_init, std::vector<double> l){
  
  double l1=l[0];
  double l2=l[1];
  // double endposiition[6]={0.0};
  //float PI=3.1415926;
  auto q_in0=q_init[0];
  auto q_in1=q_init[1];
  auto q_in2=q_init[2];
  auto q_in3=q_init[3];
  auto q_in4=q_init[4];
  auto q_in5=q_init[5];
  double ltheta1=q_in0-q[0];///q_in0:q0_joint_initial_angle,q[0]:q0_joint_angle
  double ltheta2=q_in1+q[1];///q_in1:q1_joint_initial_angle,q[1]:q1_joint_angle
  double ltheta3=q_in2-q[2];///q_in2:q2_joint_initial_angle,q[2]:q2_joint_angle

  double rtheta1=q_in3-q[3];///q_in3:q3_joint_initial_angle,q[3]:q3_joint_angle
  double rtheta2=q_in4-q[4];///q_in4:q4_joint_initial_angle,q[4]:q4_joint_angle
  double rtheta3=q_in5+q[5];///q_in5:q5_joint_initial_angle,q[5]:q5_joint_angle


  //printf("%lf %lf %lf\n",ltheta1,ltheta2,ltheta3);

  // double l_thigh = l1,l_shank = l2;//左腿大小腿的长度，thigh为大腿，shank为小腿
  // double r_thigh = l1,r_shank = l2;//右腿大小腿的长度

  // double leglength_l = sqrt(l_thigh*l_thigh+l_shank*l_shank-2*l_shank*l_thigh*cos(PI-abs(ltheta3)));//3D情况下的长度
  // double leglength_r = sqrt(r_thigh*r_thigh+r_shank*r_shank-2*r_shank*r_thigh*cos(PI-abs(rtheta3)));
  // //printf("%lf\n",leglength_l);
  // double a_l=acos((l_thigh*l_thigh+leglength_l*leglength_l-l_shank*l_shank)/(2*l_thigh*leglength_l));
  // double a_r=acos((r_thigh*r_thigh+leglength_r*leglength_r-r_shank*r_shank)/(2*r_thigh*leglength_r));

  // double a_l2=ltheta2-a_l;
  // //printf("%lf\n",a_l2);
  // double a_r2=rtheta2-a_r;

  std::vector<double> endposition = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  //该末端位置为足相对于对应腿髋关节相对位置，如需计算末端关于上身速度，则需在x、y、z方向增加相对上身坐标系的偏移量
  //printf("ltheta1: %lf, ltheta2: %lf, ltheta3: %lf\n", ltheta1, ltheta2, ltheta3);
  endposition[0] = l1*sin(ltheta2)+l2*sin(ltheta2+ltheta3); //左腿x方向的坐标
  endposition[1] = (l1*cos(ltheta2)+l2*cos(ltheta2+ltheta3))*sin(ltheta1);//左腿y方向的坐标
  endposition[2] = -(l1*cos(ltheta2)+l2*cos(ltheta2+ltheta3))*cos(ltheta1);//左腿z方向的坐标
  
  endposition[3] = l1*sin(rtheta2)+l2*sin(rtheta2+rtheta3);
  endposition[4] = (l1*cos(rtheta2)+l2*cos(rtheta2+rtheta3))*sin(rtheta1);
  endposition[5] = -(l1*cos(rtheta2)+l2*cos(rtheta2+rtheta3))*cos(rtheta1);

  return endposition;

}