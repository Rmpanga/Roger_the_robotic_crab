/* File:        MotorUnits.c                                             */
/* Description: motor units execute every simulated millisecond and are  */
/*              never disengaged, applications control Roger by altering */
/*              setpoints for each degree of freedom                     */
/* FUNCTION LIBRARY (precedes project code):                             */
/*    construct_wTb(roger->base_position, wTb) computes the homogeneous  */
/*          transform (double wTb[4][4]) relating the base frame to the  */
/*          to the world frame given input robot state (double x,y,theta)*/
/*    inverse_homogeneous_xform(aTb, bTa) - inverts a homogeneous xform  */
/*    matrix_times_vector(A,x,y) - multiplies double A[4][4] and input   */
/*          double x[4] to yield output double y[4] (y=Ax) - can be used */
/*          to apply homogeneous transforms to homogeneous vectors       */
/* Date:        11-2012                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

void update_setpoints();

/**********************************************************************/
/************* LIBRARY CODE *******************************************/
/**********************************************************************/
void construct_wTb(base_pos, wTb)
double base_pos[3]; // (x,y,theta)
double wTb[4][4];
{
  double s0, c0;
  s0 = sin(base_pos[2]);
  c0 = cos(base_pos[2]);

  wTb[0][0] = c0;  wTb[0][1] = -s0; wTb[0][2] = 0.0; wTb[0][3] = base_pos[X];
  wTb[1][0] = s0;  wTb[1][1] = c0;  wTb[1][2] = 0.0; wTb[1][3] = base_pos[Y];
  wTb[2][0] = 0.0; wTb[2][1] = 0.0; wTb[2][2] = 1.0; wTb[2][3] = 0.0;
  wTb[3][0] = 0.0; wTb[3][1] = 0.0; wTb[3][2] = 0.0; wTb[3][3] = 1.0;
}

void inverse_homogeneous_xform(in, out)
double in[4][4], out[4][4];
{
  int i,j;
  for (i=0; i<3; ++i) {
    for (j=0; j<3; ++j) {
      out[i][j] = in[j][i];
    }
  }
  out[3][0] = out[3][1] = out[3][2] = 0.0; out[3][3] = 1.0;
  
  out[0][3] = -in[0][3]*in[0][0] - in[1][3]*in[1][0] - in[2][3]*in[2][0];
  out[1][3] = -in[0][3]*in[0][1] - in[1][3]*in[1][1] - in[2][3]*in[2][1];
  out[2][3] = -in[0][3]*in[0][2] - in[1][3]*in[1][2] - in[2][3]*in[2][2];
}

void matrix_times_vector(mat, in, out)    // out = mat x in
double mat[4][4], in[4], out[4];
{
  int i,j;
  double sum;
  for (i=0; i<4; ++i) {
    sum = 0.0;
    for (j=0; j<4; ++j) {
      sum += mat[i][j] * in[j];
    }
    out[i] = sum;
  }
}

/*************************************************************************/
/* PROJECT #1 - COMPLETE THE FOLLOWING CONTROL PROCEDURES                */
/*************************************************************************/

//B = sqrt(K * I) I = 0.00008
// initial (bad) gains for the PD controllers for the eyes
double kp_eye = 5.0; //KP_EYE   
double kd_eye = 0.04; //KD_EYE; 

/* PROJECT #1.1 - PD CONTROLLER FOR THE EYES                             */
/* setpoints are joint angle values in radians for the eyes              */
void PDController_eyes(roger, time)
Robot * roger;
double time;
{
  int i;
  double theta_error, theta_ddot_des;
  
  for (i = 0; i < NEYES; i++) {
    theta_error = roger->eyes_setpoint[i] - roger->eye_theta[i];
    roger->eye_torque[i] = (kp_eye * theta_error) -( kd_eye * roger->eye_theta_dot[i]);
  }
}

// initial (bad) gains for the PD controllers for the arms 
double kp_arm = 12000; //K
double kd_arm = 77.45966924; //B

/* PROJECT #1.2 - PD CONTROLLER FOR THE ARMS                             */
/* setpoints - joint angles in radians for the shoulders and elbows      */
void PDController_arms(roger, time)
Robot * roger;
double time;
{
  int i;
  double theta_error[2];

  for (i=LEFT; i<=RIGHT; ++i) {
    // PDcontrol - desired accelerations
    theta_error[0] = roger->arm_setpoint[i][0] - roger->arm_theta[i][0];
    theta_error[1] = roger->arm_setpoint[i][1] - roger->arm_theta[i][1];


    //shoulder - bound error -pi < error < +pi
    while (theta_error[0] > M_PI) theta_error[0] -= 2.0 * M_PI;
    while (theta_error[0] < -M_PI) theta_error[0] += 2.0 * M_PI;
    //elbow - bound error -pi < error < +pi
    while (theta_error[1] > M_PI) theta_error[1] -= 2.0 * M_PI;
    while (theta_error[1] < -M_PI) theta_error[1] += 2.0 * M_PI;

    // tune kp_arm and kd_arm by changing their value using
    // "Enter Params" button

    roger->arm_torque[i][0] = (kp_arm * theta_error[0]) - (kd_arm * roger->arm_theta_dot[i][0]); 
    roger->arm_torque[i][1] = (kp_arm * theta_error[1]) - (kd_arm * roger->arm_theta_dot[i][1]); 
  }
}

// initial (bad) gains for base translational controller
double kp_base_trans = 12000;
double kd_base_trans = 293.938769134;

/* Base PD controller, Cartesian reference */
double PDBase_translate(roger, time) 
Robot * roger;
double time;
{
 
  double Fx;
  double transError; 
  double rx;
  double ry;
  double velocity;

   double te = roger->base_setpoint[Y] -roger->base_position[Y];

  rx = roger->base_setpoint[X] - roger->base_position[X];
  ry = roger->base_setpoint[Y] - roger->base_position[Y];

  transError = rx *  cos(roger->base_position[THETA]) + ry * sin(roger->base_position[THETA]);
  velocity =  roger->base_velocity[X] * cos(roger->base_position[THETA]) + ry * sin(roger->base_position[THETA]);
  Fx = kp_base_trans * transError - kd_base_trans * velocity;
  
  return(Fx);
}

// initial (bad) gains for base rotational controller
double kp_base_rot = 12000; //KP_BASE
double kd_base_rot = 293.938769134;   //KD_BASE

/* Base PD controller, Cartesian reference */
double PDBase_rotate(roger, time) 
Robot * roger;
double time;
{
  double Mz;

  double rx;
  double ry;
  double targetTheta;
  double thetaerror;

   rx = roger->base_setpoint[X] - roger->base_position[X];
   ry = roger->base_setpoint[Y] - roger->base_position[Y];

    //targetTheta = atan2(ry, rx);
    targetTheta = roger->base_setpoint[THETA];
    thetaerror  = targetTheta - roger->base_position[THETA];

    while (thetaerror > M_PI) thetaerror -= 2.0 * M_PI;
    while (thetaerror < -M_PI) thetaerror += 2.0 * M_PI;
  //  roger->base_setpoint[THETA] = M_PI*2; 
 //  printf( "%lf , %lf\n", time, thetaerror );
   Mz = kp_base_rot * thetaerror -  (kd_base_trans * roger->base_velocity[THETA]);

  return(Mz);
}

/* PROJECT #1.3 - PD CONTROLLER FOR THE BASE                             */
/* setpoints - (xy) location for translation heading in radians          */

//the base differential drive Jacobian:
//double baseJT[2][2] = ...

void PDController_base(roger, time)
Robot * roger;
double time;
{ 
  double Fx, Mz, PDBase_translate(), PDBase_rotate();

  Fx = PDBase_translate(roger,time);
  Mz = PDBase_rotate(roger,time);

  double tl = (.5 * Fx) + -1/(2 * R_AXLE ) * Mz;
  double tr = (.5 * Fx) +  1/(2 * R_AXLE ) * Mz;

  // integrated wheel torque control
  roger->wheel_torque[LEFT] = tl; 
  roger->wheel_torque[RIGHT]= tr;
}

/*************************************************************************/
/*       THE SIMULATOR EXECUTES control_roger() EVERY CONTROL CYCLE      */
/*                        *** DO NOT ALTER ***                           */
/*************************************************************************/
void control_roger(roger, time)
Robot * roger;
double time;
{

  //printf("arm set points: left 0 : %lf , left 1 %lf , right 0 %lf ,  right 1: %lf\n" ,  roger->arm_theta[LEFT][0] , roger->arm_theta[LEFT][1] , roger->arm_theta[RIGHT][0] , roger->arm_theta[RIGHT][1]);
  update_setpoints(roger);

  // turn setpoint references into torques
  PDController_eyes(roger, time);
  PDController_arms(roger, time);
  PDController_base(roger,time);
}

/*************************************************************************/
void project1_reset(roger)
Robot* roger;
{ }

/*************************************************************************/
// prompt for and read user customized input values                             
/*************************************************************************/
void project1_enter_params()
{
  // put anything in here that you would like to change at run-time
  // without re-compiling user project codes
    printf("Enter K rot and B rot\n");
    scanf("%lf %lf", &kp_base_rot, &kd_base_rot);
    printf("Enter K trans and B trans\n");
    scanf("%lf %lf",  &kp_base_trans , &kd_base_trans); 
    printf("K rot: %lf , B rot: %lf\n" , kp_base_rot , kd_base_rot );
    printf("K trans: %lf , B trans: %lf\n" , kp_base_trans, kd_base_trans);
 // printf("ARM: enter 'K B'\n"); fflush(stdout);
 // scanf("%lf %lf", &kp_arm, &kd_arm);

//  printf("EYE: K=%6.4lf  B=%6.4lf\n", kp_eye, kd_eye);
 // scanf("%lf %lf", &kp_eye, &kd_eye);
//  printf("EYE: K=%6.4lf  B=%6.4lf\n", kp_eye, kd_eye);
//  printf("ARM: K=%6.4lf  B=%6.4lf\n", kp_arm, kd_arm);
}

/*************************************************************************/
// function called when the 'visualize' button on the gui is pressed            
void project1_visualize(roger)
Robot* roger;
{ }

