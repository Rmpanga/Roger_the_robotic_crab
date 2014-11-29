/*************************************************************************/
/* File: project2.c                                                      */
/* Description: Inverse Kinematic function for Arms                      */
/*                 inv_arm_kinematics() - writes arm cspace setpoints    */
/*                 (stand alone - outside control cycle)                 */
/*              Stereo Triangulate code (in control cycle)               */
/*                 stereo_observation() - (vision.c) computes Cartesian  */
/*                 observations (mean, cov), visualize procedure draws   */
/*                 observation to canvas                                 */
/* Date:        9-1-2014                                                 */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

void construct_wTb(), inverse_homogeneous_xform(), matrix_times_vector();

//For plotting purposes
double GlobalX;
double GlobalY;
int    GlobalL;

/**********************************************************************/
/*** PROJECT #2 - INVERSE KINEMATICS: MAP (X,Y) TO (THETA1, THETA2) ***/
/**********************************************************************/
int inv_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;
  GlobalX = x; 
  GlobalY = y;
  GlobalL = limb;

  //printf("Testing base cords x= %lf , y=%lf & y\n" , x , y);
  // transform world frame (x,y)_w input into base coordinates (x,y)_b
  construct_wTb(roger->base_position, wTb);
  inverse_homogeneous_xform(wTb, bTw);
  ref_w[X] = x;
  ref_w[Y] = y;
  ref_w[2] = 0;
  ref_w[3] = 1;
  
  matrix_times_vector(bTw ,ref_w , ref_b);
 
 
  // add the arm offset for the approariate arm to get it to shoulder frame
  if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
  else ref_b[Y] += ARM_OFFSET;

  x = ref_b[X];
  y = ref_b[Y]; 
  
  // implement the inverse kinematic algorithm
   r2 = x * x + y * y;
   c2 = (r2 - LARM_1 * LARM_1  - LARM_2 * LARM_2 ) / (2 * LARM_1 * LARM_2);
   if ( -1 <= c2 && c2 <= 1){
   s2_plus  =  sqrt(1 - c2 *c2);
   s2_minus = -1 * sqrt(1 - c2 *c2);
  
   theta2_plus = atan2(s2_plus ,  c2);
   theta2_minus = atan2(s2_minus , c2);
  
   k1 = LARM_1 + LARM_2 * c2;
   k2_plus = LARM_2 * s2_plus;
   k2_minus = LARM_2 * s2_minus;
  
   alpha_plus = atan2(k2_plus , k1);
   alpha_minus = atan2(k2_minus , k1);

   theta1_plus = atan2(y, x) - alpha_plus;
   theta1_minus = atan2(y, x) - alpha_minus;

  // pick one of the two solutions and write it into the setpoint of the arm
  
if (limb == RIGHT)
{
  roger->arm_setpoint[limb][0] = theta1_plus; 
  roger->arm_setpoint[limb][1] = theta2_plus;
}
else 
{ roger->arm_setpoint[limb][0] = theta1_minus; 
  roger->arm_setpoint[limb][1] = theta2_minus;

}

  //Here I input the equations for plotting
 // printf("Arm set point limb is %lf\n " , roger->arm_setpoint[limb][0]);
 // printf("Arm set point limb is %lf\n " , roger->arm_setpoint[limb][1]);
  return(TRUE);
}
  else {
    // return TRUE if there was a solution, FALSE if the location is out of reach
  return(FALSE);
        }
}

/**********************************************************************/
/*** PROJECT #2 - triangulate on the red ball                       ***/
/**********************************************************************/

void draw_observation(); // in simulator file xrobot.c

Observation obs;

/*************************************************************************/
/* project development interface - called by GUI                         */
/* project2_control() is executed automatically when                     */
/* control mode = PROJECT2; input mode = BALL INPUTS                     */
/*************************************************************************/
void project2_control(roger, time)
Robot * roger;
double time;
{
  int stereo_observation();

 // printf("STEREO OBSERVATION state=%d\n", stereo_observation(roger, &obs));
 //x = l1c1 + l2c12  y = l1s1 + l2s12

//s12 = s1c2 + c1s2
//c12 = c1c2 − s1s2

 double tempx = LARM_1 * cos(roger->arm_theta[GlobalL][0]) + (LARM_2 * (cos(roger->arm_theta[GlobalL][0]) * cos(roger->arm_theta[GlobalL][1])) - (sin(roger->arm_theta[GlobalL][0]) * sin(roger->arm_theta[GlobalL][1])));

 double tempy = LARM_1 * sin(roger->arm_theta[GlobalL][0]) + (LARM_2 * (sin(roger->arm_theta[GlobalL][0]) * cos(roger->arm_theta[GlobalL][1])) - (cos(roger->arm_theta[GlobalL][0]) * sin(roger->arm_theta[GlobalL][1])));
 
//printf("%lf, %lf\n",tempx, tempy);
//   printf("%lf, %lf, %lf" , time , xref - xerr , yref - yact);
      //printf("%lf, %lf\n" , GlobalX , GlobalY);
     // printf("%lf, %lf, %lf\n", time , (GlobalX - tempx), (GlobalY - tempy) );
  // check if ball is in view
  // write Observation "obs" = mean and cov in world coordinates
  //  if (stereo_observation(roger, &obs)) {          // in vision.c
  //     printf("PROJECT 2: stereo_observation()- x=%6.4lf y=%6.4lf\n",
  //        obs.pos[X], obs.pos[Y]);
  //     printf("                  %lf %lf\n", obs.cov[0][0], obs.cov[0][1]);
  //     printf("                  %lf %lf\n\n",obs.cov[1][0],obs.cov[1][1]);
  //  }
  //  else printf("no valid stereo observation!\n");
}

void project2_reset(roger)
Robot* roger;
{ }

void project2_enter_params() 
{
  printf("Project 2 enter_params called. \n");
}

void project2_visualize(roger)
Robot* roger;
{ 
  void draw_observation();
  draw_observation(obs); /* defined in xrobot.c */
}


