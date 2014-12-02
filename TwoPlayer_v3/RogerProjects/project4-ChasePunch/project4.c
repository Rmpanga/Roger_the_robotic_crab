/*************************************************************************/
/* File:        project4.c                                               */
/* Description: User project #4 - empty project directory for project    */
/*              developement                                             */
/* Date:        03-30-2013                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"


extern Observation obs;
int CHASE (roger , time)
Robot* roger;
double time;
{

if (obs.pos[X] < -0.9 && obs.pos[Y] <1.3 && obs.pos[Y] >-1.3) //FOR TWO PLAYER
 {
//   printf("Chasing the ball since its X cord: %lf\n" , obs.pos[X]);
   roger->base_setpoint[X] = obs.pos[X];
   roger->base_setpoint[Y] = obs.pos[Y];
    

        //If does not work use X^2 + Y^2
//  if ((fabs(roger->base_setpoint[X] - roger->base_position[X]) <= 0.5) && (fabs(roger->base_setpoint[Y] - roger->base_position[Y]) <= 0.5)))  
//		PUNCH (roger ,time);


 //    if (((fabs(roger->base_setpoint[X] - roger->base_position[X]) <= 1) && fabs(roger->base_setpoint[Y] - roger->base_position[Y]) <=1)) 
if (inReach(roger , LEFT, obs.pos[X] , obs.pos[Y]) == TRUE)   
      return CONVERGED;
if (inReach(roger , RIGHT, obs.pos[X] , obs.pos[Y]) == TRUE)   
      return CONVERGED;
  
 }
 return NO_REFERENCE; //FOR TWO PLAYER
}


int inReach(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;
  double GlobalX = x; 
  double GlobalY = y;
  double GlobalL = limb;

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

 return (TRUE);
}
  else {
    // return TRUE if there was a solution, FALSE if the location is out of reach
  return(FALSE);
        }
}

int STOP(roger)
Robot * roger;
{
  roger->base_setpoint[X] = roger->base_position[X];
  roger->base_setpoint[Y] = roger->base_position[Y];
  printf("Roger is stopping\n");
}


int CHASEPUNCH (roger , time)
Robot* roger;
double time;
{

  double ball_x_pos = obs.pos[X];
  double ball_y_pos = obs.pos[Y];



 if (obs.pos[X] < -0.35 && obs.pos[Y] <1.5 && obs.pos[Y] >-1.5) //FOR TWO PLAYER
 {
   roger->base_setpoint[X] = obs.pos[X];
   roger->base_setpoint[Y] = obs.pos[Y];
    

  if (roger->base_position[Y] > ball_y_pos)
   {
    if (inv_arm_kinematics(roger , RIGHT , obs.pos[X] , obs.pos[Y]) == TRUE)
    {
       printf("Ball is withing right arm reach... punching\n");
       return CONVERGED;
    }
     else 
      printf("Ball is out of reach of right arm\n"); 
    }
    
    if (roger->base_position[Y] < ball_y_pos)
     { 
     if (inv_arm_kinematics(roger , LEFT , obs.pos[X] , obs.pos[Y]) == TRUE)
     {
       printf("Ball is within left arm reach...punching\n");
       return CONVERGED;
     }
     else
      printf("Ball is out of reach of left arm\n");
     }

 return TRANSIENT; 
 }
 return NO_REFERENCE; //FOR TWO PLAYER
}

int RETREAT (roger) //OKAY
Robot* roger;
{

   roger->base_setpoint[X] =  -2;
   roger->base_setpoint[Y] =   0;
   roger->base_setpoint[THETA] = roger->base_setpoint[THETA] + M_PI;
    printf("Falling back to position\n");
     
/*
        //If does not work use X^2 + Y^2
//  if ((fabs(roger->base_setpoint[X] - roger->base_position[X]) <= 0.5) && (fabs(roger->base_setpoint[Y] - roger->base_position[Y]) <= 0.5)))  
//		PUNCH (roger ,time);


     if (((fabs(roger->base_setpoint[X] - roger->base_position[X]) <= 1) && fabs(roger->base_setpoint[Y] - roger->base_position[Y]) <=1)) 
         return CONVERGED;
  */

}



int PUNCH (roger , time)
Robot* roger;
double time;
{
  double ball_x_pos = obs.pos[X];
  double ball_y_pos = obs.pos[Y];

//convert ball to base frame and if x is -negative punch left postive
//punch right
 //inv_arm_kinematics(roger, LEFT, obs.pos[X], obs.pos[Y]);

  if (roger->base_position[X] > ball_x_pos)
   {
     inv_arm_kinematics(roger , RIGHT , obs.pos[X] , obs.pos[Y]);
     inv_arm_kinematics(roger , LEFT , obs.pos[X] , obs.pos[Y]);
     return 1;
   }

  if (roger->base_position[Y] > ball_y_pos)
   {
      printf("Punching right arm\n");
      inv_arm_kinematics(roger , RIGHT , obs.pos[X] , obs.pos[Y]);
      return 1;
 }
  if (roger->base_position[Y] < ball_y_pos)
    {
      inv_arm_kinematics(roger , LEFT , obs.pos[X] , obs.pos[Y]);
        printf("Punching right arm\n");
      return 1;
     }



    //Punch both

//convert obs postiongs to world to base 


return 1; 
}

void reset_arms (roger , limb )
Robot* roger;
int limb;
{
 
 //printf("Reset Arms: Limb: %lf , Left: %lf , Right: %lf\n" , limb ,  LEFT, RIGHT);

 if (limb == LEFT)
 {
 printf("Resting left arm\n");
 roger->arm_setpoint[LEFT][0] = 2.827433;
 roger->arm_setpoint[LEFT][1] = -2.827433;
 //RETREAT(roger);
 }

 if (limb == RIGHT)
 {
 printf("Resting right arm\n");
 roger->arm_setpoint[1][0] = -2.827433;
 roger->arm_setpoint[1][1] = 2.827433;
 printf("Falling back to position\n");
 //RETREAT(roger);
 }

}





void project4_control(roger, time)
Robot* roger;
double time;
{ 

//printf("BASE X: %lf , BASE Y: %lf\n" , roger->base_position[X] , roger->base_position[Y]);
int searchtrack = SEARCHTRACK(roger, time);
int track = TRACK(roger ,time);

   /**Force detectors **/
 if (roger->ext_force[LEFT][1] > 0 )
  {   
   printf("Force on LEFT arm detected\n");
   reset_arms(roger , LEFT); 
  }
 if (roger->ext_force[RIGHT][1] > 0)
  {
    printf("Force on RIGHT arm detected\n");
    reset_arms(roger ,RIGHT);
  }


  if (roger->arm_theta_dot[LEFT][1] <= 0 && roger->arm_theta_dot[RIGHT][1] <= 0 )
   {
    reset_arms(roger , LEFT);
    reset_arms(roger , RIGHT);
   }

  PUNCH(roger, time);


printf("Ball X: %lf, Ball Y: %lf\n" , obs.pos[X], obs.pos[Y]);

if ( (CONVERGED == searchtrack || TRANSIENT == searchtrack) && (CONVERGED == track || TRANSIENT== track)) //May change with transient, optimize with lenght of arm for punch
 {

  if (CONVERGED == CHASE (roger , time))
   {
     STOP(roger); 
     PUNCH(roger , time);        
   }


       /**Force detectors **/
    if (roger->ext_force[LEFT][1] > 0 )
    {   
     printf("Force on LEFT arm detected\n");
     reset_arms(roger , LEFT , TRUE);
     RETREAT(roger);
     
    }
     if (roger->ext_force[RIGHT][1] > 0)
     {
      printf("Force on RIGHT arm detected\n");
      reset_arms(roger , RIGHT , TRUE);
      RETREAT(roger);
     }

   }
// printf("%lf ,%lf , %lf, %lf, %lf, %lf\n",time , roger->base_position[X] , roger->base_position[Y] , roger->base_position[THETA] , roger->eye_theta[LEFT] , roger->eye_theta[RIGHT]);
return;
}

/************************************************************************/
void project4_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project4_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project4_visualize(roger)
Robot* roger;
{ }
