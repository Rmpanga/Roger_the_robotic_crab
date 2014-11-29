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

 if (obs.pos[X] <= -1)
 {
   roger->base_setpoint[X] = obs.pos[X];
   roger->base_setpoint[Y] = obs.pos[Y];
    

        //If does not work use X^2 + Y^2
//  if ((fabs(roger->base_setpoint[X] - roger->base_position[X]) <= 0.5) && (fabs(roger->base_setpoint[Y] - roger->base_position[Y]) <= 0.5)))  
//		PUNCH (roger ,time);
   
  
     if (((fabs(roger->base_setpoint[X] - roger->base_position[X]) <= 1) && fabs(roger->base_setpoint[Y] - roger->base_position[Y]) <=1)) 
         return CONVERGED;
  
 }
}

int RETREAT (roger) //OKAY
Robot* roger;
{

   roger->base_setpoint[X] =  -0.97643;
   roger->base_setpoint[Y] =  -0.005089;
   roger->base_setpoint[THETA] = roger->base_setpoint[THETA] + M_PI;
     
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

  if (roger->base_position[Y] > ball_y_pos)
      inv_arm_kinematics(roger , RIGHT , obs.pos[X] , obs.pos[Y]);
 
    else if (roger->base_position[Y] < ball_y_pos)
      inv_arm_kinematics(roger , LEFT , obs.pos[X] , obs.pos[Y]);

    //Punch both

//convert obs postiongs to world to base 


return 1; 
}

void reset_arms (roger)
Robot* roger;
{

roger->arm_setpoint[LEFT][0] = 2.827433;
roger->arm_setpoint[LEFT][1] = -2.827433;
roger->arm_setpoint[RIGHT][0] = -2.827433;
roger->arm_setpoint[RIGHT][1] = 2.827433;


}



int count = 0; 

void project4_control(roger, time)
Robot* roger;
double time;
{ 
  printf("%lf , %lf , %lf , %lf\n" , time , roger->base_position[THETA] , roger->base_position[X] , roger->base_position[Y]); 

//printf("BASE X: %lf , BASE Y: %lf\n" , roger->base_position[X] , roger->base_position[Y]);
//printf("*********************  %lf\n", fabs(roger->base_setpoint[X] - roger->base_position[X]));

/** CHEATING **/

if( count == 0)
{
if (roger->base_setpoint[X] == -3)
{
    roger->base_setpoint[X] = -2;
}
 count++;
 }
if (fabs(roger->base_setpoint[X] - roger->base_position[X]) <= 0.1)  
{
  roger->base_setpoint[X] = -3;
  roger->base_setpoint[Y] = 0;
  roger->base_setpoint[THETA] = roger->base_position[THETA];
  }
 
 // count++;

/*******/

   //roger->base_setpoint[Y] = obs.pos[Y];

/*
if ( CONVERGED == SEARCHTRACK(roger , time) && CONVERGED == TRACK(roger, time)) //May change with transient, optimize with lenght of arm for punch
 {
  if (CONVERGED == CHASE (roger , time))
   {
    if (CONVERGED == CHASE (roger , time))
     {
      PUNCH(roger , time); 

      if (roger->base_position[X] >= -2) //Back up if near center
      {
       RETREAT(roger);
      }
       
     }

      
 
    if (roger->ext_force[LEFT][1] > 0 || roger->ext_force[RIGHT][1] > 0)
    {   
     reset_arms(roger);
     RETREAT(roger);
    }

   }
// printf("%lf ,%lf , %lf, %lf, %lf, %lf\n",time , roger->base_position[X] , roger->base_position[Y] , roger->base_position[THETA] , roger->eye_theta[LEFT] , roger->eye_theta[RIGHT]);
 }
return;
*/
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