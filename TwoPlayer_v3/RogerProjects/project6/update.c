/*
 *  control.c
 *  
 *
 *  Created by Shiraj Sen on 3/31/13.
 *  Updated to comply with new gcc settings 4/2014 by Mike Lanighan.
 *  Copyright 2013 University of Massachusetts Amherst. All rights reserved.
 *
 */


#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int init_control_flag = TRUE;
void project4_control(), project4_reset(), project6_reset(), project6_control();
void inv_arm_kinematics();

/***********************************************************************/
/** Input modes --- update the setpoint data structure:            *****/ 
/**    JOINT_ANGLE_INPUT mode: mouse(q1,q2) -> arm (q1, q2) ref    *****/
/**      BASE_GOAL_INPUT mode: mouse(x,y) -> base (x,y) ref	   *****/
/**      ARM_GOALS_INPUT mode: mouse(x,y) -> arm (x,y) ref  	   *****/
/**        BALL_POSITION mode: mouse(x,y) -> red ball (x,y)        *****/
/**           MAP_EDITOR mode: left button(x,y) -> obstacle        *****/
/**                            right button(x,y) -> goal           *****/
/***********************************************************************/
void update_setpoints(roger)
Robot * roger;
{
	//*******************************************************************
	//******** perform control based on selected control mode ***********
	//*******************************************************************
	
	switch(roger->control_mode) {
			
		case CHASE_PUNCH:
			project4_control(roger, roger->simtime);
			break;
			
		case PONG:
			if (roger->is_new_serve == 1)
			{
				printf("NEW SERVE \n");
				project6_reset(roger);
			}
			project6_control(roger,roger->simtime);
			break;
		default: 
			break;
	}
}

//check if input is in configuration space area and return angles
int isConfigurationInput(x, y, q1, q2, side) 
double x, y;
double *q1, *q2;
int *side;
{
  double range = 0.0;		
	
  *q1 = 0.0;
  *q2 = 0.0;
  *side = LEFT;
	
  //printf("Conf space check: %6.4lf, %6.4lf \n", x, y);
	
  //check if click is in configuration space area
  //X-Direction
  if (x < D2WX(100.0, T12LD(T1_MIN)) || 
      (x > D2WX(100.0, T12LD(T1_MAX)) && x < D2WX(100.0, T12RD(T1_MIN))) || 
      x > D2WX(100.0, T12RD(T1_MAX))) {
    //printf("x-location out of bounds!!! %6.4lf \n", x);
    return FALSE;
  }
  //Y-Direction
  else if (y < D2WY(100.0, T22LD(T2_MIN)) || y > D2WY(100.0, T22LD(T2_MAX)) ) {
    //printf("y-location out of bounds!!! %6.4lf\n", y);
    return FALSE;
  }	
	
  //calculate joint angles from click locations
  //left arm
  if (x < D2WX(100.0, T12LD(T1_MAX))) {
    *side = LEFT;
    range = D2WX(100.0, T12LD(T1_MAX)) - D2WX(100.0, T12LD(T1_MIN));
    *q1 = (x - (D2WX(100.0, T12LD(T1_MIN)) + range/2.0)) / (range/2.0) * M_PI;
    range = D2WY(100.0, T22LD(T2_MAX)) - D2WY(100.0, T22LD(T2_MIN));
    *q2 = (y - (D2WY(100.0, T22LD(T2_MIN)) + range/2.0)) / (range/2.0) * M_PI;
  }
  //right arm
  else { 
    *side = RIGHT;
    range = D2WX(100.0, T12RD(T1_MAX)) - D2WX(100.0, T12RD(T1_MIN));
    *q1 = (x - (D2WX(100.0, T12RD(T1_MIN)) + range/2.0)) / (range/2.0) * M_PI;
    range = D2WY(100.0, T22RD(T2_MAX)) - D2WY(100.0, T22RD(T2_MIN));
    *q2 = (y - (D2WY(100.0, T22RD(T2_MIN)) + range/2.0)) / (range/2.0) * M_PI;
  }
  return TRUE;
}

//check if input is in cartesian space area
int isCartesianInput(x_input, y_input, x, y) 
double x_input, y_input;
double *x, *y;
{
	if (x_input < MIN_X || x_input > MAX_X ||
		y_input < MIN_Y || y_input > MAX_Y) {
		//printf("Location out of bounds!!!\n");
		return FALSE;
	}
	*x = x_input;
	*y = y_input;
	
	return TRUE;
}

void Teleoperation_Cartesian_input_arms(roger, x, y, button)
Robot* roger;
double x;
double y;
int button;
{
  int limb = 0;
	
  printf("Arm goal input - x: %4.3f, y: %4.3f - button: %d\n", x, y, button);
	
  limb = 0;
  if (button == LEFT_BUTTON) {
    limb = LEFT;
    printf("world frame input for LEFT arm x=%6.4lf y=%6.4lf\n", x, y);
  }
  else if (button == RIGHT_BUTTON) {
    limb = RIGHT;
    printf("world frame input for RIGHT arm x=%6.4lf y=%6.4lf\n", x, y);
  }
  else return;
  inv_arm_kinematics(roger, limb, x,y); //PROJECT #2 - Kinematics.c
}

void Teleoperation_Cartesian_input_base(roger, x, y, button)
Robot* roger;
double x;
double y;
int button;
{
	double dx, dy, theta;
	
	roger->base_setpoint[X] = x;
	roger->base_setpoint[Y] = y;
	dx = x - roger->base_position[X];
	dy = y - roger->base_position[Y];
	theta = roger->base_setpoint[THETA] = atan2(dy,dx);
	printf("world frame Base goal: x=%6.4lf y=%6.4lf theta=%6.4lf\n", 
		   x, y, theta);
}

// initialize all setpoints to reflect the current posture, set the Cartesian
//    button_reference as appropriate for the new mode (base, left arm,
//       right arm, stereo head, harmonic function, integrated app)
void initialize_control_mode(roger)
Robot * roger;
{
  double wTb[4][4],ref_w[4],ref_b[4];
  int i,j;
	
  // define all setpoints to current positions and zero velocities
  roger->base_setpoint[X] = roger->base_position[X];// + BASE_CONTROL_OFFSET*cos(roger->base_position[THETA]);
  roger->base_setpoint[Y] = roger->base_position[Y];// + BASE_CONTROL_OFFSET*sin(roger->base_position[THETA]);
  roger->base_setpoint[THETA] = roger->base_position[THETA];
	
  roger->arm_setpoint[LEFT][0] = roger->arm_theta[LEFT][0];
  roger->arm_setpoint[LEFT][1] = roger->arm_theta[LEFT][1];
  roger->arm_setpoint[RIGHT][0] = roger->arm_theta[RIGHT][0];
  roger->arm_setpoint[RIGHT][1] = roger->arm_theta[RIGHT][1];
	
  roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT];
  roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT];
	
  //call the reset method of the last mode --> clear all thing you modified
  switch ((roger->control_mode + N_CONTROL_MODES - 1) % N_CONTROL_MODES) 
    // "+ N_CONTROL_MODES" needed as modulo of negative numbers incorrect
    {
    case CHASE_PUNCH: 
      project4_reset(roger);
      break;
    case PONG: 
      project6_reset(roger); 
      break;
    default:
      break;
    }
  init_control_flag = FALSE;  
}
