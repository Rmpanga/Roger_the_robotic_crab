/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #3 - observe, filter, and visualize         */
/*                                (no motor control)                     */
/* Date:        6-04-2013                                                */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
// #include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

extern Estimate x_plus;      // declared in ./kalman.c
extern Observation obs;      // declared in ../project2-Kinematics/project2/.c
extern double sigma_process; // declared in ./kalman.c
extern double sigma_obs;     // declared in ./kalman.c

void init_filter_parameters(), kalman_filter(), draw_estimate(), x_draw_line();

/* RETURN STATUS FOR ALL ACTIONS:                                        */
/* 0 - "DONT_KNOW", 1 - "NO_REFERENCE", 2 - "TRANSIENT", 3 - "CONVERGED" */

int FilterInit = TRUE;
/*************************************************************************/
/* project development interface - called by GUI                         */
/*************************************************************************/
void project5_control(roger,time)
Robot * roger;
double time;
{
  if (FilterInit == TRUE) {
    init_filter_parameters();
    FilterInit = FALSE;
  }
  else kalman_filter(roger, time); // update state = [ x y xdot ydot ]^T
}

/*************************************************************************/
void project5_reset(roger)
Robot* roger;
{ }

/*************************************************************************/
void project5_enter_params() 
{
  printf("Project 3 enter_params called. \n");
  printf("sigma_obs = %6.4lf    sigma_process = %6.4lf\n",
	 sigma_obs, sigma_process);
  printf("enter new 'sigma_obs sigma_process <CR>'\n"); fflush(stdout);
  scanf("%lf %lf", &sigma_obs, &sigma_process);
}

#define VMAG 0.5
/*************************************************************************/
// function called when the 'visualize' button on the gui is pressed
void project5_visualize(roger)
Robot* roger;
{
  //draw_estimate(10.0, x_plus);
  //x_draw_line(BLUE, x_plus.state[X], x_plus.state[Y], (x_plus.state[X] + VMAG*x_plus.state[XDOT]), (x_plus.state[Y] + VMAG*x_plus.state[YDOT]));
}

