/************************************************************************/
/**     control.c - the control stub for Roger-the-Crab                **/
/**     Spring 2011                                                    **/
/**     Rod Grupen                                                   %  **/
/************************************************************************/

#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "include/Roger.h"
#include "include/simulate.h"
#include "include/control.h"
#include "include/modes.h"

#define VERBOSE FALSE

Map world_model;   // empirical occupancy grid, potential grid, color assigment
int reset_flag;    // puts the simulator in the start-up state 


/* ====== DO NOT modify the code below ============================ */
// HACK
rectify_theta(arm)
Arm arm[NARM_FRAMES]; 
{
  if (arm[1].theta < (-M_PI))
    arm[1].theta += 2.0 * M_PI;
  if (arm[1].theta > M_PI)
    arm[1].theta -= 2.0 * M_PI;
  if (arm[2].theta < (-M_PI))
    arm[2].theta += 2.0 * M_PI;
  if (arm[2].theta > M_PI)
    arm[2].theta -= 2.0 * M_PI;
}

control_obj(obj, time)
Obj * obj;
double time; 
{
  // if you want to settle the object down in the course of an experiment
  // set the reset flag to TRUE
  if (reset_flag) {
    obj->mass = 0.20;
    obj->position[0] = 0.15;
    obj->position[1] = 0.30;
    obj->velocity[0] = 0.00;
    obj->velocity[1] = 0.00;
    obj->extForce[0] = obj->extForce[1] = 0.0;
    reset_flag = 0;
  }
}
/* ============================================== */


