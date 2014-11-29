/*************************************************************************/
/* File:        project6.c                                               */
/* Description: User project #6                                          */
/* Date:        01-29-2011                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"


void project6_control(roger, time)
Robot* roger;
double time;
{
printf("inside PONG!!! \n");
}

void project6_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project6_enter_params()
{
  printf("Project 6 enter_params called. \n");
}

/* do not alter */
//function called when the 'visualize' button on the gui is pressed
void project6_visualize(roger)
Robot* roger;
{ }
