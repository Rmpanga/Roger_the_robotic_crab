 /**************************************************************************/
/* File:        modes.h                                                    */
/* Description: everything for input and control modes of the simulator    */
/* Author:      Rod Grupen                                                 */
/* Date:        11-1-2009                                                  */
/**************************************************************************/
#ifndef MODES_H
#define MODES_H

//Mouse buttons
#define LEFT_BUTTON Button1
#define MIDDLE_BUTTON Button2
#define RIGHT_BUTTON Button3

/**************************************************************************/
// Modes
enum input_modes {
	JOINT_ANGLE_INPUT = 0,
	BASE_GOAL_INPUT,
	ARM_GOAL_INPUT,
	BALL_INPUT,
	MAP_INPUT,
	//	PROJECT_INPUT,
	N_INPUT_MODES  //number of modes
};

enum control_modes {
	CHASE_PUNCH = 0,
	PONG,
	N_CONTROL_MODES //number of modes
};



#endif
