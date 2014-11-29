/****************************************************************/
/** xrobot.c: simulates and renders mobile manipulator         **/
/**           version of Roger-the-Crab                        **/
/** author:   Grupen                                           **/
/** date:     April, 2010                                      **/
/****************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "Xkw/Xkw.h"

#include "include/Roger.h"
#include "include/simulate.h"
#include "include/control.h"
#include "include/modes.h"

// simulator data structures that comprise Roger
Base mobile_base_1;
Base mobile_base_home_1;
Eye eyes_1[NEYES];
Eye eyes_home_1[NEYES];
Arm arms_1[NARMS][NARM_FRAMES];
Arm arms_home_1[NARMS][NARM_FRAMES];
Robot Roger_1;// the user application interface data structure

// simulator data structures that comprise Roger
Base mobile_base_2;
Base mobile_base_home_2;
Eye eyes_2[NEYES];
Eye eyes_home_2[NEYES];
Arm arms_2[NARMS][NARM_FRAMES];
Arm arms_home_2[NARMS][NARM_FRAMES];
Robot Roger_2;// the user application interface data structure

Obj object;

SimColor world_colors[113];

History history_1[MAX_HISTORY]; // a diagnostic tool to illustrate a trajectory
int history_ptr_1=0;

History history_2[MAX_HISTORY]; // a diagnostic tool to illustrate a trajectory
int history_ptr_2=0;

//extern init_control_flag;
int init_control_flag = TRUE;

// global Boolean reset flag - eliminates user defined obstacles and goals
// reconfigures Roger and single object to starting configuration
int reset;

// global simulator clock
double simtime = 0.0;

void x_canvas_proc(), x_start_proc(), x_params_proc(), x_input_mode_1_proc(),
  x_control_mode_1_proc(), x_input_mode_2_proc(), x_control_mode_2_proc(), x_pause_proc();
void x_quit_proc(), x_timer_proc(), x_visualize_proc();
void x_sock_1_proc(), x_sock_2_proc();
void initialize_control_mode();
void SIMmatXvec(), SIMmatXmat(), SIMinv_transform(), SIMcopy_matrix();
void simulate_object(), simulate_arm(), simulate_base(), simulate_eyes();
void SocketCommunicate(), SocketInit();
void usleep();

Display          *display;
Window           window;
Pixmap           pixmap;
XtAppContext     app_con;
GC               gc;
int              screen;
Widget           canvas_w, input_mode_1_w, control_mode_1_w, params_w,
  start_w, popup_w = NULL;
Widget			 input_mode_2_w, control_mode_2_w, sock_1_w, sock_2_w, pause_w=NULL;
Widget           intensity_w, stream_w;
XtIntervalId     timer = 0;
int              width = WIDTH, height = HEIGHT, depth;
unsigned long    foreground, background;

int zoom = ZOOM_SCALE; 

int numRoger = 0;
int socknew1, socknew2;

int init_flag = 2;  //TRUE
double possessionTime_1 = 0;
double possessionTime_2 = 0;
double continuous_possesionTime_1 = 0;
double continuous_possesionTime_2 = 0;
int goal_1 = 0;
int goal_2 = 0;
int touchWall_1 = 0;
int touchWall_2 = 0;

void x_init_colors()
{
  int i;
	
  //  printf("initializing grey scale colors..."); fflush(stdout);
  for (i = 0; i <= 100; i++) { // 0 => black; 100 => white
    sprintf(world_colors[i].name, "grey%d", i);
    world_colors[i].display_color =
      XkwParseColor(display, world_colors[i].name);
    world_colors[i].red = (int) i*2.55;
    world_colors[i].green = (int) i*2.55;
    world_colors[i].blue = (int) i*2.55;
  }
  strcpy(world_colors[101].name, "dark red");
  world_colors[101].display_color = XkwParseColor(display, "dark red");
  world_colors[101].red = 139;
  world_colors[101].green = 0;
  world_colors[101].blue = 0;
	
  strcpy(world_colors[102].name, "red");
  world_colors[102].display_color = XkwParseColor(display, "red");
  world_colors[102].red = 255;
  world_colors[102].green = 0;
  world_colors[102].blue = 0;
	
  strcpy(world_colors[103].name, "hot pink");
  world_colors[103].display_color = XkwParseColor(display, "hot pink");
  world_colors[103].red = 255;
  world_colors[103].green = 105;
  world_colors[103].blue = 180;
	
  strcpy(world_colors[104].name, "navy");
  world_colors[104].display_color = XkwParseColor(display, "navy");
  world_colors[104].red = 65;
  world_colors[104].green = 105;
  world_colors[104].blue = 225;
  
  strcpy(world_colors[105].name, "blue");
  world_colors[105].display_color = XkwParseColor(display, "blue");
  world_colors[105].red = 0;
  world_colors[105].green = 0;
  world_colors[105].blue = 255;
	
  strcpy(world_colors[106].name, "light sky blue");
  world_colors[106].display_color = XkwParseColor(display, "light sky blue");
  world_colors[106].red = 250;
  world_colors[106].green = 128;
  world_colors[106].blue = 114;
	
  strcpy(world_colors[107].name, "dark green");
  world_colors[107].display_color = XkwParseColor(display, "dark green");
  world_colors[107].red = 244;
  world_colors[107].green = 164;
  world_colors[107].blue = 96;
	
  strcpy(world_colors[108].name, "green");
  world_colors[108].display_color = XkwParseColor(display, "green");
  world_colors[108].red = 0;
  world_colors[108].green = 255;
  world_colors[108].blue = 0;
	
  strcpy(world_colors[109].name, "light green");
  world_colors[109].display_color = XkwParseColor(display, "light green");
  world_colors[109].red = 46;
  world_colors[109].green = 139;
  world_colors[109].blue = 87;
	
  strcpy(world_colors[110].name, "gold");
  world_colors[110].display_color = XkwParseColor(display, "gold");
  world_colors[110].red = 160;
  world_colors[110].green = 82;
  world_colors[110].blue = 45;
	
  strcpy(world_colors[111].name, "yellow");
  world_colors[111].display_color = XkwParseColor(display, "yellow");
  world_colors[111].red = 255;
  world_colors[111].green = 255;
  world_colors[111].blue = 0;
	
  strcpy(world_colors[112].name, "light goldenrod");
  world_colors[112].display_color = XkwParseColor(display, "light goldenrod");
  world_colors[112].red = 192;
  world_colors[112].green = 192;
  world_colors[112].blue = 192;
}

void motor_model(roger)
Robot* roger;
{
	// model the shoulder motors
	int i;
	double shoulder_max_torque;
	for(i=0; i<2; i++){
		// check that torque is within overall limit
		if(roger->arm_torque[i][0]>SHOULDER_TS)
		{
			roger->arm_torque[i][0]=SHOULDER_TS;
		}
		else if(roger->arm_torque[i][0]<-1*SHOULDER_TS)
		{
			roger->arm_torque[i][0]=-1*SHOULDER_TS;
		}

		// check that velocity limited torque is within computed bounds
		shoulder_max_torque = -SHOULDER_TS/SHOULDER_WS*roger->arm_theta_dot[i][0];

		if(roger->arm_theta_dot[i][0] > 0)
		{
			// spinning positive
			shoulder_max_torque+=SHOULDER_TS;
			if(roger->arm_torque[i][0]>shoulder_max_torque)
			{
				roger->arm_torque[i][0]=shoulder_max_torque;
			}
		}
		else if(roger->arm_theta_dot[i][0] < 0)
		{
			// spinning negative
			shoulder_max_torque -=SHOULDER_TS;
			if(roger->arm_torque[i][0]<shoulder_max_torque)
			{
				roger->arm_torque[i][0]=shoulder_max_torque;
			}
		}
	}
	// model the elbows
	double elbow_max_torque;
	for(i=0; i<2; i++){
		// check that torques are within overall limits
		if(roger->arm_torque[i][1]>ELBOW_TS)
		{
			roger->arm_torque[i][1]=ELBOW_TS;
		}
		else if(roger->arm_torque[i][1]<-1*ELBOW_TS)
		{
			roger->arm_torque[i][1]=-1*ELBOW_TS;
		}
		// apply torque limits based on speed
		elbow_max_torque = -ELBOW_TS/ELBOW_WS*roger->arm_theta_dot[i][1];
		if(roger->arm_theta_dot[i][1] > 0)
		{
			// spinning positive
			elbow_max_torque +=ELBOW_TS;
			if(roger->arm_torque[i][1]>elbow_max_torque)
			{
				roger->arm_torque[i][1]=elbow_max_torque;
			}
		}
		else if(roger->arm_theta_dot[i][1] < 0)
		{
			// spinning negative
			elbow_max_torque -=ELBOW_TS;
			if(roger->arm_torque[i][1]<elbow_max_torque)
			{
				roger->arm_torque[i][1]=elbow_max_torque;
			}
		}

	}

	// model the eyes
	double eye_max_torque;
	for(i=0; i<2; i++){
		// check that torques are within overall limits
		if(roger->eye_torque[i]>EYE_TS)
		{
			roger->eye_torque[i]=EYE_TS;
		}
		else if(roger->eye_torque[i]<-1*EYE_TS)
		{
			roger->eye_torque[i]=-1*EYE_TS;
		}
		// apply velocity dependent torque limits
		eye_max_torque = -EYE_TS/EYE_WS*roger->eye_theta_dot[i];

		if(roger->eye_theta_dot[i] > 0)
		{
			// spinning positive
			eye_max_torque +=EYE_TS;
			if(roger->eye_torque[i]>eye_max_torque)
			{
				roger->eye_torque[i]=eye_max_torque;
			}
		}
		else if(roger->eye_theta_dot[i] < 0)
		{
			// spinning negative
			eye_max_torque -=EYE_TS;
			if(roger->eye_torque[i]<eye_max_torque)
			{
				roger->eye_torque[i]=eye_max_torque;
			}
		}
	}


	// model the wheels
	double wheel_max_torque;
	for(i=0; i<2; i++){
		// check overall torque limit
		if(roger->wheel_torque[i]>WHEEL_TS)
		{
			roger->wheel_torque[i]=WHEEL_TS;
		}
		else if(roger->wheel_torque[i]<-1*WHEEL_TS)
		{
			roger->wheel_torque[i]=-1*WHEEL_TS;
		}

		// apply velocity based torque limits
		wheel_max_torque = -WHEEL_TS/WHEEL_WS*roger->wheel_theta_dot[i];
		if(roger->wheel_theta_dot[i] > 0)
		{
			// spinning positive
			wheel_max_torque +=WHEEL_TS;
			if(roger->wheel_torque[i]>wheel_max_torque)
			{
				roger->wheel_torque[i]=wheel_max_torque;
			}
		}
		else if(roger->wheel_theta_dot[i] < 0)
		{
			// spinning negative
			wheel_max_torque -=WHEEL_TS;
			if(roger->wheel_torque[i]<wheel_max_torque)
			{
				roger->wheel_torque[i]=wheel_max_torque;
			}
		}

	}

}


// #ifndef MAX_RAND
// #define MAX_RAND   2147483647.0
// #endif

#define R_GOAL 0.35

void initialize_random_object()
{
  // ORIGINAL (SINGLE) OBJECT
  object.mass = 0.2;
  // srand((unsigned int)(time(NULL)));
  double N1 = ((double)(rand() % 1000 + 1))/1000.0;
  double N2 = ((double)(rand() % 1000 + 1))/1000.0;

  double dx = MAX_X - MIN_X - 2.0*XDELTA - 2.0*R_OBJ;
  double dy = MAX_Y - MIN_Y - 2.0*YDELTA - 2.0*R_OBJ - R_GOAL;

  object.position[X] = (MIN_X + XDELTA + R_OBJ) + N1*dx;
  object.position[Y] = (MIN_Y + YDELTA + R_OBJ + R_GOAL) + N2*dy;

  // printf("%lf %lf %lf %lf\n",
  //        N1, N2, object.position[X], object.position[Y]);

  object.velocity[X] = object.velocity[Y] = 0.0;
  object.extForce[X] = object.extForce[Y] = 0.0;
}

void initialize_pong_object()
{
	object.position[X] =0;
	object.position[Y] = 0;

	// ORIGINAL (SINGLE) OBJECT
	//  object.mass = 0.2;
	
	srand((unsigned int)(time(NULL)));
	double N1 = ((double)(rand() % 1000 + 1))/1000.0;
	//srand((unsigned int)(time(NULL)));
	double N2 = ((double)(rand() % 1000 + 1))/1000.0;

	//  double dx = MAX_X - MIN_X - 2.0*XDELTA - 2.0*R_OBJ;
	//  double dy = MAX_Y - MIN_Y - 2.0*YDELTA - 2.0*R_OBJ - R_GOAL;

	//object.position[X] = object.position[X];// WIDTH/2; //(MIN_X + XDELTA + R_OBJ) + N1*dx;
	//object.position[Y] = object.position[Y]; //HEIGHT/2; //(MIN_Y + YDELTA + R_OBJ + R_GOAL) + N2*dy;

	// printf("%lf %lf %lf %lf\n",
	//        N1, N2, object.position[X], object.position[Y]);

	double vx = (N1 - 0.5) * 20;
	double vy = (N2 - 0.5) * 20;
	object.velocity[X] = vx;
	object.velocity[Y] = vy;
	//printf("%lf %lf \n",
	//	   object.velocity[X], object.velocity[Y]);
	object.extForce[X] = object.extForce[Y] = 0.0;
}


void place_object(x,y)
double x,y;
{
  object.mass = 0.2;
  object.position[X] = x;
  object.position[Y] = y;
  object.velocity[X] = object.velocity[Y] = 0.0;
  object.extForce[X] = object.extForce[Y] = 0.0;
}

void x_params_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
	/*
  switch (Roger.control_mode) {
  case PROJECT3: project3_enter_params(); break;
  case PROJECT4: project4_enter_params(); break;
  case PROJECT5: project5_enter_params(); break;
  case PROJECT6: project6_enter_params(); break;
  default:
    //called general enter_params() in UserIO.c
    enter_params();
  }
	 */
}

int change_input_mode_1()
{
  static int input_mode;

  input_mode = (input_mode + 1) % N_INPUT_MODES;
  //init_input_flag = TRUE;
  return (input_mode);
}

int change_input_mode_2()
{
	static int input_mode;
	
	input_mode = (input_mode + 1) % N_INPUT_MODES;
	//init_input_flag = TRUE;
	return (input_mode);
}


void x_input_mode_1_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Roger_1.input_mode = change_input_mode_1();

  switch (Roger_1.input_mode) {
  case JOINT_ANGLE_INPUT:
    XkwSetWidgetLabel(input_mode_1_w, "Input 1: Joint angles"); break;
  case BASE_GOAL_INPUT:
    XkwSetWidgetLabel(input_mode_1_w, "Input 1: Base goal"); break;
  case ARM_GOAL_INPUT:
    XkwSetWidgetLabel(input_mode_1_w, "Input 1: Arm goals"); break;
  case BALL_INPUT:
    XkwSetWidgetLabel(input_mode_1_w, "Input 1: Ball position"); break;
  case MAP_INPUT:
    XkwSetWidgetLabel(input_mode_1_w, "Input 1: Map Editor"); break;
  default: break;
  }
}

void x_input_mode_2_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
	Roger_2.input_mode = change_input_mode_2();

	switch (Roger_2.input_mode) {
		case JOINT_ANGLE_INPUT:
			XkwSetWidgetLabel(input_mode_2_w, "Input 2: Joint angles"); break;
		case BASE_GOAL_INPUT:
			XkwSetWidgetLabel(input_mode_2_w, "Input 2: Base goal"); break;
		case ARM_GOAL_INPUT:
			XkwSetWidgetLabel(input_mode_2_w, "Input 2: Arm goals"); break;
		case BALL_INPUT:
			XkwSetWidgetLabel(input_mode_2_w, "Input 2: Ball position"); break;
		case MAP_INPUT:
			XkwSetWidgetLabel(input_mode_2_w, "Input 2: Map Editor"); break;
		default: break;
	}
}

int change_control_mode_1()
{
  static int control_mode;

  control_mode = (control_mode + 1) % N_CONTROL_MODES;
  init_control_flag = TRUE;
  return (control_mode);
}

int change_control_mode_2()
{
	static int control_mode;
	
	control_mode = (control_mode + 1) % N_CONTROL_MODES;
	init_control_flag = TRUE;
	return (control_mode);
}

void x_control_mode_1_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  Roger_1.control_mode = change_control_mode_1();

  switch (Roger_1.control_mode) {
	  case CHASE_PUNCH:
		  XkwSetWidgetLabel(control_mode_1_w, "Control 1: ChasePunch"); break;
	  case PONG:
		  XkwSetWidgetLabel(control_mode_1_w, "Control 1: Pong"); break;
  default: break;
  }
  //call init here makes it independent of timer running
  //initialize_control_mode(&Roger_1);
}

void x_control_mode_2_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
	Roger_2.control_mode = change_control_mode_2();

	switch (Roger_2.control_mode) {
		case CHASE_PUNCH:
			XkwSetWidgetLabel(control_mode_2_w, "Control 2: ChasePunch"); break;
		case PONG:
			XkwSetWidgetLabel(control_mode_2_w, "Control 2: Pong"); break;
		default: break;
	}
	//call init here makes it independent of timer running
	//initialize_control_mode(&Roger_2);
}

void x_quit_proc(w, client_data, call_data)
Widget w;
XtPointer call_data, client_data;
{
  XFreePixmap(display, pixmap);
  XFreeGC(display, gc);
  XtDestroyApplicationContext(app_con);
  exit(0);
}

void x_canvas_proc(w, client_data, call_data)
Widget w;
XtPointer call_data, client_data;
{
  XEvent *event = ((XEvent *) call_data);
  char text[10];
  KeySym key;
  double x, y, theta1, theta2;
  int i, j, xbin, ybin;
  int c;
  static Dimension nwidth = WIDTH, nheight = HEIGHT;
  void x_expose(), x_clear();
	
  switch (event->type) {
  case ConfigureNotify:
    nwidth = event->xconfigure.width;
    nheight = event->xconfigure.height;
    break;
  case Expose:
    if (nwidth == width && nheight == height)
      x_expose();
    else {
      /* width = nwidth; height = nheight; */
      x_expose();
    }
    break;

  case ButtonPress:
		  if (event->xbutton.button == Button1) {
			  printf("Controlling 1st Roger\n");
			  Roger_1.button_reference[X] = D2WX(zoom,event->xbutton.x);
			  Roger_1.button_reference[Y] = D2WY(zoom,event->xbutton.y);
			  Roger_1.button_event = event->xbutton.button;
			  printf("Roger 1 -> Position : %lf,%lf--%d,%d\n",Roger_1.button_reference[X], Roger_1.button_reference[Y],event->xbutton.x,event->xbutton.y);
		  }
		  else if (event->xbutton.button == Button3) {
			  printf("Controlling 2nd Roger\n");
			  Roger_2.button_reference[X] = D2WX(zoom,event->xbutton.x);
			  Roger_2.button_reference[Y] = D2WY(zoom,event->xbutton.y);
			  Roger_2.button_event = event->xbutton.button;
			  printf("Roger 2 -> Position : %lf,%lf--%d,%d\n",Roger_2.button_reference[X], Roger_2.button_reference[Y],event->xbutton.x,event->xbutton.y);
		  }
		  //if (event->xbutton.button == LEFT_BUTTON) { }
    break;

  case ButtonRelease:
    break;

  case KeyPress:
    c = XLookupString((XKeyEvent *) event, text, 10, &key, 0);
    if (c == 1)
      switch (text[0]) {
      case 'h':
	break;
      case 'c':
	x_clear();
	x_expose();
	break;
      case 'q':
	x_quit_proc(w, client_data, call_data);
      }
  }
}

void x_draw_line(color, start_x, start_y, end_x, end_y)
int color;
double start_x, start_y, end_x, end_y;
{
  XSetForeground(display, gc, world_colors[color].display_color);
  XDrawLine(display, pixmap, gc,  W2DX(zoom, start_x), W2DY(zoom, start_y),
	    W2DX(zoom, end_x), W2DY(zoom, end_y));
}

/*
void x_draw_circle(color, center_x, center_y, radius, fill)
int color, fill;
double center_x, center_y, radius;
{
  XSetForeground(display, gc, world_colors[color].display_color);
  draw_circle(W2DX(zoom,center_x),W2DY(zoom,center_y),W2DR(zoom,radius),fill);
}
*/

void x_expose()
{
  XCopyArea(display, pixmap, window, gc, 0, 0, width, height, 0, 0);
}

void x_clear()
{
  XSetForeground(display, gc, background);
  XFillRectangle(display, pixmap, gc, 0, 0, width, height);
}

#define STEP         0.01
#define STREAM_SPACE 0
/*
void x_visualize_proc(w,client_data,call_data)
{
  static int draw_visual = FALSE;
  void draw_roger(), draw_object(), draw_frames(), draw_history();

  if (draw_visual == TRUE) {
    draw_visual = FALSE;
    XkwSetWidgetLabel(start_w, "Stop");
    timer = XtAppAddTimeOut(app_con, TIMER_UPDATE, x_timer_proc,
			    (XtPointer) NULL);
    return;
  }
  else {
    if (timer) {
      XkwSetWidgetLabel(start_w, "Start");
      XtRemoveTimeOut(timer);
    }
    timer = 0;
    draw_visual = TRUE;

    switch (Roger.control_mode) {
    case PROJECT3:
      project3_visualize(&Roger);
      break;
    case PROJECT4:
      project4_visualize(&Roger);
      break;
    case PROJECT5:
      project5_visualize(&Roger);
      break;
    case PROJECT6:
      project6_visualize(&Roger);
      break;
    default:
      break;
    }

    draw_object(object);
    draw_roger(mobile_base, arms, eyes);
    draw_frames();

    if (HISTORY) {
      draw_history();
    }
    x_expose();
  }
}
*/


void mark_used(ii,jj,aux)
int ii, jj;
int aux[NBINS][NBINS];
{
  int j,k;
  double dist;

  for (j=-STREAM_SPACE; j<=STREAM_SPACE; ++j) {
    for (k=-STREAM_SPACE; k<=STREAM_SPACE; ++k) {
      dist = sqrt(SQR((double)j) + SQR((double)k));
      if ((dist < (2.0*STREAM_SPACE + 1.0)) &&
	  ((ii+j) >= 0) && ((ii+j) < NBINS) &&
	  ((jj+k) >= 0) && ((jj+k) < NBINS))
	aux[ii+j][jj+k] = TRUE;
    }
  }
}

void game_over_detection()
{
	if (goal_1 == 11 || goal_2 == 11) {
		//XtPointer client_data, call_data;
		//x_params_proc(start_w, client_data, call_data);
	    XkwSetWidgetLabel(start_w, "New Game");
	    XtRemoveTimeOut(timer);
	    timer = 0;
	}
}

void write_interface(reset)
int reset;
{
  int i,j;

  // ====== roger 1
  // pass in afferents (read only)
  Roger_1.eye_theta[0] = eyes_1[0].theta;
  Roger_1.eye_theta_dot[0] = eyes_1[0].theta_dot;
  Roger_1.eye_theta[1] = eyes_1[1].theta;
  Roger_1.eye_theta_dot[1] = eyes_1[1].theta_dot;
  for (i=0;i<NPIXELS;++i) {
      Roger_1.image[LEFT][i][RED_CHANNEL] =
	world_colors[eyes_1[LEFT].image[i]].red;
      Roger_1.image[LEFT][i][GREEN_CHANNEL]=
	world_colors[eyes_1[LEFT].image[i]].green;
      Roger_1.image[LEFT][i][BLUE_CHANNEL] =
	world_colors[eyes_1[LEFT].image[i]].blue;
      Roger_1.image[RIGHT][i][RED_CHANNEL] =
	world_colors[eyes_1[RIGHT].image[i]].red;
      Roger_1.image[RIGHT][i][GREEN_CHANNEL] =
	world_colors[eyes_1[RIGHT].image[i]].green;
      Roger_1.image[RIGHT][i][BLUE_CHANNEL] =
	world_colors[eyes_1[RIGHT].image[i]].blue;
  }
  Roger_1.arm_theta[0][0] = arms_1[0][1].theta;
  Roger_1.arm_theta[0][1] = arms_1[0][2].theta;
  Roger_1.arm_theta[1][0] = arms_1[1][1].theta;
  Roger_1.arm_theta[1][1] = arms_1[1][2].theta;
  Roger_1.arm_theta_dot[0][0] = arms_1[0][1].theta_dot;
  Roger_1.arm_theta_dot[0][1] = arms_1[0][2].theta_dot;
  Roger_1.arm_theta_dot[1][0] = arms_1[1][1].theta_dot;
  Roger_1.arm_theta_dot[1][1] = arms_1[1][2].theta_dot;
  Roger_1.ext_force[0][X] = arms_1[0][NARM_FRAMES - 1].extForce[X];
  Roger_1.ext_force[0][Y] = arms_1[0][NARM_FRAMES - 1].extForce[Y];
  Roger_1.ext_force[1][X] = arms_1[1][NARM_FRAMES - 1].extForce[X];
  Roger_1.ext_force[1][Y] = arms_1[1][NARM_FRAMES - 1].extForce[Y];

  Roger_1.base_position[0] = mobile_base_1.x;
  Roger_1.base_position[1] = mobile_base_1.y;
  Roger_1.base_position[2] = mobile_base_1.theta;
  Roger_1.base_velocity[0] = mobile_base_1.x_dot;
  Roger_1.base_velocity[1] = mobile_base_1.y_dot;
  Roger_1.base_velocity[2] = mobile_base_1.theta_dot;

  // zero efferents (write only)
  Roger_1.eye_torque[0] = Roger_1.eye_torque[1] = 0.0;
  Roger_1.arm_torque[0][0]=Roger_1.arm_torque[0][1]=Roger_1.arm_torque[1][0]=
    Roger_1.arm_torque[1][1] = 0.0;
  Roger_1.wheel_torque[0] = Roger_1.wheel_torque[1] = 0.0;
  Roger_1.simtime = simtime;
  Roger_1.opponent_base_position[0] = mobile_base_2.x;
  Roger_1.opponent_base_position[1] = mobile_base_2.y;
  Roger_1.opponent_base_position[2] = mobile_base_2.theta;
  Roger_1.opponent_base_velocity[0] = mobile_base_2.x_dot;
  Roger_1.opponent_base_velocity[1] = mobile_base_2.y_dot;
  Roger_1.opponent_base_velocity[2] = mobile_base_2.theta_dot;


	// ====== roger 1
	// pass in afferents (read only)
	Roger_2.eye_theta[0] = eyes_2[0].theta;
	Roger_2.eye_theta_dot[0] = eyes_2[0].theta_dot;
	Roger_2.eye_theta[1] = eyes_2[1].theta;
	Roger_2.eye_theta_dot[1] = eyes_2[1].theta_dot;
	for (i=0;i<NPIXELS;++i) {
		Roger_2.image[LEFT][i][RED_CHANNEL] =
		world_colors[eyes_2[LEFT].image[i]].red;
		Roger_2.image[LEFT][i][GREEN_CHANNEL]=
		world_colors[eyes_2[LEFT].image[i]].green;
		Roger_2.image[LEFT][i][BLUE_CHANNEL] =
		world_colors[eyes_2[LEFT].image[i]].blue;
		Roger_2.image[RIGHT][i][RED_CHANNEL] =
		world_colors[eyes_2[RIGHT].image[i]].red;
		Roger_2.image[RIGHT][i][GREEN_CHANNEL] =
		world_colors[eyes_2[RIGHT].image[i]].green;
		Roger_2.image[RIGHT][i][BLUE_CHANNEL] =
		world_colors[eyes_2[RIGHT].image[i]].blue;
	}
	Roger_2.arm_theta[0][0] = arms_2[0][1].theta;
	Roger_2.arm_theta[0][1] = arms_2[0][2].theta;
	Roger_2.arm_theta[1][0] = arms_2[1][1].theta;
	Roger_2.arm_theta[1][1] = arms_2[1][2].theta;
	Roger_2.arm_theta_dot[0][0] = arms_2[0][1].theta_dot;
	Roger_2.arm_theta_dot[0][1] = arms_2[0][2].theta_dot;
	Roger_2.arm_theta_dot[1][0] = arms_2[1][1].theta_dot;
	Roger_2.arm_theta_dot[1][1] = arms_2[1][2].theta_dot;
	Roger_2.ext_force[0][X] = arms_2[0][NARM_FRAMES - 1].extForce[X];
	Roger_2.ext_force[0][Y] = arms_2[0][NARM_FRAMES - 1].extForce[Y];
	Roger_2.ext_force[1][X] = arms_2[1][NARM_FRAMES - 1].extForce[X];
	Roger_2.ext_force[1][Y] = arms_2[1][NARM_FRAMES - 1].extForce[Y];

	Roger_2.base_position[0] = mobile_base_2.x;
	Roger_2.base_position[1] = mobile_base_2.y;
	Roger_2.base_position[2] = mobile_base_2.theta;
	Roger_2.base_velocity[0] = mobile_base_2.x_dot;
	Roger_2.base_velocity[1] = mobile_base_2.y_dot;
	Roger_2.base_velocity[2] = mobile_base_2.theta_dot;

	// zero efferents (write only)
	Roger_2.eye_torque[0] = Roger_2.eye_torque[1] = 0.0;
	Roger_2.arm_torque[0][0]=Roger_2.arm_torque[0][1]=Roger_2.arm_torque[1][0]=
    Roger_2.arm_torque[1][1] = 0.0;
	Roger_2.wheel_torque[0] = Roger_2.wheel_torque[1] = 0.0;
	Roger_2.simtime = simtime;
	Roger_2.opponent_base_position[0] = mobile_base_1.x;
	Roger_2.opponent_base_position[1] = mobile_base_1.y;
	Roger_2.opponent_base_position[2] = mobile_base_1.theta;
	Roger_2.opponent_base_velocity[0] = mobile_base_1.x_dot;
	Roger_2.opponent_base_velocity[1] = mobile_base_1.y_dot;
	Roger_2.opponent_base_velocity[2] = mobile_base_1.theta_dot;

}

void read_interface()
{
	// pass back torques (write only)
	arms_1[0][1].torque = Roger_1.arm_torque[0][0];
	arms_1[1][1].torque = Roger_1.arm_torque[1][0];
	arms_1[0][2].torque = Roger_1.arm_torque[0][1];
	arms_1[1][2].torque = Roger_1.arm_torque[1][1];

	eyes_1[0].torque = Roger_1.eye_torque[0];
	eyes_1[1].torque = Roger_1.eye_torque[1];

	mobile_base_1.wheel_torque[0] = Roger_1.wheel_torque[0];
	mobile_base_1.wheel_torque[1] = Roger_1.wheel_torque[1];

	// pass back torques (write only)
	arms_2[0][1].torque = Roger_2.arm_torque[0][0];
	arms_2[1][1].torque = Roger_2.arm_torque[1][0];
	arms_2[0][2].torque = Roger_2.arm_torque[0][1];
	arms_2[1][2].torque = Roger_2.arm_torque[1][1];

	eyes_2[0].torque = Roger_2.eye_torque[0];
	eyes_2[1].torque = Roger_2.eye_torque[1];

	mobile_base_2.wheel_torque[0] = Roger_2.wheel_torque[0];
	mobile_base_2.wheel_torque[1] = Roger_2.wheel_torque[1];

}

/* forward kinematics in base frame **************************************/
void sim_fwd_kinematics(arm_id, theta1, theta2, x, y)
int arm_id;
double theta1, theta2;
double *x, *y;
{
  *x = LARM_1 * cos(theta1) + LARM_2 * cos(theta1 + theta2);
  *y = LARM_1 * sin(theta1) + LARM_2 * sin(theta1 + theta2);

  if (arm_id == LEFT) *y += ARM_OFFSET;
  else *y -= ARM_OFFSET;
}

void sim_arm_Jacobian(theta1,theta2, Jacobian)
double theta1,theta2;
double Jacobian[2][2];
{
  Jacobian[0][0] = -LARM_1*sin(theta1) - LARM_2*sin(theta1 + theta2);
  Jacobian[0][1] = -LARM_2*sin(theta1 + theta2);
  Jacobian[1][0] =  LARM_1*cos(theta1) + LARM_2*cos(theta1 + theta2);
  Jacobian[1][1] =  LARM_2*cos(theta1 + theta2);
}

#define NBODY 4 // single ball, two hands, roger's body

void compute_external_forces_1(base, arms, obj)
Base * base;
Arm arms[NARMS][NARM_FRAMES];
Obj * obj;
{
  int i, j, row, col;

  double x, y, p_b[4], p_w[4], v_b[4], v_w[4];
  double r[NBODY][2], mag, v[NBODY][2], R[NBODY], J[2][2], dr[2];
  double vi_proj, vj_proj, force;
  double F[NBODY][2], dij, sum[2];

  // initialize forces on dynamic bodies: base, hands, object
  base->extForce[X] = base->extForce[Y] = 0.0;
  obj->extForce[X] = obj->extForce[Y] = 0.0;
  for (i = 0; i < NARMS; ++i) {
    arms[i][NARM_FRAMES - 1].extForce[X] = 0.0;
    arms[i][NARM_FRAMES - 1].extForce[Y] = 0.0;
  }
	touchWall_1 = 0;

  // define the position and radius of the NBODY dynamic bodies
  // BASE #0
  r[0][X] = base->x;
  r[0][Y] = base->y;
  v[0][X] = base->x_dot;
  v[0][Y] = base->y_dot;
  R[0] = R_BASE;

  // ARM #1
  sim_fwd_kinematics(LEFT, arms[LEFT][1].theta, arms[LEFT][2].theta, &x, &y);
  p_b[0] = x; p_b[1] = y; p_b[2] = 0.0; p_b[3] = 1.0;
  SIMmatXvec(base->wTb, p_b, p_w);
  r[1][X] = p_w[X];
  r[1][Y] = p_w[Y];
	
  // hand velocity relative to base written in base coordinates
  sim_arm_Jacobian(arms[LEFT][1].theta, arms[LEFT][2].theta, J);
  v_b[X] = J[0][0]*arms[LEFT][1].theta_dot + J[0][1]*arms[LEFT][2].theta_dot -
    R_BASE*base->theta_dot;
  v_b[Y] = J[1][0]*arms[LEFT][1].theta_dot + J[1][1]*arms[LEFT][2].theta_dot;
  v_b[2] = 0.0;
  v_b[3] = 0.0; // homogeneous vector

  SIMmatXvec(base->wTb, v_b, v_w);
  v[1][X] = base->x_dot + v_w[X];
  v[1][Y] = base->y_dot + v_w[Y];
  R[1] = R_TACTILE;

  // ARM #2
  sim_fwd_kinematics(RIGHT, arms[RIGHT][1].theta, arms[RIGHT][2].theta, &x,&y);
  p_b[0] = x; p_b[1] = y; p_b[2] = 0.0; p_b[3] = 1.0;
  SIMmatXvec(base->wTb, p_b, p_w);
  r[2][X] = p_w[X];
  r[2][Y] = p_w[Y];

  // hand velocity relative to base written in base coordinates
  sim_arm_Jacobian(arms[RIGHT][1].theta, arms[RIGHT][2].theta, J);
  v_b[X] = J[0][0]*arms[RIGHT][1].theta_dot + J[0][1]*arms[RIGHT][2].theta_dot
    + R_BASE*base->theta_dot;
  v_b[Y] = J[1][0]*arms[RIGHT][1].theta_dot + J[1][1]*arms[RIGHT][2].theta_dot;
  v_b[2] = 0.0;
  v_b[3] = 0.0; // homogeneous vector

  SIMmatXvec(base->wTb, v_b, v_w);
  v[2][X] = base->x_dot + v_w[X];
  v[2][Y] = base->y_dot + v_w[Y];
  R[2] = R_TACTILE;

  // OBJ #3
  r[3][X] = obj->position[X];
  r[3][Y] = obj->position[Y];
  v[3][X] = obj->velocity[X];
  v[3][Y] = obj->velocity[Y];
  R[3] = R_OBJ;
	
  //  sum multi-body collision forces on the every body:
  //     base #0, arm #1 arm #2, object #3
  for (i = 0; i < NBODY; ++i) {
    sum[X] = sum[Y] = 0.0;
    for (j = 0; j < NBODY; ++j) {
      if (j != i) {
	dr[X] = r[i][X] - r[j][X];
	dr[Y] = r[i][Y] - r[j][Y];
	mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));
	
	dij = MAX(0.0, (R[i]+R[j]-mag));

	if (dij > 0.0) { // body i is in compression
	  // compute dij_dot
	  vi_proj = v[i][X]*dr[X]/mag + v[i][Y]*dr[Y]/mag;
	  vj_proj = v[j][X]*dr[X]/mag + v[j][Y]*dr[Y]/mag;

	  force = K_COLLIDE*dij + B_COLLIDE*(MAX(0.0, (vi_proj - vj_proj)));
	  // force = K_COLLIDE*dij - B_COLLIDE*(vi_proj - vj_proj);
	  sum[X] += force * dr[X] / mag;
	  sum[Y] += force * dr[Y] / mag;
	}
      }
    }
    for (row=0; row<NBINS; ++row) {
      for (col=0; col<NBINS; ++col) {
	if (Roger_1.world_map.occupancy_map[row][col] == OBSTACLE) {
	  dr[X] = r[i][X] - (MIN_X + (col+0.5)*XDELTA);
	  dr[Y] = r[i][Y] - (MAX_Y - (row+0.5)*YDELTA);
	  mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));

	  dij = MAX(0.0, (R[i]+R_OBSTACLE-mag));

	  if (dij > 0.0) {
		  if (i != 3)
			  touchWall_1 = 1;
	    //	base->contact_theta = theta = atan2(ry, rx);
	    vi_proj = v[i][X]*dr[X]/mag + v[i][Y]*dr[Y]/mag;
	    force = K_COLLIDE*dij - B_COLLIDE*(vi_proj);
	    sum[X] += force * dr[X] / mag;
	    sum[Y] += force * dr[Y] / mag;
	  }
	}
      }
    }
    F[i][X] = sum[X];
    F[i][Y] = sum[Y];
  }

  // BASE
  base->extForce[X] = F[0][X];
  base->extForce[Y] = F[0][Y];

  // ARM #1
  arms[LEFT][NARM_FRAMES - 1].extForce[X] = -F[1][X];
  arms[LEFT][NARM_FRAMES - 1].extForce[Y] = -F[1][Y];

  // ARM #2
  arms[RIGHT][NARM_FRAMES - 1].extForce[X] = -F[2][X];
  arms[RIGHT][NARM_FRAMES - 1].extForce[Y] = -F[2][Y];

	// OBJ
	if (F[3][X] > MAX_FORCE)
		obj->extForce[X] = MAX_FORCE;
	else if(F[3][X] < -MAX_FORCE)
		obj->extForce[X] = -MAX_FORCE;
	else
		obj->extForce[X] = F[3][X];

	if (F[3][Y] > MAX_FORCE)
		obj->extForce[Y] = MAX_FORCE;
	else if(F[3][Y] < -MAX_FORCE)
		obj->extForce[Y] = -MAX_FORCE;
	else
		obj->extForce[Y] = F[3][Y];
  // OBJ
  //obj->extForce[X] = F[3][X];
  //obj->extForce[Y] = F[3][Y];
}

void compute_external_forces_2(base, arms, obj)
Base * base;
Arm arms[NARMS][NARM_FRAMES];
Obj * obj;
{
	int i, j, row, col;
	
	double x, y, p_b[4], p_w[4], v_b[4], v_w[4];
	double r[NBODY][2], mag, v[NBODY][2], R[NBODY], J[2][2], dr[2];
	double vi_proj, vj_proj, force;
	double F[NBODY][2], dij, sum[2];

	// initialize forces on dynamic bodies: base, hands, object
	base->extForce[X] = base->extForce[Y] = 0.0;
	obj->extForce[X] = obj->extForce[Y] = 0.0;
	for (i = 0; i < NARMS; ++i) {
		arms[i][NARM_FRAMES - 1].extForce[X] = 0.0;
		arms[i][NARM_FRAMES - 1].extForce[Y] = 0.0;
	}
	touchWall_2 = 0;

	// define the position and radius of the NBODY dynamic bodies
	// BASE #0
	r[0][X] = base->x;
	r[0][Y] = base->y;
	v[0][X] = base->x_dot;
	v[0][Y] = base->y_dot;
	R[0] = R_BASE;

	// ARM #1
	sim_fwd_kinematics(LEFT, arms[LEFT][1].theta, arms[LEFT][2].theta, &x, &y);
	p_b[0] = x; p_b[1] = y; p_b[2] = 0.0; p_b[3] = 1.0;
	SIMmatXvec(base->wTb, p_b, p_w);
	r[1][X] = p_w[X];
	r[1][Y] = p_w[Y];
	
	// hand velocity relative to base written in base coordinates
	sim_arm_Jacobian(arms[LEFT][1].theta, arms[LEFT][2].theta, J);
	v_b[X] = J[0][0]*arms[LEFT][1].theta_dot + J[0][1]*arms[LEFT][2].theta_dot -
    R_BASE*base->theta_dot;
	v_b[Y] = J[1][0]*arms[LEFT][1].theta_dot + J[1][1]*arms[LEFT][2].theta_dot;
	v_b[2] = 0.0;
	v_b[3] = 0.0; // homogeneous vector
	
	SIMmatXvec(base->wTb, v_b, v_w);
	v[1][X] = base->x_dot + v_w[X];
	v[1][Y] = base->y_dot + v_w[Y];
	R[1] = R_TACTILE;
	
	// ARM #2
	sim_fwd_kinematics(RIGHT, arms[RIGHT][1].theta, arms[RIGHT][2].theta, &x,&y);
	p_b[0] = x; p_b[1] = y; p_b[2] = 0.0; p_b[3] = 1.0;
	SIMmatXvec(base->wTb, p_b, p_w);
	r[2][X] = p_w[X];
	r[2][Y] = p_w[Y];

	// hand velocity relative to base written in base coordinates
	sim_arm_Jacobian(arms[RIGHT][1].theta, arms[RIGHT][2].theta, J);
	v_b[X] = J[0][0]*arms[RIGHT][1].theta_dot + J[0][1]*arms[RIGHT][2].theta_dot
    + R_BASE*base->theta_dot;
	v_b[Y] = J[1][0]*arms[RIGHT][1].theta_dot + J[1][1]*arms[RIGHT][2].theta_dot;
	v_b[2] = 0.0;
	v_b[3] = 0.0; // homogeneous vector

	SIMmatXvec(base->wTb, v_b, v_w);
	v[2][X] = base->x_dot + v_w[X];
	v[2][Y] = base->y_dot + v_w[Y];
	R[2] = R_TACTILE;

	// OBJ #3
	r[3][X] = -obj->position[X];
	r[3][Y] = obj->position[Y];
	v[3][X] = -obj->velocity[X];
	v[3][Y] = obj->velocity[Y];
	R[3] = R_OBJ;

	//  sum multi-body collision forces on the every body:
	//     base #0, arm #1 arm #2, object #3
	for (i = 0; i < NBODY; ++i) {
		sum[X] = sum[Y] = 0.0;
		for (j = 0; j < NBODY; ++j) {
			if (j != i) {
				dr[X] = r[i][X] - r[j][X];
				dr[Y] = r[i][Y] - r[j][Y];
				mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));

				dij = MAX(0.0, (R[i]+R[j]-mag));

				if (dij > 0.0) { // body i is in compression
					// compute dij_dot
					vi_proj = v[i][X]*dr[X]/mag + v[i][Y]*dr[Y]/mag;
					vj_proj = v[j][X]*dr[X]/mag + v[j][Y]*dr[Y]/mag;

					force = K_COLLIDE*dij + B_COLLIDE*(MAX(0.0, (vi_proj - vj_proj)));
					// force = K_COLLIDE*dij - B_COLLIDE*(vi_proj - vj_proj);
					sum[X] += force * dr[X] / mag;
					sum[Y] += force * dr[Y] / mag;
				}
			}
		}
		for (row=0; row<NBINS; ++row) {
			for (col=0; col<NBINS; ++col) {
				if (Roger_2.world_map.occupancy_map[row][col] == OBSTACLE) {
					dr[X] = r[i][X] - (MIN_X + (col+0.5)*XDELTA);
					dr[Y] = r[i][Y] - (MAX_Y - (row+0.5)*YDELTA);
					mag = sqrt(SQR(dr[X]) + SQR(dr[Y]));

					dij = MAX(0.0, (R[i]+R_OBSTACLE-mag));

					if (dij > 0.0) {
						if (i != 3)
							touchWall_2 = 1;
						//	base->contact_theta = theta = atan2(ry, rx);
						vi_proj = v[i][X]*dr[X]/mag + v[i][Y]*dr[Y]/mag;
						force = K_COLLIDE*dij - B_COLLIDE*(vi_proj);
						sum[X] += force * dr[X] / mag;
						sum[Y] += force * dr[Y] / mag;
					}
				}
			}
		}
		F[i][X] = sum[X];
		F[i][Y] = sum[Y];
	}
	
	// BASE
	base->extForce[X] = F[0][X];
	base->extForce[Y] = F[0][Y];

	// ARM #1
	arms[LEFT][NARM_FRAMES - 1].extForce[X] = -F[1][X];
	arms[LEFT][NARM_FRAMES - 1].extForce[Y] = -F[1][Y];
	
	// ARM #2
	arms[RIGHT][NARM_FRAMES - 1].extForce[X] = -F[2][X];
	arms[RIGHT][NARM_FRAMES - 1].extForce[Y] = -F[2][Y];

	// OBJ
	if (F[3][X] > MAX_FORCE)
		obj->extForce[X] = MAX_FORCE;
	else if(F[3][X] < -MAX_FORCE)
		obj->extForce[X] = -MAX_FORCE;
	else
		obj->extForce[X] = F[3][X];
	
	if (F[3][Y] > MAX_FORCE)
		obj->extForce[Y] = MAX_FORCE;
	else if(F[3][Y] < -MAX_FORCE)
		obj->extForce[Y] = -MAX_FORCE;
	else
		obj->extForce[Y] = F[3][Y];
	// OBJ
	//obj->extForce[X] = F[3][X];
	//obj->extForce[Y] = F[3][Y];
}

//void compute_reflection(Obj * obj)
//{
//  static int bump_flag = 0;
//  double gridSize = 2*MAX_X/NBINS;
//  int gx = (obj->position[X]-MIN_X)/gridSize;
//  int gy = (MAX_Y-obj->position[Y])/gridSize;
//  int x,y,xmin=0,ymin=0;
//  double dmin = 100000;
//  if (Roger.world_map.occupancy_map[gy][gx] == OBSTACLE)
//    printf("INSIDE RED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//
//  if(Roger.world_map.occupancy_map[gy][gx] == OBSTACLE) {
//    if (bump_flag == 0) {
//      int window = 40;
//      for(x=gx-window; x<gx+window;x++) {
//	for(y=gy-window;y<gy+window;y++) {
//	  if(y<0 || y>NBINS || x<0 || x>NBINS)
//	    continue;
//	  if(Roger.world_map.occupancy_map[gy][gx] != OBSTACLE)
//	    continue;
//
//	  double dist = sqrt((gx-x) * (gx-x) + (gy-y) * (gy-y));
//	  if(dist < dmin) {
//	    dmin = dist;
//	    xmin = x;
//	    ymin = y;
//	    printf("============dist =%f===========\n",dist);
//	    printf("============dmin =%f===========\n",dmin);
//	    printf("xmin=%d,ymin = %d\n",xmin,ymin);
//	  }
//	}
//      }
//      double n_alpha = atan2(gy-ymin,gx-xmin);
//      double v_theta = atan2(obj->velocity[Y],obj->velocity[X]);
//      double out_beta = 2*n_alpha - v_theta + 3.1415926;
//      double v_mag = sqrt((obj->velocity[X]) * obj->velocity[X] +
//			  obj->velocity[Y] *obj->velocity[Y]);
//      printf("============n_alpha =%f===========\n",n_alpha);
//      obj->velocity[X] = v_mag * cos(out_beta);
//      obj->velocity[Y] = v_mag * sin(out_beta);
//    }
//    bump_flag = 1;
//  }
//  if(Roger.world_map.occupancy_map[gy][gx] == FREESPACE) {
//    bump_flag = 0;
//  }
//}

#define NSAMPLES 10

// Roger is now being treated as a global variable and passed in as
// a local argument
void update_cspace_arms(roger)
Robot * roger;
{
	/*
  int i,j, arm, xbin, ybin, dl, link, collide;
  double t1, t2, x, y, dx[3], dy[3];
  double wTb[4][4], bTw[4][4], b[4], w[4];

  // MAP CARTESIAN OBSTACLES TO OBSTACLES IN ARM CSPACES
  construct_wTb(roger->base_position, wTb);

  for (arm=0; arm < NARMS; ++arm) {
    for (i=0;i<NBINS;++i) {
      t2 = T2_MAX - (i+0.5)*TDELTA;
      for (j=0;j<NBINS;++j) {
	t1 = T1_MIN + (j+0.5)*TDELTA;

	// default status
	roger->arm_map[arm].occupancy_map[i][j] = FREESPACE;
	collide=FALSE;
	if ((j <= 0) || (j >= (NBINS - 1))) {
	  Roger.arm_map[arm].occupancy_map[i][j] = OBSTACLE;
	  Roger.arm_map[arm].potential_map[i][j] = 1.0;
	}
	else if ((i <= 0) || (i >= (NBINS - 1))) {
	  Roger.arm_map[arm].occupancy_map[i][j] = OBSTACLE;
	  Roger.arm_map[arm].potential_map[i][j] = 1.0;
	}
	else {
	  // in base coordinates
	  if (arm==LEFT) {x = 0.0; y = ARM_OFFSET;}
	  else  {x = 0.0; y = -ARM_OFFSET;}
	  dx[1] = LARM_1*cos(t1)/NSAMPLES;
	  dx[2] = LARM_2*cos(t1+t2)/NSAMPLES;
	  dy[1] = LARM_1*sin(t1)/NSAMPLES;
	  dy[2] = LARM_2*sin(t1+t2)/NSAMPLES;

	  // walk the length of the manipulator in Cartesian space
	  for (link=1; ((link <= 2) && (!collide)); ++link) {
	    for (dl=0; (dl<NSAMPLES); ++dl) {
	      x += dx[link]; y += dy[link];

	      b[0]=x; b[1]=y; b[2]=0.0; b[3]=1.0;

	      SIMmatXvec(wTb, b, w);

	      xbin = (int)((w[0]-MIN_X)/XDELTA);
	      ybin = (int)((MAX_Y-w[1])/YDELTA);

	      if (roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE)
		collide = TRUE;
	    }
	  }
	  if (collide) {
	    roger->arm_map[arm].occupancy_map[i][j] = OBSTACLE;
	    roger->arm_map[arm].potential_map[i][j] = 1.0;
	  }
	}
	//map goals
	sim_fwd_kinematics(arm,t1,t2,&x,&y);
	b[0]=x; b[1]=y; b[2]=0.0; b[3]=1.0;
	SIMmatXvec(wTb, b, w);

	xbin = (int)((w[0]-MIN_X)/XDELTA);
	ybin = (int)((MAX_Y-w[1])/YDELTA);
	if (roger->world_map.occupancy_map[ybin][xbin] == GOAL) {
	  roger->arm_map[arm].occupancy_map[i][j] = GOAL;
	  roger->arm_map[arm].potential_map[i][j] = 0.0;
	}
      }
    }
  }
	 */
}

//update_cspace_base(roger)
//Robot * roger;
//{
//  int i,j, xbin, ybin, dl, link, collide;
//  double t1, t2, x, y, dx[3], dy[3];
//  double wTb[4][4], bTw[4][4], b[4], w[4];
//
//  // MAP CARTESIAN OBSTACLES TO DIRICHLET CONSTRAINTS IN CSPACE
//  // for each CSpace bin:
//  //	printf("\n Updating CSPACE for BASE");
//  //construct_wTb(roger->base_position, wTb);
//
//  for (i=0;i<NBINS;++i) {
//    for (j=0;j<NBINS;++j) {
//      cspace[BASE].boundary_map[i][j] = FREESPACE;
//      if (CartesianMap[i][j] == DIRICHLET) {
//	  cspace[BASE].boundary_map[i][j] = DIRICHLET;
//	  cspace[BASE].potential_map[i][j] = 1.0;
//	}
//      if (CartesianMap[i][j] == GOAL) {
//	  cspace[BASE].boundary_map[i][j] = GOAL;
//	  cspace[BASE].potential_map[i][j] = 0.0;
//	}
//      cspace[BASE].boundary_map[0][j] =
//	cspace[BASE].boundary_map[(NBINS-1)][j] = DIRICHLET;
//      cspace[BASE].potential_map[0][j] =
//	   cspace[BASE].potential_map[(NBINS-1)][j] = 1.0;
//    }
//
//    // joint angle limits:
//    // theta1: is toroidal   theta2: -3.1 rad < theta2 < +3.1 rad
//    cspace[BASE].boundary_map[i][0] =
//       cspace[BASE].boundary_map[i][(NBINS-1)] = DIRICHLET;
//    cspace[BASE].potential_map[i][0] =
//       cspace[BASE].potential_map[i][(NBINS-1)] = 1.0;
//  }
//  //	printf("\n Updating CSPACE for BASE");
//  fflush(stdout);
//}


void draw_score()
{
	char buffer[64];
	int n;

	n = sprintf(buffer, "Time: %6.3lf", possessionTime_1);
	XSetForeground(display, gc, foreground);
	XDrawString(display, pixmap, gc,
				W2DX(zoom,MIN_X-1.0), W2DY(zoom,1.0), buffer, n);

	n = sprintf(buffer, "Time: %6.3lf", possessionTime_2);
	XSetForeground(display, gc, foreground);
	XDrawString(display, pixmap, gc,
				W2DX(zoom,MAX_X+0.5), W2DY(zoom,1.0), buffer, n);

	n = sprintf(buffer, "Goal: %d", goal_1);
	XSetForeground(display, gc, foreground);
	XDrawString(display, pixmap, gc,
				W2DX(zoom,MIN_X-1.0), W2DY(zoom,1.3), buffer, n);

	n = sprintf(buffer, "Goal: %d", goal_2);
	XSetForeground(display, gc, foreground);
	XDrawString(display, pixmap, gc,
				W2DX(zoom,MAX_X+0.5), W2DY(zoom,1.3), buffer, n);

	n = sprintf(buffer, "Roger1");
	XSetForeground(display, gc, foreground);
	XDrawString(display, pixmap, gc,
				W2DX(zoom,MIN_X-1.0), W2DY(zoom,1.6), buffer, n);

	n = sprintf(buffer, "Roger2");
	XSetForeground(display, gc, foreground);
	XDrawString(display, pixmap, gc,
				W2DX(zoom,MAX_X+0.5), W2DY(zoom,1.6), buffer, n);

}


void draw_circle(cu, cv, r, fill)
int cu, cv, r, fill;
{
  if (fill == NOFILL)
    XDrawArc(display, pixmap, gc, cu - r, cv - r, 2* r , 2* r , 0, 64*360);
  else
    XFillArc(display, pixmap, gc, cu-r, cv-r, 2*r, 2*r, 0, 64*360);
}

void draw_frames()
{
#define FRAME_L 0.04
#define FRAME_T 0.045

	XSetForeground(display, gc, foreground);

	// the Cartesian frame
	/* x-axis */
	XDrawLine(display, pixmap, gc,
			  W2DX(zoom,0.0), W2DY(zoom,0.0),
			  W2DX(zoom,(FRAME_L*4.0)), W2DY(zoom,0.0));
	XDrawString(display, pixmap, gc,
				W2DX(zoom,FRAME_T*4.0), W2DY(zoom,0.0), "x", 1);

	/* y-axis */
	XDrawLine(display, pixmap, gc, W2DX(zoom,0.0), W2DY(zoom,0.0),
			  W2DX(zoom,0.0), W2DY(zoom,FRAME_L*4.0));
	XDrawString(display, pixmap, gc,
				W2DX(zoom,0.0), W2DY(zoom,FRAME_T*4.0), "y", 1);
	/*
	// the LEFT CSpace frame
	// q1-axis
	XDrawLine(display, pixmap, gc, T12LD(0.0), T22LD(0.0),
			  T12LD(FRAME_L*2.0*M_PI), T22LD(0.0));
	XDrawString(display, pixmap, gc,
				T12LD(FRAME_T*2.0*M_PI), T22LD(0.0), "q1", 2);

	// q2-axis
	XDrawLine(display, pixmap, gc, T12LD(0.0), T22LD(0.0), T12LD(0.0),
			  T22LD(FRAME_L*2.0*M_PI));
	XDrawString(display, pixmap, gc,
				T12LD(0.0), T22LD(FRAME_T*2.0*M_PI), "q2", 2);

	XDrawString(display, pixmap, gc, T12LD(-0.4), T22LD(-3.5), "left arm", 8);

	// the RIGHT CSpace frame
	// q1-axis
	XDrawLine(display, pixmap, gc, T12RD(0.0), T22RD(0.0),
			  T12RD(FRAME_L*2.0*M_PI), T22RD(0.0));
	XDrawString(display, pixmap, gc,
				T12RD(FRAME_T*2.0*M_PI), T22RD(0.0), "q1", 2);

	// q2-axis
	XDrawLine(display, pixmap, gc, T12RD(0.0), T22RD(0.0), T12RD(0.0),
			  T22RD(FRAME_L*2.0*M_PI));
	XDrawString(display, pixmap, gc,
				T12RD(0.0), T22RD(FRAME_T*2.0*M_PI), "q2", 2);

	XDrawString(display, pixmap, gc, T12RD(-0.45), T22RD(-3.5), "right arm", 9);

	*/

#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}

void draw_frame(xform)
double xform[4][4]; {
#define FRAME_L 0.04
#define FRAME_T 0.045

  XSetForeground(display, gc, foreground);

  /* x-axis */
  XDrawLine(display, pixmap, gc, W2DX(zoom,xform[0][3]),
	    W2DY(zoom,xform[1][3]),
	    W2DX(zoom,xform[0][3]+FRAME_L*xform[0][0]),
	    W2DY(zoom,xform[1][3]+FRAME_L*xform[1][0]));
  XDrawString(display, pixmap, gc,
	      W2DX(zoom,xform[0][3]+FRAME_T*xform[0][0]),
	      W2DY(zoom,xform[1][3]+FRAME_T*xform[1][0]), "x", 1);

  /* y-axis */
  XDrawLine(display, pixmap, gc, W2DX(zoom,xform[0][3]),
	    W2DY(zoom,xform[1][3]),
	    W2DX(zoom,xform[0][3]+FRAME_L*xform[0][1]),
	    W2DY(zoom,xform[1][3]+FRAME_L*xform[1][1]));
  XDrawString(display, pixmap, gc,
	      W2DX(zoom,xform[0][3]+FRAME_T*xform[0][1]),
	      W2DY(zoom,xform[1][3]+FRAME_T*xform[1][1]), "y", 1);
#undef FRAME_L // 0.04
#undef FRAME_T // 0.045
}

void draw_boundaries()
{
  /******************************************************************/
  /**  draw world                                                  **/
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc, W2DX(zoom,MIN_X), W2DY(zoom,MAX_Y),
	    W2DX(zoom,MAX_X), W2DY(zoom,MAX_Y));
  XDrawLine(display, pixmap, gc, W2DX(zoom,MAX_X), W2DY(zoom,MAX_Y),
	    W2DX(zoom,MAX_X), W2DY(zoom,MIN_Y));
  XDrawLine(display, pixmap, gc, W2DX(zoom,MAX_X), W2DY(zoom,MIN_Y),
	    W2DX(zoom,MIN_X), W2DY(zoom,MIN_Y));
  XDrawLine(display, pixmap, gc, W2DX(zoom,MIN_X), W2DY(zoom,MIN_Y),
	    W2DX(zoom,MIN_X), W2DY(zoom,MAX_Y));

	XDrawLine(display, pixmap, gc, W2DX(zoom,MIN_X+(MAX_X-MIN_X)/2), W2DY(zoom,MIN_Y),
			  W2DX(zoom,MIN_X+(MAX_X-MIN_X)/2), W2DY(zoom,MAX_Y));
  /* draw LEFT boundaries */
	/*
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc, T12LD(T1_MIN), T22LD(T2_MAX), T12LD(T1_MAX),
	    T22LD(T2_MAX));
  XDrawLine(display, pixmap, gc, T12LD(T1_MAX), T22LD(T2_MAX), T12LD(T1_MAX),
	    T22LD(T2_MIN));
  XDrawLine(display, pixmap, gc, T12LD(T1_MAX), T22LD(T2_MIN), T12LD(T1_MIN),
	    T22LD(T2_MIN));
  XDrawLine(display, pixmap, gc, T12LD(T1_MIN), T22LD(T2_MIN), T12LD(T1_MIN),
	    T22LD(T2_MAX));
	*/
  /* draw RIGHT boundaries */
	/*
  XSetForeground(display, gc, foreground);
  XDrawLine(display, pixmap, gc, T12RD(T1_MIN), T22RD(T2_MAX), T12RD(T1_MAX),
	    T22RD(T2_MAX));
  XDrawLine(display, pixmap, gc, T12RD(T1_MAX), T22RD(T2_MAX), T12RD(T1_MAX),
	    T22RD(T2_MIN));
  XDrawLine(display, pixmap, gc, T12RD(T1_MAX), T22RD(T2_MIN), T12RD(T1_MIN),
	    T22RD(T2_MIN));
  XDrawLine(display, pixmap, gc, T12RD(T1_MIN), T22RD(T2_MIN), T12RD(T1_MIN),
	    T22RD(T2_MAX));
	 */
}

// draw the NBINSxNBINS potential maps in their respective areas of the canvas
void draw_potential_maps() {
  int i, j, Cart_grey_index, left_arm_grey_index, right_arm_grey_index;
  double x, y, t1, t2;
  double Cart_bin_potential, left_arm_bin_potential, right_arm_bin_potential;

  for (i = 0; i < NBINS; ++i) {
    y = MAX_Y - i*YDELTA;
    t2 = T2_MAX - i*TDELTA;
    for (j = 0; j < NBINS; ++j) {
      x = MIN_X + j*XDELTA;
      t1 = T1_MIN + j*TDELTA;
      // user map grey level fill
      Cart_bin_potential = Roger_1.world_map.potential_map[i][j];
      left_arm_bin_potential = Roger_1.arm_map[LEFT].potential_map[i][j];
      right_arm_bin_potential = Roger_1.arm_map[RIGHT].potential_map[i][j];

      // 0 <= grey indices <= 100
      Cart_grey_index = (int) (Cart_bin_potential * 100.0);
      left_arm_grey_index = (int) (left_arm_bin_potential * 100.0);
      right_arm_grey_index = (int) (right_arm_bin_potential * 100.0);

      // Cartesian Map
      // fill is either:
      //   a grey level depicting the user defined potential
      XSetForeground(display, gc, world_colors[Cart_grey_index].display_color);
      //   a user map perceived obstacle color, or
      if (Roger_1.world_map.occupancy_map[i][j] == OBSTACLE)
	XSetForeground(display, gc,
		world_colors[Roger_1.world_map.color_map[i][j]].display_color);
	   else if (Roger_1.world_map.occupancy_map[i][j] == DILATED_OBSTACLE)
	XSetForeground(display, gc,
		world_colors[Roger_1.world_map.color_map[i][j]].display_color);
      //   a user defined goal
      else if (Roger_1.world_map.occupancy_map[i][j] == GOAL)
	XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc, W2DX(zoom,x), W2DY(zoom,y),
		     (W2DR(zoom,XDELTA) + 1), (W2DR(zoom,YDELTA) + 1));

      //      // each real obstacle should be outlined in the obstacle color
      //      if (real_world.occupancy_map[i][j] == OBSTACLE) {
      //	XSetForeground(display, gc,
      //	       world_colors[real_world.color_map[i][j]].display_color);
      //	x = MIN_X + j*XDELTA; y = MAX_Y - i*YDELTA;
      //	XDrawRectangle(display, pixmap, gc, W2DX(zoom,x), W2DY(zoom,y),
      //		     (W2DR(zoom,XDELTA)), (W2DR(zoom,YDELTA)));
      //      }

      // Left Arm Map
 /*     XSetForeground(display, gc,
		     world_colors[left_arm_grey_index].display_color);
      if (Roger_1.arm_map[LEFT].occupancy_map[i][j] == OBSTACLE)
	XSetForeground(display, gc, world_colors[RED].display_color);
      else if (Roger_1.arm_map[LEFT].occupancy_map[i][j] == GOAL)
	XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc, T12LD(t1), T22LD(t2),
		     (T2DR(TDELTA) + 1), (T2DR(TDELTA) + 1));

      // Right Arm Map
      XSetForeground(display, gc,
		     world_colors[right_arm_grey_index].display_color);
      if (Roger_1.arm_map[RIGHT].occupancy_map[i][j] == OBSTACLE)
	XSetForeground(display, gc, world_colors[RED].display_color);
      else if (Roger_1.arm_map[RIGHT].occupancy_map[i][j] == GOAL)
	XSetForeground(display, gc, world_colors[GOAL_COLOR].display_color);
      XFillRectangle(display, pixmap, gc, T12RD(t1), T22RD(t2),
		     (T2DR(TDELTA) + 1), (T2DR(TDELTA) + 1));
  */
    }
  }
  draw_boundaries();
  draw_frames();
}

void draw_object(obj)
Obj obj;
{
  XSetForeground(display, gc, world_colors[OBJECT_COLOR].display_color);
  draw_circle(W2DX(zoom,obj.position[X]), W2DY(zoom,obj.position[Y]),
	      W2DR(zoom,R_OBJ), FILL);
}

void draw_eye_1(base, eye)
Base base;
Eye eye;
{
  double px, py;
  double rx, ry, from_x, from_y, to_x, to_y;
  double lambda_x, lambda_y;
  int xbin, ybin;

  px = base.wTb[0][0]*eye.position[0] + base.wTb[0][1]*eye.position[1] +
    base.wTb[0][3];
  py = base.wTb[1][0]*eye.position[0] + base.wTb[1][1]*eye.position[1] +
    base.wTb[1][3];

  from_x = px; from_y = py;
  rx = cos(base.theta + eye.theta);
  ry = sin(base.theta + eye.theta);
  /*
  //old code to cut off eye gaze at map boundary
  lambda_x = lambda_y = 1000.0;
  if (rx > 0.1)	lambda_x = (MAX_X - px) / rx;
  else if (rx < -0.1) lambda_x = (MIN_X - px) / rx;
  if (ry > 0.1) lambda_y = (MAX_Y - py) / ry;
  else if (ry < -0.1) lambda_y = (MIN_Y - py) / ry;

  if (lambda_x < lambda_y) {
    if (rx > 0.0) to_x = MAX_X - XDELTA;
    else to_x = MIN_X + XDELTA;
    to_y = py + lambda_x * ry;
  }
  else {
    if (ry > 0.0) to_y = MAX_Y - YDELTA;
    else to_y = MIN_Y + YDELTA;
    to_x = px + lambda_y * rx;
  }
  */


  //trace the eye direction till you hit an obstacle
  to_x = from_x;
  to_y = from_y;

  while (to_x < MAX_X && to_x > MIN_X && to_y < MAX_Y && to_y > MIN_Y)
  {
		//get bin for location
		ybin = (int)((MAX_Y - to_y)/YDELTA);
		xbin = (int)((to_x - MIN_X)/XDELTA);

		//check for obstacle collision
  		if (Roger_1.world_map.occupancy_map[ybin][xbin] == OBSTACLE)
		{
			break;
		}
		to_x += rx * 0.001;
		to_y += ry * 0.001;
  }

  XSetForeground(display, gc, world_colors[GAZE_COLOR].display_color);
  XDrawLine(display, pixmap, gc, W2DX(zoom,from_x), W2DY(zoom,from_y),
	    W2DX(zoom,to_x), W2DY(zoom,to_y));

  XSetForeground(display, gc, foreground);
  draw_circle(W2DX(zoom,px), W2DY(zoom,py), W2DR(zoom,R_EYE), NOFILL);
  draw_circle(W2DX(zoom, px+(R_EYE-R_PUPIL)*rx),
	      W2DY(zoom, py+(R_EYE-R_PUPIL)*ry), W2DR(zoom,R_PUPIL),
	      FILL);
}

void draw_eye_2(base, eye)
Base base;
Eye eye;
{
	double px, py;
	double rx, ry, from_x, from_y, to_x, to_y;
	double lambda_x, lambda_y;
	int xbin, ybin;

	px = base.wTb[0][0]*eye.position[0] + base.wTb[0][1]*eye.position[1] +
    base.wTb[0][3];
	py = base.wTb[1][0]*eye.position[0] + base.wTb[1][1]*eye.position[1] +
    base.wTb[1][3];

	from_x = px; from_y = py;
	rx = cos(base.theta + eye.theta);
	ry = sin(base.theta + eye.theta);
	/*
	 //old code to cut off eye gaze at map boundary
	 lambda_x = lambda_y = 1000.0;
	 if (rx > 0.1)	lambda_x = (MAX_X - px) / rx;
	 else if (rx < -0.1) lambda_x = (MIN_X - px) / rx;
	 if (ry > 0.1) lambda_y = (MAX_Y - py) / ry;
	 else if (ry < -0.1) lambda_y = (MIN_Y - py) / ry;

	 if (lambda_x < lambda_y) {
	 if (rx > 0.0) to_x = MAX_X - XDELTA;
	 else to_x = MIN_X + XDELTA;
	 to_y = py + lambda_x * ry;
	 }
	 else {
	 if (ry > 0.0) to_y = MAX_Y - YDELTA;
	 else to_y = MIN_Y + YDELTA;
	 to_x = px + lambda_y * rx;
	 }
	 */


	//trace the eye direction till you hit an obstacle
	to_x = from_x;
	to_y = from_y;

	while (to_x < MAX_X && to_x > MIN_X && to_y < MAX_Y && to_y > MIN_Y)
	{
		//get bin for location
		ybin = (int)((MAX_Y - to_y)/YDELTA);
		xbin = (int)((to_x - MIN_X)/XDELTA);

		//check for obstacle collision
  		if (Roger_2.world_map.occupancy_map[ybin][xbin] == OBSTACLE)
		{
			break;
		}
		to_x += rx * 0.001;
		to_y += ry * 0.001;
	}

	XSetForeground(display, gc, world_colors[GAZE_COLOR].display_color);
	XDrawLine(display, pixmap, gc, W2DX(zoom,-from_x), W2DY(zoom,from_y),
			  W2DX(zoom,-to_x), W2DY(zoom,to_y));

	XSetForeground(display, gc, foreground);
	draw_circle(W2DX(zoom,-px), W2DY(zoom,py), W2DR(zoom,R_EYE), NOFILL);
	draw_circle(W2DX(zoom, -(px+(R_EYE-R_PUPIL)*rx)),
				W2DY(zoom, py+(R_EYE-R_PUPIL)*ry), W2DR(zoom,R_PUPIL),
				FILL);
}

void draw_image(eye, rogerID)
int eye;
int rogerID;
{
    int i, color, dx;

	if (rogerID == 1) {
		XSetForeground(display, gc, foreground);
		if (eye == LEFT)
			XDrawRectangle(display, pixmap, gc, LEFT_IMAGE_X_1 - 1, IMAGE_Y - 1,
						   IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);
		else if (eye == RIGHT)
			XDrawRectangle(display, pixmap, gc, RIGHT_IMAGE_X_1 - 1, IMAGE_Y - 1,
						   IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);

		for (i = 0, dx = 0; i < NPIXELS; i++, dx += PIXEL_WIDTH) {
			color = eyes_1[eye].image[i];
			//    XSetForeground(display, gc, color);

			// Commented out by Dan.
			//printf("color=%d\n", color);

			//    if (color > 99) XSetForeground(display, gc, background);
			//    else if (intensity >= 0)
			//      XSetForeground(display, gc, image_color[intensity]);
			//    if (color == ARM_COLOR)
			//      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
			//    else if (color == ARM_COLOR)
			//      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
			//    else if (color == OBJECT_COLOR)
			//      XSetForeground(display, gc,
			//         world_colors[OBJECT_COLOR].display_color);
			XSetForeground(display, gc, world_colors[color].display_color);

			if (eye == LEFT)
				XFillRectangle(display, pixmap, gc, LEFT_IMAGE_X_1 + dx, IMAGE_Y,
							   PIXEL_WIDTH, PIXEL_HEIGHT);
			else if (eye == RIGHT)
				XFillRectangle(display, pixmap, gc, RIGHT_IMAGE_X_1 + dx, IMAGE_Y,
							   PIXEL_WIDTH, PIXEL_HEIGHT);
		}
	}
	else if (rogerID == 2) {
		XSetForeground(display, gc, foreground);
		if (eye == LEFT)
			XDrawRectangle(display, pixmap, gc, LEFT_IMAGE_X_2 - 1, IMAGE_Y - 1,
						   IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);
		else if (eye == RIGHT)
			XDrawRectangle(display, pixmap, gc, RIGHT_IMAGE_X_2 - 1, IMAGE_Y - 1,
						   IMAGE_WIDTH + 1, PIXEL_HEIGHT + 1);

		for (i = 0, dx = 0; i < NPIXELS; i++, dx += PIXEL_WIDTH) {
			color = eyes_2[eye].image[i];
			//    XSetForeground(display, gc, color);

			// Commented out by Dan.
			//printf("color=%d\n", color);

			//    if (color > 99) XSetForeground(display, gc, background);
			//    else if (intensity >= 0)
			//      XSetForeground(display, gc, image_color[intensity]);
			//    if (color == ARM_COLOR)
			//      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
			//    else if (color == ARM_COLOR)
			//      XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
			//    else if (color == OBJECT_COLOR)
			//      XSetForeground(display, gc,
			//         world_colors[OBJECT_COLOR].display_color);
			XSetForeground(display, gc, world_colors[color].display_color);

			if (eye == LEFT)
				XFillRectangle(display, pixmap, gc, LEFT_IMAGE_X_2 + dx, IMAGE_Y,
							   PIXEL_WIDTH, PIXEL_HEIGHT);
			else if (eye == RIGHT)
				XFillRectangle(display, pixmap, gc, RIGHT_IMAGE_X_2 + dx, IMAGE_Y,
							   PIXEL_WIDTH, PIXEL_HEIGHT);
		}
	}
}


void draw_roger_1(mobile_base, arms, eyes)
Base mobile_base;
Arm arms[NARMS][NARM_FRAMES];
Eye eyes[NEYES];
{
  int i, j;
  double r_b[4], r_w[4], fhat[2];
  double theta1, theta2, mag;
  double temp0[4][4], temp1[4][4];
  XPoint rect[4];
  void draw_history_1();

  /******************************************************************/
  /* draw mobile base */
  XSetForeground(display, gc, foreground);
  draw_circle(W2DX(zoom,mobile_base.wTb[0][3]),
	      W2DY(zoom,mobile_base.wTb[1][3]), W2DR(zoom,R_BASE), NOFILL);

  // draw contact forces on object from body
  mag = sqrt(SQR(mobile_base.extForce[X]) + SQR(mobile_base.extForce[Y]));

  if (mag > 0.0) {
    fhat[X] = mobile_base.extForce[X] / mag;
    fhat[Y] = mobile_base.extForce[Y] / mag;

    XDrawLine(display, pixmap, gc,
	      W2DX(zoom, mobile_base.x - R_BASE*fhat[X]),
	      W2DY(zoom, mobile_base.y - R_BASE*fhat[Y]),
	      W2DX(zoom, mobile_base.x - (R_BASE+0.08)*fhat[X]),
	      W2DY(zoom, mobile_base.y - (R_BASE+0.08)*fhat[Y]));

    //    XDrawLine(display, pixmap, gc,
    //      W2DX(zoom, mobile_base.x + R_BASE*cos(mobile_base.contact_theta)),
    //      W2DY(zoom, mobile_base.y + R_BASE*sin(mobile_base.contact_theta)),
    //      W2DX(zoom, mobile_base.x + R_BASE*cos(mobile_base.contact_theta)
    //	   + 0.08*mobile_base.extforce[Y]/mag),
    //      W2DY(zoom, mobile_base.y + R_BASE*sin(mobile_base.contact_theta)
    //	   - 0.08*mobile_base.extforce[X]/mag));
  }

  //  draw_wheels();
  r_b[0] = R_BASE/2.0; r_b[1] = R_BASE+R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(W2DX(zoom, r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),
	      FILL);
  r_b[0] = -R_BASE/2.0; r_b[1] = R_BASE+R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(W2DX(zoom, r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),
	      FILL);

  r_b[0] = R_BASE/2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[0].x = (short) (W2DX(zoom, r_w[0]));
  rect[0].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = R_BASE/2.0; r_b[1] = (R_BASE+2*R_WHEEL); r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[1].x = (short) (W2DX(zoom, r_w[0]));
  rect[1].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = -R_BASE/2.0; r_b[1] = (R_BASE+2*R_WHEEL); r_b[2]=0.0; r_b[3]=1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[2].x = (short) (W2DX(zoom, r_w[0]));
  rect[2].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = -R_BASE / 2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[3].x = (short) (W2DX(zoom, r_w[0]));
  rect[3].y = (short) (W2DY(zoom, r_w[1]));

  XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);

  r_b[0] = R_BASE/2.0; r_b[1] = -R_BASE-R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(W2DX(zoom, r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),FILL);
  r_b[0] = -R_BASE/2.0; r_b[1] = -R_BASE-R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  draw_circle(W2DX(zoom, r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),FILL);

  r_b[0] = R_BASE/2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[0].x = (short) (W2DX(zoom, r_w[0]));
  rect[0].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = R_BASE/2.0; r_b[1] = -(R_BASE+2*R_WHEEL); r_b[2] = 0.0; r_b[3]=1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[1].x = (short) (W2DX(zoom, r_w[0]));
  rect[1].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = -R_BASE/2.0; r_b[1] = -(R_BASE+2*R_WHEEL); r_b[2] = 0.0; r_b[3]=1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[2].x = (short) (W2DX(zoom, r_w[0]));
  rect[2].y = (short) (W2DY(zoom, r_w[1]));

  r_b[0] = -R_BASE/2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
  SIMmatXvec(mobile_base.wTb, r_b, r_w);
  rect[3].x = (short) (W2DX(zoom, r_w[0]));
  rect[3].y = (short) (W2DY(zoom, r_w[1]));

  XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);
  /******************************************************************/
  /* draw eyes */
  for (i = 0; i < NEYES; i++)
  {
    draw_eye_1(mobile_base, eyes[i]);

    /******************************************************************/
    /* draw displays **************************************************/
    /* draw coordinate in configuration space for left and right eyes */
 /*   XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
    if (i == LEFT)
      XFillRectangle(display, pixmap, gc, T12LD(eyes[i].theta),
		     T22LD(0.0), (T2DR(TDELTA) + 1), (T2DR(TDELTA) + 1));
    else if (i == RIGHT)
      XFillRectangle(display, pixmap, gc, T12RD(eyes[i].theta),
		     T22RD(0.0), (T2DR(TDELTA) + 1), (T2DR(TDELTA) + 1));*/
  }
  /******************************************************************/
  /* draw arms */
  for (j=0; j<NARMS; j++) {
    XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
    SIMmatXmat(mobile_base.wTb, arms[j][0].iTj, temp0);

    for (i=1; i<NARM_FRAMES; i++) {
      SIMmatXmat(temp0, arms[j][i].iTj, temp1);
      XDrawLine(display, pixmap, gc, W2DX(zoom,temp0[0][3]),
		W2DY(zoom,temp0[1][3]), W2DX(zoom,temp1[0][3]),
		W2DY(zoom,temp1[1][3]));
      if (i==(NARM_FRAMES-1))
	draw_circle(W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]),
		    W2DR(zoom,R_TACTILE), FILL);
      else {
	draw_circle(W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]),
		    W2DR(zoom,R_JOINT), NOFILL);
	SIMcopy_matrix(temp1, temp0);
      }
    }

    // draw endpoint forces
    mag = sqrt(SQR(arms[j][NARM_FRAMES-1].extForce[X]) +
	       SQR(arms[j][NARM_FRAMES-1].extForce[Y]));
    if (mag>0.0) {
      XDrawLine(display, pixmap, gc,
		W2DX(zoom,temp1[0][3]),	W2DY(zoom,temp1[1][3]),
          W2DX(zoom,temp1[0][3] - 0.08*arms[j][NARM_FRAMES-1].extForce[X]/mag),
         W2DY(zoom,temp1[1][3] - 0.08*arms[j][NARM_FRAMES-1].extForce[Y]/mag));

      // XDrawLine(display, pixmap, gc,
      //   W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]),
      //   W2DX(zoom,temp1[0][3] + 0.125*arms[j][NARM_FRAMES-1].extForce[Y]/mag),
      //   W2DY(zoom,temp1[1][3] - 0.125*arms[j][NARM_FRAMES-1].extForce[X]/mag));
    }

    /******************************************************************/
    /* draw displays **************************************************/
    /* draw coordinate in configuration space for left and right arms */
 /*   if (j == LEFT)
      XFillRectangle(display, pixmap, gc, T12LD(arms[j][1].theta),
		     T22LD(arms[j][2].theta), (T2DR(TDELTA) + 1),
		     (T2DR(TDELTA) + 1));
    else if (j == RIGHT)
      XFillRectangle(display, pixmap, gc, T12RD(arms[j][1].theta),
		     T22RD(arms[j][2].theta), (T2DR(TDELTA) + 1),
		     (T2DR(TDELTA) + 1));
  */
  }
  if (HISTORY) draw_history_1();

  /* visual images *************************************************/
  draw_image(LEFT, 1);
  draw_image(RIGHT, 1);
}

void draw_roger_2(mobile_base, arms, eyes)
Base mobile_base;
Arm arms[NARMS][NARM_FRAMES];
Eye eyes[NEYES];
{
	//register i, j;
	int i,j;
        double r_b[4], r_w[4], fhat[2];
	double theta1, theta2, mag;
	double temp0[4][4], temp1[4][4];
	XPoint rect[4];
	void draw_history_2();

	/******************************************************************/
	/* draw mobile base */
	XSetForeground(display, gc, foreground);
	draw_circle(W2DX(zoom,-mobile_base.wTb[0][3]),
				W2DY(zoom,mobile_base.wTb[1][3]), W2DR(zoom,R_BASE), NOFILL);

	// draw contact forces on object from body
	mag = sqrt(SQR(mobile_base.extForce[X]) + SQR(mobile_base.extForce[Y]));

	if (mag > 0.0) {
		fhat[X] = mobile_base.extForce[X] / mag;
		fhat[Y] = mobile_base.extForce[Y] / mag;

		XDrawLine(display, pixmap, gc,
				  W2DX(zoom,-( mobile_base.x - R_BASE*fhat[X])),
				  W2DY(zoom, mobile_base.y - R_BASE*fhat[Y]),
				  W2DX(zoom, -(mobile_base.x - (R_BASE+0.08)*fhat[X])),
				  W2DY(zoom, mobile_base.y - (R_BASE+0.08)*fhat[Y]));

		//    XDrawLine(display, pixmap, gc,
		//      W2DX(zoom, mobile_base.x + R_BASE*cos(mobile_base.contact_theta)),
		//      W2DY(zoom, mobile_base.y + R_BASE*sin(mobile_base.contact_theta)),
		//      W2DX(zoom, mobile_base.x + R_BASE*cos(mobile_base.contact_theta)
		//	   + 0.08*mobile_base.extforce[Y]/mag),
		//      W2DY(zoom, mobile_base.y + R_BASE*sin(mobile_base.contact_theta)
		//	   - 0.08*mobile_base.extforce[X]/mag));
	}

	//  draw_wheels();
	r_b[0] = R_BASE/2.0; r_b[1] = R_BASE+R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	draw_circle(W2DX(zoom, -r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),
				FILL);
	r_b[0] = -R_BASE/2.0; r_b[1] = R_BASE+R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	draw_circle(W2DX(zoom, -r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),
				FILL);

	r_b[0] = R_BASE/2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	rect[0].x = (short) (W2DX(zoom, -r_w[0]));
	rect[0].y = (short) (W2DY(zoom, r_w[1]));

	r_b[0] = R_BASE/2.0; r_b[1] = (R_BASE+2*R_WHEEL); r_b[2] = 0.0; r_b[3] = 1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	rect[1].x = (short) (W2DX(zoom, -r_w[0]));
	rect[1].y = (short) (W2DY(zoom, r_w[1]));

	r_b[0] = -R_BASE/2.0; r_b[1] = (R_BASE+2*R_WHEEL); r_b[2]=0.0; r_b[3]=1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	rect[2].x = (short) (W2DX(zoom, -r_w[0]));
	rect[2].y = (short) (W2DY(zoom, r_w[1]));

	r_b[0] = -R_BASE / 2.0; r_b[1] = R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	rect[3].x = (short) (W2DX(zoom, -r_w[0]));
	rect[3].y = (short) (W2DY(zoom, r_w[1]));

	XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);

	r_b[0] = R_BASE/2.0; r_b[1] = -R_BASE-R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	draw_circle(W2DX(zoom, -r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),FILL);
	r_b[0] = -R_BASE/2.0; r_b[1] = -R_BASE-R_WHEEL; r_b[2] = 0.0; r_b[3] = 1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	draw_circle(W2DX(zoom, -r_w[0]), W2DY(zoom, r_w[1]), W2DR(zoom,R_WHEEL),FILL);

	r_b[0] = R_BASE/2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	rect[0].x = (short) (W2DX(zoom, -r_w[0]));
	rect[0].y = (short) (W2DY(zoom, r_w[1]));

	r_b[0] = R_BASE/2.0; r_b[1] = -(R_BASE+2*R_WHEEL); r_b[2] = 0.0; r_b[3]=1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	rect[1].x = (short) (W2DX(zoom, -r_w[0]));
	rect[1].y = (short) (W2DY(zoom, r_w[1]));

	r_b[0] = -R_BASE/2.0; r_b[1] = -(R_BASE+2*R_WHEEL); r_b[2] = 0.0; r_b[3]=1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	rect[2].x = (short) (W2DX(zoom, -r_w[0]));
	rect[2].y = (short) (W2DY(zoom, r_w[1]));

	r_b[0] = -R_BASE/2.0; r_b[1] = -R_BASE; r_b[2] = 0.0; r_b[3] = 1.0;
	SIMmatXvec(mobile_base.wTb, r_b, r_w);
	rect[3].x = (short) (W2DX(zoom, -r_w[0]));
	rect[3].y = (short) (W2DY(zoom, r_w[1]));

	XFillPolygon(display, pixmap, gc, rect, 4, Convex, CoordModeOrigin);
	/******************************************************************/
	/* draw eyes */
	for (i = 0; i < NEYES; i++)
	{
		draw_eye_2(mobile_base, eyes[i]);

		/******************************************************************/
		/* draw displays **************************************************/
		/* draw coordinate in configuration space for left and right eyes */
	/*	XSetForeground(display, gc, world_colors[EYE_COLOR].display_color);
		if (i == LEFT)
			XFillRectangle(display, pixmap, gc, T12LD(eyes[i].theta),
						   T22LD(0.0), (T2DR(TDELTA) + 1), (T2DR(TDELTA) + 1));
		else if (i == RIGHT)
			XFillRectangle(display, pixmap, gc, T12RD(eyes[i].theta),
						   T22RD(0.0), (T2DR(TDELTA) + 1), (T2DR(TDELTA) + 1));*/
	}
	/******************************************************************/
	/* draw arms */
	for (j=0; j<NARMS; j++) {
		XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);
		SIMmatXmat(mobile_base.wTb, arms[j][0].iTj, temp0);

		for (i=1; i<NARM_FRAMES; i++) {
			SIMmatXmat(temp0, arms[j][i].iTj, temp1);
			XDrawLine(display, pixmap, gc, W2DX(zoom,-temp0[0][3]),
					  W2DY(zoom,temp0[1][3]), W2DX(zoom,-temp1[0][3]),
					  W2DY(zoom,temp1[1][3]));
			if (i==(NARM_FRAMES-1))
				draw_circle(W2DX(zoom,-temp1[0][3]), W2DY(zoom,temp1[1][3]),
							W2DR(zoom,R_TACTILE), FILL);
			else {
				draw_circle(W2DX(zoom,-temp1[0][3]), W2DY(zoom,temp1[1][3]),
							W2DR(zoom,R_JOINT), NOFILL);
				SIMcopy_matrix(temp1, temp0);
			}
		}

		// draw endpoint forces
		mag = sqrt(SQR(arms[j][NARM_FRAMES-1].extForce[X]) +
				   SQR(arms[j][NARM_FRAMES-1].extForce[Y]));
		if (mag>0.0) {
			XDrawLine(display, pixmap, gc,
					  W2DX(zoom,-temp1[0][3]),	W2DY(zoom,temp1[1][3]),
					  W2DX(zoom,-temp1[0][3] - 0.08*arms[j][NARM_FRAMES-1].extForce[X]/mag),
					  W2DY(zoom,-temp1[1][3] - 0.08*arms[j][NARM_FRAMES-1].extForce[Y]/mag));

			// XDrawLine(display, pixmap, gc,
			//   W2DX(zoom,temp1[0][3]), W2DY(zoom,temp1[1][3]),
			//   W2DX(zoom,temp1[0][3] + 0.125*arms[j][NARM_FRAMES-1].extForce[Y]/mag),
			//   W2DY(zoom,temp1[1][3] - 0.125*arms[j][NARM_FRAMES-1].extForce[X]/mag));
		}

		/******************************************************************/
		/* draw displays **************************************************/
		/* draw coordinate in configuration space for left and right arms */
		/*   if (j == LEFT)
		 XFillRectangle(display, pixmap, gc, T12LD(arms[j][1].theta),
		 T22LD(arms[j][2].theta), (T2DR(TDELTA) + 1),
		 (T2DR(TDELTA) + 1));
		 else if (j == RIGHT)
		 XFillRectangle(display, pixmap, gc, T12RD(arms[j][1].theta),
		 T22RD(arms[j][2].theta), (T2DR(TDELTA) + 1),
		 (T2DR(TDELTA) + 1));
		 */
	}
	if (HISTORY) draw_history_2();

	/* visual images *************************************************/
	draw_image(LEFT, 2);
	draw_image(RIGHT, 2);
}

void draw_all()
{
  int n;
  char buffer[64];
  void draw_potential_maps(), draw_object(), draw_roger();

  x_clear();

  draw_potential_maps();
  draw_object(object);

	draw_roger_1(mobile_base_1, arms_1, eyes_1);
	draw_roger_2(mobile_base_2, arms_2, eyes_2);

	draw_score();
  //draw_roger(mobile_base, arms, eyes);
  //  draw_ellipse(manipulator(LEFT));
  //  draw_ellipse(manipulator(RIGHT));

  // XSetForeground(display, gc, goal_color);
  //  draw_ellipse(observation);
  //  draw_ellipse(spatial_goals[CENTROID]);
  //  draw_ellipse(spatial_goals[LEFT_EDGE]);
  //  draw_ellipse(spatial_goals[RIGHT_EDGE]);

  //  if (p_index==6) draw_ellipse(grasp_goal);
  //  XSetForeground(display, gc, target_color);
  //  draw_ellipse(target);

  n = sprintf(buffer, "total elapsed time = %6.3lf", simtime);
  XSetForeground(display, gc, foreground);
  XDrawString(display, pixmap, gc,
	      W2DX(zoom,3.0), W2DY(zoom,1.8), buffer, n);

  x_expose();
}



void draw_history_1()
{
  int h;

  // draw history of all Cartesian arm postures
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);

  for (h = 0; h < history_ptr_1; ++h) {
    // draw Cartesian history of the mobile platform
    XFillRectangle(display, pixmap, gc,
		   W2DX(zoom, (history_1[h].base_pos[0]-XDELTA/4.0)),
		   W2DY(zoom, (history_1[h].base_pos[1]-YDELTA/4.0)),
		   W2DR(zoom,XDELTA/2.0),
		   W2DR(zoom,YDELTA/2.0));
  }
}

void draw_history_2()
{
  int h;
  
  // draw history of all Cartesian arm postures
  XSetForeground(display, gc, world_colors[ARM_COLOR].display_color);

  for (h = 0; h < history_ptr_2; ++h) {
    // draw Cartesian history of the mobile platform
    XFillRectangle(display, pixmap, gc,
		   W2DX(zoom, (history_2[h].base_pos[0]-XDELTA/4.0)),
		   W2DY(zoom, (history_2[h].base_pos[1]-YDELTA/4.0)),
		   W2DR(zoom,XDELTA/2.0),
		   W2DR(zoom,YDELTA/2.0));
  }
}

#define VMAG 0.5

// observations in world coordinates
void draw_estimate0(scale, est)
double scale;
Estimate est;
{
  draw_circle(W2DX(ZOOM_SCALE, est.state[X]),
	      W2DY(ZOOM_SCALE, est.state[Y]),
	      W2DR(ZOOM_SCALE,R_BASE), NOFILL);
  
  x_draw_line(BLUE, est.state[X], est.state[Y],
	      (est.state[X] + VMAG*est.state[XDOT]),
	      (est.state[Y] + VMAG*est.state[YDOT]));
}

#define FRAME_L 0.08

// observations in world coordinates
void draw_observation(obs)
Observation obs;
{
  double a, b, c, root[2];

  double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
  double ref_w[4], ref_b[4], wTb[4][4], bTw[4][4];
  double theta, dx0, dy0, dx1, dy1;

  printf("inside draw_observation()\n");
  printf("x=%6.4lf y=%6.4lf \n\n", obs.pos[X], obs.pos[Y]);

  //printf("   %lf %lf\n", obs.cov[0][0], obs.cov[0][1]);
  //printf("   %lf %lf\n", obs.cov[1][0], obs.cov[1][1]);

  // DRAW THE CURRENT ESTIMATED STATE VARIABLES AND THEIR VELOCITIES
  //  draw_circle(W2DX(zoom,est.state[X]), W2DY(zoom,est.state[Y]),
  //            W2DR(zoom,R_JOINT), FILL);
  
  // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) + c
  //       [B  C]
  a = 1.0;
  b = -(obs.cov[0][0] + obs.cov[1][1]);
  c = obs.cov[0][0] * obs.cov[1][1] - obs.cov[1][0] * obs.cov[0][1];

  root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
  root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

  if (fabs(obs.cov[1][0]) > 0.0001) {
    dy = 1.0;
    dx = -(obs.cov[1][1]-root[0])/obs.cov[1][0];

    mag = sqrt(SQR(dx) + SQR(dy));
    //    eigenvalue[0] = scale*sqrt(root[0]);
    eigenvalue[0] = sqrt(root[0]);
    eigenvector[0][0] = dx / mag;
    eigenvector[0][1] = dy / mag;

    // the eigenvector for root 1
    //    eigenvalue[1] = scale*sqrt(root[1]);
    eigenvalue[1] = sqrt(root[1]);
    eigenvector[1][0] = -eigenvector[0][1];
    eigenvector[1][1] = eigenvector[0][0];
  }
  // when ball is directly in front of Roger:
  else{
    // set eigenvalue 0 to be the greater root
    if (fabs(root[0]) > fabs(root[1])) {
      eigenvalue[0] = sqrt(root[0]);
      eigenvalue[1] = sqrt(root[1]);
    }
    else {
      eigenvalue[0] = sqrt(root[1]);
      eigenvalue[1] = sqrt(root[0]);
    }

    // when cov item A > cov item C, Roger is facing either of the green
    // walls; when cov item A < cov item C, Roger is facing either of the
    // blue walls
    if (fabs(obs.cov[0][0]) > fabs(obs.cov[1][1])) {
      eigenvector[0][0] = 1.0;
      eigenvector[0][1] = 0.0;
      eigenvector[1][0] = -0.0;
      eigenvector[1][1] = 1.0;
    }
    else {
      eigenvector[0][0] = 0.0;
      eigenvector[0][1] = 1.0;
      eigenvector[1][0] = -1.0;
      eigenvector[1][1] = 0.0;
    }
  }
  

  // draw cross hair
  XDrawLine(display, pixmap, gc,
            W2DX(zoom,(obs.pos[X] - (FRAME_L/2.0)*eigenvector[0][X])),
            W2DY(zoom,(obs.pos[Y] - (FRAME_L/2.0)*eigenvector[0][Y])),
            W2DX(zoom,(obs.pos[X] + (FRAME_L/2.0)*eigenvector[0][X])),
            W2DY(zoom,(obs.pos[Y] + (FRAME_L/2.0)*eigenvector[0][Y])));
  XDrawLine(display, pixmap, gc,
            W2DX(zoom,(obs.pos[X] - (FRAME_L/2.0)*eigenvector[1][X])),
            W2DY(zoom,(obs.pos[Y] - (FRAME_L/2.0)*eigenvector[1][Y])),
            W2DX(zoom,(obs.pos[X] + (FRAME_L/2.0)*eigenvector[1][X])),
            W2DY(zoom,(obs.pos[Y] + (FRAME_L/2.0)*eigenvector[1][Y])));
  dx0 = eigenvalue[0] * eigenvector[0][X];
  dy0 = eigenvalue[0] * eigenvector[0][Y];

  //printf("observation cov:\n");
  //printf("\t%lf %lf\n\t%lf %lf\n", obs.cov[0][0], obs.cov[0][1], obs.cov[1][0], obs.cov[1][1]);

  for (theta = 0.0; theta <= 2*M_PI; theta += M_PI / 20.0) {
    dx1 = (eigenvalue[0]*cos(theta))*eigenvector[0][X] +
      (eigenvalue[1]*sin(theta))*eigenvector[1][X];
    dy1 = (eigenvalue[0]*cos(theta))*eigenvector[0][Y] +
      (eigenvalue[1]*sin(theta))*eigenvector[1][Y];
    XDrawLine(display, pixmap, gc, W2DX(zoom,(obs.pos[X] + dx0)),
	      W2DY(zoom,(obs.pos[Y] + dy0)),
	      W2DX(zoom,(obs.pos[X] + dx1)),
	      W2DY(zoom,(obs.pos[Y] + dy1)));
    dx0 = dx1;
    dy0 = dy1;
  }
}

// estimates in world coordinates
void draw_estimate(scale, est)
    double scale;
    Estimate est;
{
    double a, b, c, root[2];
    double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
    double ref_w[2], ref_b[2], wTb[4][4], bTw[4][4];
    double theta, dx0, dy0, dx1, dy1;

    printf("inside draw_estimate()\n");
    printf("x=%6.4lf y=%6.4lf xdot=%6.4lf ydot=%6.4lf\n\n", est.state[X], est.state[Y], est.state[XDOT], est.state[YDOT]);

    //printf("   %lf %lf\n", est.cov[0][0], est.cov[0][1]);
    //printf("   %lf %lf\n", est.cov[1][0], est.cov[1][1]);

    // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) + c
    //       [B  C]
    a = 1.0;
    b = -(est.cov[0][0] + est.cov[1][1]);
    c = est.cov[0][0] * est.cov[1][1] - est.cov[1][0] * est.cov[0][1];

    root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
    root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

    if (fabs(est.cov[1][0]) > 0.000001) {
        dy = 1.0;
        dx = -(est.cov[1][1]-root[0])/est.cov[1][0];

        mag = sqrt(SQR(dx) + SQR(dy));
        //      eigenvalue[0] = scale*0.0221*sqrt(root[0]);
        eigenvalue[0] = scale*sqrt(root[0]);
        eigenvector[0][0] = dx / mag;
        eigenvector[0][1] = dy / mag;

        // the eigenvector for root 1
        //      eigenvalue[1] = scale*0.0221*sqrt(root[1]);
        eigenvalue[1] = scale*sqrt(root[1]);
        eigenvector[1][0] = -eigenvector[0][1];
        eigenvector[1][1] = eigenvector[0][0];
    }
    else{
        // set eigenvalue[0] to be the greater root
        if (fabs(root[0]) > fabs(root[1])) {
            eigenvalue[0] = scale*sqrt(root[0]);
            eigenvalue[1] = scale*sqrt(root[1]);
        }
        else {
            eigenvalue[0] = scale*sqrt(root[1]);
            eigenvalue[1] = scale*sqrt(root[0]);
        }

        // when cov item A > cov item C, Roger is facing either of the green
        // walls; when cov item A < cov item C, Roger is facing either of the
        // blue walls
        if (fabs(est.cov[0][0]) > fabs(est.cov[1][1])) {
            eigenvector[0][0] = 1.0;
            eigenvector[0][1] = 0.0;
            eigenvector[1][0] = -0.0;
            eigenvector[1][1] = 1.0;
        }
        else {
            eigenvector[0][0] = 0.0;
            eigenvector[0][1] = 1.0;
            eigenvector[1][0] = -1.0;
            eigenvector[1][1] = 0.0;
        }
    }


    // draw cross hair
    XDrawLine(display, pixmap, gc,
            W2DX(zoom,(est.state[X] - (FRAME_L/2.0)*eigenvector[0][X])),
            W2DY(zoom,(est.state[Y] - (FRAME_L/2.0)*eigenvector[0][Y])),
            W2DX(zoom,(est.state[X] + (FRAME_L/2.0)*eigenvector[0][X])),
            W2DY(zoom,(est.state[Y] + (FRAME_L/2.0)*eigenvector[0][Y])));
    XDrawLine(display, pixmap, gc,
            W2DX(zoom,(est.state[X] - (FRAME_L/2.0)*eigenvector[1][X])),
            W2DY(zoom,(est.state[Y] - (FRAME_L/2.0)*eigenvector[1][Y])),
            W2DX(zoom,(est.state[X] + (FRAME_L/2.0)*eigenvector[1][X])),
            W2DY(zoom,(est.state[Y] + (FRAME_L/2.0)*eigenvector[1][Y])));
    dx0 = eigenvalue[0] * eigenvector[0][X];
    dy0 = eigenvalue[0] * eigenvector[0][Y];

    //printf("estimate cov:\n");
    //printf("\t%lf %lf\n\t%lf %lf\n", est.cov[0][0], est.cov[0][1], est.cov[1][0], est.cov[1][1]);

    for (theta = 0.0; theta <= 2*M_PI; theta += M_PI / 20.0) {
        dx1 = (eigenvalue[0]*cos(theta))*eigenvector[0][X] +
            (eigenvalue[1]*sin(theta))*eigenvector[1][X];
        dy1 = (eigenvalue[0]*cos(theta))*eigenvector[0][Y] +
            (eigenvalue[1]*sin(theta))*eigenvector[1][Y];
        XDrawLine(display, pixmap, gc, W2DX(zoom,(est.state[X] + dx0)),
                W2DY(zoom,(est.state[Y] + dy0)),
                W2DX(zoom,(est.state[X] + dx1)),
                W2DY(zoom,(est.state[Y] + dy1)));
        dx0 = dx1;
        dy0 = dy1;
    }
}

void draw_ellipse(est)
    Estimate est;
{
    double m[2][2], a, b, c, root[2];
    double dx, dy, mag, eigenvalue[2], eigenvector[2][2];
    double theta, dx0, dy0, dx1, dy1;

    //  printf("observation time = %lf  time = %lf\n", est.t, simtime);

    //if ((est.t == simtime)) {

    // DRAW THE CURRENT ESTIMATED STATE VARIABLES AND THEIR VELOCITIES
    draw_circle(W2DX(zoom,est.state[X]), W2DY(zoom,est.state[Y]),
            W2DR(zoom,R_JOINT), FILL);

    m[0][0] = est.cov[0][0];
    m[0][1] = est.cov[0][1];
    m[1][0] = est.cov[1][0];
    m[1][1] = est.cov[1][1];

    // cov = [A  B] => det |JJt| = a(lambda)^2 + b(lambda) +c
    //       [B  C]
    a = 1.0;
    b = -(m[0][0] + m[1][1]);
    c = m[0][0] * m[1][1] - m[1][0] * m[0][1];

    root[0] = (-b + sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);
    root[1] = (-b - sqrt(SQR(b) - 4.0 * a * c)) / (2.0 * a);

    // the eigenvector for root 0
    dy = 1.0;
    dx = -m[0][1] / (m[0][0] - root[0]);
    mag = sqrt(SQR(dx) + SQR(dy));
    eigenvalue[0] = sqrt(root[0]);
    eigenvector[0][0] = dx / mag;
    eigenvector[0][1] = dy / mag;

    // the eigenvector for root 1
    dy = 1.0;
    dx = -m[0][1] / (m[0][0] - root[1]);
    mag = sqrt(SQR(dx) + SQR(dy));
    eigenvalue[1] = sqrt(root[1]);
    eigenvector[1][0] = dx / mag;
    eigenvector[1][1] = dy / mag;

    dx0 = eigenvalue[0] * eigenvector[0][X];
    dy0 = eigenvalue[0] * eigenvector[0][Y];
    for (theta = M_PI / 20.0; theta < 2*M_PI; theta += M_PI / 20.0) {
        dx1 = (eigenvalue[0]*cos(theta))*eigenvector[0][X] +
            (eigenvalue[1]*sin(theta))*eigenvector[1][X];
        dy1 = (eigenvalue[0]*cos(theta))*eigenvector[0][Y] +
            (eigenvalue[1]*sin(theta))*eigenvector[1][Y];
        XDrawLine(display, pixmap, gc, W2DX(zoom,(est.state[X] + dx0)),
                W2DY(zoom,(est.state[Y] + dy0)),
                W2DX(zoom,(est.state[X] + dx1)),
                W2DY(zoom,(est.state[Y] + dy1)));
        dx0 = dx1;
        dy0 = dy1;
    }
    // }
}

typedef struct _visible_object {
  double dx, dy;
  double radius;
  int color;
} VisibleObject;

void insertion_sort(vob, sort, num)
VisibleObject *vob;
int *sort, num;
{
  int i,j, temp;

  for(i=0; i<num; i++) sort[i] = i;

  for (i=1; i<num; i++) {
    j = i - 1;
    temp = sort[i];
    while ((j>=0) && (sqrt(SQR(vob[sort[j]].dx) + SQR(vob[sort[j]].dy)) <=
		      sqrt(SQR(vob[temp].dx) + SQR(vob[temp].dy)))) {
      sort[j+1] = sort[j];
      j--;
    }
    sort[j+1] = temp;
  }
}

// NEEDS TO BE FIXED...DOESN'T SEEM TO ACCOUNT FOR EYE ANGLE
// x,y in eye coordinate frame base coordinates
void pinhole_camera(vob, i, rogerID)
VisibleObject vob;
int i;
int rogerID;
{
  int j, low_bin_index, high_bin_index;
  double phi, beta, alpha;

	if (rogerID == 1)
		phi = atan2(vob.dy, vob.dx) - eyes_1[i].theta; // eye frame heading eye->object
	else
		phi = atan2(vob.dy, vob.dx) - eyes_2[i].theta; // eye frame heading eye->object
  //  printf("      phi for eye %d = %6.4lf\n", i, phi);
  if (fabs(phi) < FOV) { /* feature projects onto image plane */
    alpha = atan2(vob.radius, sqrt(SQR(vob.dx)+SQR(vob.dy)));
    low_bin_index = (int) (NPIXELS / 2.0 * (1.0 + tan(phi - alpha)));
    low_bin_index = MAX(low_bin_index,0);
    high_bin_index = (int) (NPIXELS / 2.0 * (1.0 + tan(phi + alpha)));
    high_bin_index = MIN(high_bin_index,(NPIXELS-1));
    for (j = low_bin_index; j <= high_bin_index; j++) {
		if (rogerID == 1)
			eyes_1[i].image[j] = vob.color;
		else if (rogerID == 2)
			eyes_2[i].image[j] = vob.color;
    }
  }
  //  else {
  //     printf("FEATURE OUTSIDE FIELD OF VIEW \n");
  //  }
}

void make_images(rogerID)
int rogerID;
{
  int i, j, eye, o_index; // intensity[3];
  double x, y;
  double p_b[4], p_w[4], bTw[4][4];
  //  int feature_id,

  int sort[NBINS*NBINS];
  VisibleObject vobject[NBINS*NBINS];

  /* initialize image white */
  // make sure eye's images keep changing when roger is moving
	for (i = 0; i < NPIXELS; i++) {
		if (rogerID == 1) {
			eyes_1[LEFT].image[i] = eyes_1[RIGHT].image[i] = 100;
		}
		else if (rogerID == 2) {
			eyes_2[LEFT].image[i] = eyes_2[RIGHT].image[i] = 100;
		}
    }

  for (eye = LEFT; eye <= RIGHT; ++eye) {

	  //    first 3 elements in the range array are the hands and the ball
	  /* LEFT_ARM - in body frame */
	  if (rogerID == 1) {
		  sim_fwd_kinematics(LEFT, arms_1[LEFT][1].theta, arms_1[LEFT][2].theta, &x, &y);
		  // convert to eye coordinates
		  vobject[0].dx = x - eyes_1[eye].position[X];
		  vobject[0].dy = y - eyes_1[eye].position[Y];
		  vobject[0].radius = R_JOINT;
		  vobject[0].color = ARM_COLOR;
	  }
	  else if (rogerID == 2) {
		  sim_fwd_kinematics(LEFT, arms_2[LEFT][1].theta, arms_2[LEFT][2].theta, &x, &y);
		  // convert to eye coordinates
		  vobject[0].dx = x - eyes_2[eye].position[X];
		  vobject[0].dy = y - eyes_2[eye].position[Y];
		  vobject[0].radius = R_JOINT;
		  vobject[0].color = ARM_COLOR;
	  }

	  /* RIGHT_ARM - in body frame */
	  if (rogerID == 1) {
		  sim_fwd_kinematics(RIGHT,arms_1[RIGHT][1].theta,arms_1[RIGHT][2].theta,&x,&y);
		  vobject[1].dx = x - eyes_1[eye].position[X];
		  vobject[1].dy = y - eyes_1[eye].position[Y];
		  vobject[1].radius = R_JOINT;
		  vobject[1].color = ARM_COLOR;
	  }
	  else if (rogerID == 2) {
		  sim_fwd_kinematics(RIGHT,arms_2[RIGHT][1].theta,arms_2[RIGHT][2].theta,&x,&y);
		  vobject[1].dx = x - eyes_2[eye].position[X];
		  vobject[1].dy = y - eyes_2[eye].position[Y];
		  vobject[1].radius = R_JOINT;
		  vobject[1].color = ARM_COLOR;
	  }

    /* OBJECT */
	  if(rogerID == 1) {
    SIMinv_transform(mobile_base_1.wTb, bTw);
    p_w[0] = object.position[X]; p_w[1] = object.position[Y];
    p_w[2] = 0.0; p_w[3] = 1.0;
    SIMmatXvec(bTw, p_w, p_b);
    vobject[2].dx = p_b[X] - eyes_1[eye].position[X];
    vobject[2].dy = p_b[Y] - eyes_1[eye].position[Y];
    vobject[2].radius = R_OBJ;
    vobject[2].color = OBJECT_COLOR;
	  }
	  else if(rogerID == 2) {
		  SIMinv_transform(mobile_base_2.wTb, bTw);
		  p_w[0] = -object.position[X]; p_w[1] = object.position[Y];
		  p_w[2] = 0.0; p_w[3] = 1.0;
		  SIMmatXvec(bTw, p_w, p_b);
		  vobject[2].dx = p_b[X] - eyes_2[eye].position[X];
		  vobject[2].dy = p_b[Y] - eyes_2[eye].position[Y];
		  vobject[2].radius = R_OBJ;
		  vobject[2].color = OBJECT_COLOR;
	  }

    // after the first three, the rest are colored obstacles in the occupancy
    // grid
    o_index = 3; // points at the next empty element of the range array
    for (i=0;i<NBINS;++i) {
      y = MAX_Y - i*YDELTA;
      for (j=0;j<NBINS; ++j) {
	if (Roger_1.world_map.occupancy_map[i][j] == OBSTACLE) {
	  p_w[0] = MIN_X + j*XDELTA; p_w[1] = y;
	  p_w[2] = 0.0; p_w[3] = 1.0;
	  SIMmatXvec(bTw, p_w, p_b);
		if (rogerID == 1) {
			vobject[o_index].dx = p_b[X] - eyes_1[eye].position[X];
			vobject[o_index].dy = p_b[Y] - eyes_1[eye].position[Y];
			vobject[o_index].radius = R_OBSTACLE;
			vobject[o_index++].color = Roger_1.world_map.color_map[i][j];
		}
		else if (rogerID == 2) {
			vobject[o_index].dx = p_b[X] - eyes_2[eye].position[X];
			vobject[o_index].dy = p_b[Y] - eyes_2[eye].position[Y];
			vobject[o_index].radius = R_OBSTACLE;
			vobject[o_index++].color = Roger_2.world_map.color_map[i][j];   // !!!
		}

	}
      }
    }
    insertion_sort(vobject, sort, o_index);
    for (i=0; i<o_index; ++i)
      pinhole_camera(vobject[sort[i]], eye, rogerID);
  }
}

void initialize_simulator(rst)
int rst;
{
	int i,j,k,l;

	if (rst) {
		/************************************************************************/
		// MOBILE BASE
		for (i=0;i<4;++i){
			for (j=0;j<4;++j) {
				mobile_base_1.wTb[i][j] = mobile_base_home_1.wTb[i][j];
				mobile_base_2.wTb[i][j] = mobile_base_home_2.wTb[i][j];
			}
		}
		mobile_base_1.x = mobile_base_home_1.x;
		mobile_base_1.x_dot = mobile_base_home_1.x_dot;
		mobile_base_1.y = mobile_base_home_1.y;
		mobile_base_1.y_dot = mobile_base_home_1.y_dot;
		mobile_base_1.theta = mobile_base_home_1.theta;
		mobile_base_1.theta_dot = mobile_base_home_1.theta_dot;

		mobile_base_2.x = mobile_base_home_2.x;
		mobile_base_2.x_dot = mobile_base_home_2.x_dot;
		mobile_base_2.y = mobile_base_home_2.y;
		mobile_base_2.y_dot = mobile_base_home_2.y_dot;
		mobile_base_2.theta = mobile_base_home_2.theta;
		mobile_base_2.theta_dot = mobile_base_home_2.theta_dot;

		for (i=0;i<2;++i) {
			mobile_base_1.wheel_torque[i] = mobile_base_home_1.wheel_torque[i];
			mobile_base_2.wheel_torque[i] = mobile_base_home_2.wheel_torque[i];
		}
		mobile_base_1.contact_theta = mobile_base_home_1.contact_theta;
		mobile_base_2.contact_theta = mobile_base_home_2.contact_theta;
		for (i=0;i<2;++i) {
			mobile_base_1.extForce[i] = mobile_base_home_1.extForce[i];
			mobile_base_2.extForce[i] = mobile_base_home_2.extForce[i];
		}

		/************************************************************************/
		// LEFT AND RIGHT EYE
		for (i = 0; i < NPIXELS; ++i) {
			eyes_1[LEFT].image[i] = eyes_1[RIGHT].image[i] = 99;
			eyes_2[LEFT].image[i] = eyes_2[RIGHT].image[i] = 99;
		}

		// initialize LEFT eye
		eyes_1[LEFT].position[X] = 0.0;
		eyes_1[LEFT].position[Y] = BASELINE;
		eyes_1[LEFT].theta = 0.0;
		eyes_1[LEFT].theta_dot = 0.0;
		eyes_1[LEFT].torque = 0.0;

		eyes_2[LEFT].position[X] = 0.0;
		eyes_2[LEFT].position[Y] = BASELINE;
		eyes_2[LEFT].theta = 0.0;
		eyes_2[LEFT].theta_dot = 0.0;
		eyes_2[LEFT].torque = 0.0;

		// RIGHT eye
		eyes_1[RIGHT].position[X] = 0.0;
		eyes_1[RIGHT].position[Y] = -BASELINE;
		eyes_1[RIGHT].theta = 0.0;
		eyes_1[RIGHT].theta_dot = 0.0;
		eyes_1[RIGHT].torque = 0.0;

		eyes_2[RIGHT].position[X] = 0.0;
		eyes_2[RIGHT].position[Y] = -BASELINE;
		eyes_2[RIGHT].theta = 0.0;
		eyes_2[RIGHT].theta_dot = 0.0;
		eyes_2[RIGHT].torque = 0.0;

		/************************************************************************/
		// LEFT AND RIGHT ARM
		for (i=0;i<NARMS;++i) {
			for (j=0;j<NARM_FRAMES;++j) {
				for (k=0;k<4;++k) {
					for (l=0;l<4;++l) {
						arms_1[i][j].iTj[k][l] = arms_home_1[i][j].iTj[k][l];
						arms_2[i][j].iTj[k][l] = arms_home_2[i][j].iTj[k][l];
					}
				}
				arms_1[i][j].dof_type = arms_home_1[i][j].dof_type;
				arms_1[i][j].axis = arms_home_1[i][j].axis;
				arms_1[i][j].theta = arms_home_1[i][j].theta;
				arms_1[i][j].theta_dot = arms_home_1[i][j].theta_dot;
				arms_1[i][j].torque = arms_home_1[i][j].torque;
				arms_1[i][j].extForce[0] = arms_home_1[i][j].extForce[0];
				arms_1[i][j].extForce[1] = arms_home_1[i][j].extForce[1];

				arms_2[i][j].dof_type = arms_home_2[i][j].dof_type;
				arms_2[i][j].axis = arms_home_2[i][j].axis;
				arms_2[i][j].theta = arms_home_2[i][j].theta;
				arms_2[i][j].theta_dot = arms_home_2[i][j].theta_dot;
				arms_2[i][j].torque = arms_home_2[i][j].torque;
				arms_2[i][j].extForce[0] = arms_home_2[i][j].extForce[0];
				arms_2[i][j].extForce[1] = arms_home_2[i][j].extForce[1];

			}
		}

		/************************************************************************/
		// INITIALIZE THE WORLD GEOMETRY
		for (i = 0; i < NBINS; ++i) {   // rows
			for (j = 0; j < NBINS; ++j) {   // cols
				if (reset) {
					Roger_1.world_map.occupancy_map[i][j] = FREESPACE;
					Roger_1.world_map.potential_map[i][j] = 1.0;
					Roger_1.arm_map[LEFT].occupancy_map[i][j] = FREESPACE;
					Roger_1.arm_map[LEFT].potential_map[i][j] = 1.0;
					Roger_1.arm_map[RIGHT].occupancy_map[i][j] = FREESPACE;
					Roger_1.arm_map[RIGHT].potential_map[i][j] = 1.0;

					Roger_2.world_map.occupancy_map[i][j] = FREESPACE;
					Roger_2.world_map.potential_map[i][j] = 1.0;
					Roger_2.arm_map[LEFT].occupancy_map[i][j] = FREESPACE;
					Roger_2.arm_map[LEFT].potential_map[i][j] = 1.0;
					Roger_2.arm_map[RIGHT].occupancy_map[i][j] = FREESPACE;
					Roger_2.arm_map[RIGHT].potential_map[i][j] = 1.0;
				}

				// left and right walls
				if ((j <= 0) || (j >= (NBINS - 1))) {
					Roger_1.world_map.occupancy_map[i][j] = OBSTACLE;
					Roger_1.world_map.potential_map[i][j] = 1.0;
					Roger_1.world_map.color_map[i][j] = LIGHTGREEN;

					Roger_2.world_map.occupancy_map[i][j] = OBSTACLE;
					Roger_2.world_map.potential_map[i][j] = 1.0;
					Roger_2.world_map.color_map[i][j] = LIGHTGREEN;
				}
				// top and bottom walls
				if ((i <= 0) || (i >= (NBINS - 1))) {
					Roger_1.world_map.occupancy_map[i][j] = OBSTACLE;
					Roger_1.world_map.potential_map[i][j] = 1.0;
					Roger_1.world_map.color_map[i][j] = DARKBLUE;

					Roger_2.world_map.occupancy_map[i][j] = OBSTACLE;
					Roger_2.world_map.potential_map[i][j] = 1.0;
					Roger_2.world_map.color_map[i][j] = DARKBLUE;
				}
			}
		}
		Roger_1.simtime = 0.0;
		Roger_2.simtime = 0.0;

		//SocketCommunicate(&Roger_1, sockfd1);
		//if (numOfRoger == 2)
		//  SocketCommunicate(&Roger_2, sockfd2);

		//     // ***** the three room geometry *****
		//     // add room partitions to Cartesian map - horizonal wall
		//     for (j=(NBINS/2-NBINS/3); j < (NBINS/2+NBINS/3); ++j) {
		//        for (i=(NBINS/2-1); i<(NBINS/2+1); ++i) {
		//          Roger.world_map.occupancy_map[i][j] = OBSTACLE;
		//          Roger.world_map.potential_map[i][j] = 1.0;
		//          if (rst) add_bin_bumper(i,j);
		//        }
		//      }
		//	// vertical wall
		//      for (j=(NBINS/2-1); j<(NBINS/2+1); ++j) {
		//        for (i=(NBINS/2+1); i<(NBINS); ++i) {
		//          Roger.world_map.occupancy_map[i][j] = OBSTACLE;
		//          Roger.world_map.potential_map[i][j] = 1.0;
		//          if (rst) add_bin_bumper(i,j);
		//        }
		//      }
		// // **********************************************
		//  // ***** the soccer goal geometry *****
		//  i=56;
		//  for (j=0; j<22; ++j) {
		//    Roger.world_map.occupancy_map[i][j] =
		//      Roger.world_map.occupancy_map[i][j+42] = OBSTACLE;
		//    Roger.world_map.potential_map[i][j] =
		//      Roger.world_map.potential_map[i][j+42] = 1.0;
		//    Roger.world_map.color_map[i][j] =
		//      Roger.world_map.color_map[i][j+42] = obstacle_color[4];
		//
		//  }
		//
		//  for (i=56;i<NBINS;++i) {
		//    Roger.world_map.occupancy_map[i][21] =
		//      Roger.world_map.occupancy_map[i][42] = OBSTACLE;
		//    Roger.world_map.potential_map[i][21] =
		//      Roger.world_map.potential_map[i][42] = 1.0;
		//    Roger.world_map.color_map[i][21] =
		//      Roger.world_map.color_map[i][42] = obstacle_color[4];
		//  }
		//  // ****************************************************

		// in the new code organization, this is the user responsibility
		//  if (ARM_CSPACE_MAP) update_cspace_arms(&Roger);
		//  //  sor(&Roger);

		/************************************************************************/
		// initialize the volatile elements (afferents and efferents)
		// of the applications interface for user control code
		write_interface(rst);
	}
}

void new_game() {
	reset=TRUE;
	initialize_simulator(reset); // initializes world boundaries,
	// mobile_base, eyes[2], arms[2], and
	// Roger interface structure

	initialize_control_mode(&Roger_1);
	initialize_control_mode(&Roger_2);

	place_object(WIDTH/2, HEIGHT/2);
	initialize_pong_object();

	possessionTime_1 = 0;
	possessionTime_2 = 0;
	goal_1 = 0;
	goal_2 = 0;

	simulate_base(&mobile_base_1);
	simulate_arm(mobile_base_1.wTb, arms_1);
	simulate_eyes(mobile_base_1.wTb, eyes_1);

	simulate_base(&mobile_base_2);
	simulate_arm(mobile_base_2.wTb, arms_2);
	simulate_eyes(mobile_base_2.wTb, eyes_2);

	simulate_object(&object);

	make_images(1);
	make_images(2);
	draw_all();
}

void new_serve() {
	reset=TRUE;
	initialize_simulator(reset); // initializes world boundaries,
	Roger_1.is_new_serve = 1;
	Roger_2.is_new_serve = 1;
	// mobile_base, eyes[2], arms[2], and
	// Roger interface structure

	initialize_control_mode(&Roger_1);
	initialize_control_mode(&Roger_2);

	place_object(WIDTH/2, HEIGHT/2);
	initialize_pong_object();

	possessionTime_1 = 0;
	possessionTime_2 = 0;
	//goal_1 = 0;
	//goal_2 = 0;

	simulate_base(&mobile_base_1);
	simulate_arm(mobile_base_1.wTb, arms_1);
	simulate_eyes(mobile_base_1.wTb, eyes_1);

	simulate_base(&mobile_base_2);
	simulate_arm(mobile_base_2.wTb, arms_2);
	simulate_eyes(mobile_base_2.wTb, eyes_2);

	simulate_object(&object);

	make_images(1);
	make_images(2);
	draw_all();
}

void goal_detection()
{
	static int ball_which_side = 1;
	double r[2];
	r[0] = object.position[0];
	r[1] = object.position[1];
	Roger_1.is_new_serve = 0;
	Roger_2.is_new_serve = 0;

	int middle = MIN_X + (MAX_X-MIN_X) / 2;
	if (r[0] > middle)
	{
		possessionTime_2 += DT;
		if (ball_which_side == 2)
		{
			continuous_possesionTime_2 += DT;
			continuous_possesionTime_1 = 0;
		}
		else
			continuous_possesionTime_2 = 0;
	}
	else if (r[0] < middle)
	{
		possessionTime_1 += DT;
		if (ball_which_side == 1)
		{
			continuous_possesionTime_1 += DT;
			continuous_possesionTime_2 = 0;
		}
		else
			continuous_possesionTime_1 = 0;
	}

	if (r[0] > middle)
		ball_which_side = 2;
	else
		ball_which_side = 1;

	double maxPossesionTime = 5.0;
	if (continuous_possesionTime_1 + continuous_possesionTime_2 > maxPossesionTime)
	{
		if (continuous_possesionTime_1 < continuous_possesionTime_2)
		{
			goal_1++;
			continuous_possesionTime_1 = 0;
			continuous_possesionTime_2 = 0;
			possessionTime_1 = 0;
			possessionTime_2 = 0;
			usleep(1000000);
			new_serve();
		}
		else {
			goal_2 ++;
			continuous_possesionTime_1 = 0;
			continuous_possesionTime_2 = 0;
			possessionTime_1 = 0;
			possessionTime_2 = 0;
			usleep(1000000);
			new_serve();
		}
		return;
	}

	double maxTime = 20.0;
	if (possessionTime_1 + possessionTime_2 > maxTime) {
		if (possessionTime_1 < possessionTime_2) {
			goal_1 ++;
			continuous_possesionTime_1 = 0;
			continuous_possesionTime_2 = 0;
			possessionTime_1 = 0;
			possessionTime_2 = 0;
			usleep(1000000);
			new_serve();
		}
		else {
			goal_2 ++;
			continuous_possesionTime_1 = 0;
			continuous_possesionTime_2 = 0;
			possessionTime_1 = 0;
			possessionTime_2 = 0;
			usleep(1000000);
			new_serve();
		}
	}
	else {
		if (r[0] >= MAX_X - (R_GOAL)) {
			goal_1 ++;
			continuous_possesionTime_1 = 0;
			continuous_possesionTime_2 = 0;
			possessionTime_1 = 0;
			possessionTime_2 = 0;
			usleep(1000000);
			new_serve();
		}
		else if (r[0] <= MIN_X + (R_GOAL)) {
			goal_2 ++;
			continuous_possesionTime_1 = 0;
			continuous_possesionTime_2 = 0;
			possessionTime_1 = 0;
			possessionTime_2 = 0;
			usleep(1000000);
			new_serve();
		}
	}


	/*  char *temp, buf[20];

	 if (object.position[Y] < (MIN_Y + R_GOAL)) {
	 printf("Scored at: %f\n",simtime);

	 temp = gcvt(simtime,5,buf);
	 printf("temp=%s\n",temp);

	 XSetForeground(display, gc, foreground);
	 XDrawString(display, pixmap, gc,
	 W2DX(zoom,3.0), W2DY(zoom,1.8), temp, 5);
	 XDrawString(display, pixmap, gc, T12RD(-0.45), T22RD(-3.6), "time=", 5);

	 XkwSetWidgetLabel(start_w, "Start");
	 XtRemoveTimeOut(timer);
	 timer = 0;
	 }
	 */
}

int fault_detection()
{
	Roger_1.is_new_serve = 0;
	Roger_2.is_new_serve = 0;
	if (touchWall_2) {
		printf("Roger 2 touched wall\n");
		goal_1 ++;
		continuous_possesionTime_1 = 0;
		continuous_possesionTime_2 = 0;
		possessionTime_1 = 0;
		possessionTime_2 = 0;
		usleep(1000000);
		new_serve();
		return 1;
	}
	else if (touchWall_1) {
		printf("Roger 1 touched wall\n");
		goal_2 ++;
		continuous_possesionTime_1 = 0;
		continuous_possesionTime_2 = 0;
		possessionTime_1 = 0;
		possessionTime_2 = 0;
		usleep(1000000);
		new_serve();
		return 1;
	}

	double x, y, p_b[4], p_wl[4], p_wr[4];

	sim_fwd_kinematics(LEFT, arms_1[LEFT][1].theta, arms_1[LEFT][2].theta, &x, &y);
	p_b[0] = x; p_b[1] = y; p_b[2] = 0.0; p_b[3] = 1.0;
	SIMmatXvec(mobile_base_1.wTb, p_b, p_wl);
	sim_fwd_kinematics(RIGHT, arms_1[RIGHT][1].theta, arms_1[RIGHT][2].theta, &x, &y);
	p_b[0] = x; p_b[1] = y; p_b[2] = 0.0; p_b[3] = 1.0;
	SIMmatXvec(mobile_base_1.wTb, p_b, p_wr);
	//printf("Roger 1 : %lf,%lf,%lf\n",Roger_1.base_position[0],p_wl[X],p_wr[X]);
	if (Roger_1.base_position[0] > 0-R_BASE || p_wl[X] > 0-R_TACTILE || p_wr[X] > 0-R_TACTILE) {
		printf("Roger 1 crossed center \n");
		goal_2 ++;
		continuous_possesionTime_1 = 0;
		continuous_possesionTime_2 = 0;
		possessionTime_1 = 0;
		possessionTime_2 = 0;
		usleep(1000000);
		new_serve();
		return 1;
	}


	sim_fwd_kinematics(LEFT, arms_2[LEFT][1].theta, arms_2[LEFT][2].theta, &x, &y);
	p_b[0] = x; p_b[1] = y; p_b[2] = 0.0; p_b[3] = 1.0;
	SIMmatXvec(mobile_base_2.wTb, p_b, p_wl);
	sim_fwd_kinematics(RIGHT, arms_2[RIGHT][1].theta, arms_2[RIGHT][2].theta, &x, &y);
	p_b[0] = x; p_b[1] = y; p_b[2] = 0.0; p_b[3] = 1.0;
	SIMmatXvec(mobile_base_2.wTb, p_b, p_wr);
	//printf("Roger 2 : %lf,%lf,%lf\n",Roger_2.base_position[0],p_wl[X],p_wr[X]);
	if (Roger_2.base_position[0] > 0-R_BASE || p_wl[X] > 0-R_TACTILE || p_wr[X] > 0-R_TACTILE) {
		printf("Roger 2 crossed center\n");
		goal_1 ++;
		continuous_possesionTime_1 = 0;
		continuous_possesionTime_2 = 0;
		possessionTime_1 = 0;
		possessionTime_2 = 0;
		usleep(1000000);
		new_serve();
		return 1;
	}

	return 0;
}

void x_timer_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  int i, j, reset;
  static int render = RENDER_RATE, servo = SERVO_RATE, image = IMAGE_RATE;
  void goal_detection();
  //  static double simtime=0.0;

  if (servo++ == SERVO_RATE) {

    reset=FALSE;
    write_interface(reset); // reset eliminates user goals and obstacles
    // from occupancy grids

	  if (Roger_1.input_mode == 3 ) {
		  if(Roger_1.button_event) {
			  place_object(Roger_1.button_reference[X], Roger_1.button_reference[Y]);
			  Roger_1.button_event = FALSE;
		  }
	  }

	  if (Roger_2.input_mode == 3 ) {
		  if(Roger_2.button_event) {
			  place_object(Roger_2.button_reference[X], Roger_2.button_reference[Y]);
			  Roger_2.button_event = FALSE;
		  }
	  }

   // control_roger(&Roger, simtime);

	  double middle = MIN_X+(MAX_X-MIN_X)/2;
	  double rogerX;
	  double dx = rogerX - middle;

	  if (socknew1 > 0)
		  SocketCommunicate(&Roger_1, socknew1);
	  if (socknew2 > 0)
		  SocketCommunicate(&Roger_2, socknew2);

	  motor_model(&Roger_1);
	  motor_model(&Roger_2);
    read_interface();
    servo = 1;
  }

  /* writes collision forces into respective data structures */
  /* obstacle data structure is global */
	Obj obj1 = object;
	Obj obj2 = object;
    compute_external_forces_1(&mobile_base_1, arms_1, &obj1);
	compute_external_forces_2(&mobile_base_2, arms_2, &obj2);
	object.extForce[X] = obj1.extForce[X] - obj2.extForce[X];
	object.extForce[Y] = obj1.extForce[Y] + obj2.extForce[Y];

	simulate_base(&mobile_base_1);
	simulate_arm(mobile_base_1.wTb, arms_1);
	simulate_eyes(mobile_base_1.wTb, eyes_1);
	//simulate_object(&object);

	simulate_base(&mobile_base_2);
	simulate_arm(mobile_base_2.wTb, arms_2);
	simulate_eyes(mobile_base_2.wTb, eyes_2);

	simulate_object(&object);

	if(object.position[1] > MAX_Y + 0.2) {
		object.position[1] = MAX_Y;
		object.velocity[1] = -object.velocity[1];
	}
	else if(object.position[1] < MIN_Y - 0.2) {
		object.position[1] = MIN_Y;
		object.velocity[1] = -object.velocity[1];
	}

	if(image++ == IMAGE_RATE)
	{
	make_images(1);
	make_images(2);
		image = 1;
	}

	if (render++ == RENDER_RATE) {
		if ((HISTORY) && (history_ptr_1 < MAX_HISTORY)) {
			history_1[history_ptr_1].arm_pos[LEFT][0] = arms_1[LEFT][1].theta;
			history_1[history_ptr_1].arm_pos[LEFT][1] = arms_1[LEFT][2].theta;

			history_1[history_ptr_1].arm_pos[RIGHT][0] = arms_1[RIGHT][1].theta;
			history_1[history_ptr_1].arm_pos[RIGHT][1] = arms_1[RIGHT][2].theta;

			history_1[history_ptr_1].base_pos[0] = mobile_base_1.x;
			history_1[history_ptr_1].base_pos[1] = mobile_base_1.y;
			history_1[history_ptr_1].base_pos[2] = mobile_base_1.theta;

			++history_ptr_1;
		}

		if ((HISTORY) && (history_ptr_2 < MAX_HISTORY)) {
			history_2[history_ptr_2].arm_pos[LEFT][0] = arms_2[LEFT][1].theta;
			history_2[history_ptr_2].arm_pos[LEFT][1] = arms_2[LEFT][2].theta;

			history_2[history_ptr_2].arm_pos[RIGHT][0] = arms_2[RIGHT][1].theta;
			history_2[history_ptr_2].arm_pos[RIGHT][1] = arms_2[RIGHT][2].theta;

			history_2[history_ptr_2].base_pos[0] = mobile_base_2.x;
			history_2[history_ptr_2].base_pos[1] = mobile_base_2.y;
			history_2[history_ptr_2].base_pos[2] = mobile_base_2.theta;

			++history_ptr_2;
		}

		draw_all();
		render = 1;
	}
	simtime += DT;

	timer = XtAppAddTimeOut(app_con, TIMER_UPDATE, x_timer_proc,
							(XtPointer) NULL);


	int fault = fault_detection();
	if (!fault)
		goal_detection();

	game_over_detection();

}

void x_pause_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
	new_game();
}

void x_start_proc(w, client_data, call_data)
Widget w;
XtPointer client_data, call_data;
{
  if (!timer) {
    XkwSetWidgetLabel(start_w, "Stop");
    timer = XtAppAddTimeOut(app_con, TIMER_UPDATE, x_timer_proc,
			    (XtPointer) NULL);
	  new_game();
    //    initialize_random_object();
  }
  else {
    XkwSetWidgetLabel(start_w, "New Game");
    XtRemoveTimeOut(timer);
    timer = 0;
  }
}

int main(argc, argv)
int argc;char **argv;
{
	if (argc != 2) {
		printf("Please indicate the number of rogers.\n");
		return 0;
	}

	numRoger = atoi(argv[1]);

	SocketInit(SERVERPORT_1, &socknew1);
	if (numRoger == 2)
		SocketInit(SERVERPORT_2, &socknew2);
    //printf("%d, %d\n", socknew1, socknew2);


  int i;
  static String fallback_resources[] = {
    "*title:	Roger-the-Crab",
    "*Roger-the-Crab*x:	100",
    "*Roger-the-Crab*y:	100",
    NULL,
  };
  Widget toplevel, form, widget;
  void x_clear();

  toplevel = XtAppInitialize(&app_con, "Roger-the-Crab", NULL, ZERO, &argc,
			     argv, fallback_resources, NULL, ZERO);
  form = XkwMakeForm(toplevel);
  widget = NULL;
  start_w = widget = XkwMakeCommand(form, NULL, widget, x_start_proc,
				    "New Game", BOXW, BOXH);
  //input_mode_1_w = widget = XkwMakeCommand(form, NULL, widget, x_input_mode_1_proc,
	//				 "Input 1: Joint angles", BOXW, BOXH);
  control_mode_1_w = widget = XkwMakeCommand(form, NULL, widget,
					   x_control_mode_1_proc,
					   "Control 1: ChasePunch", BOXW, BOXH);
  //input_mode_2_w = widget = XkwMakeCommand(form, NULL, widget, x_input_mode_2_proc,
	//										 "Input 2: Joint angles", BOXW, BOXH);
  control_mode_2_w = widget = XkwMakeCommand(form, NULL, widget,
											   x_control_mode_2_proc,
											   "Control 2: ChasePunch", BOXW, BOXH);
 // params_w = widget = XkwMakeCommand(form, NULL, widget, x_params_proc,
	//			     "Enter Params", BOXW, BOXH);
  //stream_w = widget = XkwMakeCommand(form, NULL, widget, x_visualize_proc,
	//			     "Visualize",  BOXW, BOXH);
  widget = XkwMakeCommand(form, NULL, widget, x_quit_proc,
			  "Quit", BOXW,	BOXH);
  canvas_w = widget = XkwMakeCanvas(form, widget, NULL,
				    x_canvas_proc, width, height);
  XtRealizeWidget(toplevel);
  display = XtDisplay(canvas_w);
  window = XtWindow(canvas_w);
  screen = DefaultScreen(display);
  depth = DefaultDepth(display, screen);
  foreground = BlackPixel(display, screen);
  background = WhitePixel(display, screen);

  gc = XCreateGC(display, window, 0, NULL);
  XSetFunction(display, gc, GXcopy);
  XSetForeground(display, gc, foreground);
  XSetBackground(display, gc, background);

  pixmap = XCreatePixmap(display, window, width, height, depth);
  x_clear();

  x_init_colors();

  reset=TRUE;
  initialize_simulator(reset); // initializes world boundaries,
  // mobile_base, eyes[2], arms[2], and
  // Roger interface structure

   place_object(WIDTH/2, HEIGHT/2);
	
	simulate_base(&mobile_base_1);
	simulate_arm(mobile_base_1.wTb, arms_1);
	simulate_eyes(mobile_base_1.wTb, eyes_1);
	
	simulate_base(&mobile_base_2);
	simulate_arm(mobile_base_2.wTb, arms_2);
	simulate_eyes(mobile_base_2.wTb, eyes_2);

	simulate_object(&object);
	
	make_images(1);
	make_images(2);
	draw_all();
	
  XtAppMainLoop(app_con);
}

/*
void goal_detection()
{
  char *temp, buf[20];
	
  if (object.position[Y] < (MIN_Y + R_GOAL)) {
    printf("Scored at: %f\n",simtime);
		
    temp = gcvt(simtime,5,buf);
    printf("temp=%s\n",temp);
		
    XSetForeground(display, gc, foreground);
    XDrawString(display, pixmap, gc,
		W2DX(zoom,3.0), W2DY(zoom,1.8), temp, 5);
    XDrawString(display, pixmap, gc, T12RD(-0.45), T22RD(-3.6), "time=", 5);
    
    XkwSetWidgetLabel(start_w, "Start");
    XtRemoveTimeOut(timer);
    timer = 0;
  }
}
*/

