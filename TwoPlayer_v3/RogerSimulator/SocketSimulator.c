#include <stdio.h>
#include <unistd.h>
#include <sys/types.h> 
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "Roger.h"
#include "simulate.h"
#include "control.h"

#define MAXDATASIZE 600000
#define MAXCONN_NUM 10

int numbytes;
char buf[MAXDATASIZE];
char bufData[MAXDATASIZE];

int SocketInit(int port, int* socknew)
{
  printf("Inside SocketInit() \n");
    int sockfd;
    
	struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    unsigned int sin_size;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket error\n");
	//printf("socket error\n");
        return 1;
    }

    int on = 1;
    //printf("Before setsockopt\n");
    int ret = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    //printf("After setsockopt \n");

    memset(&client_addr, 0, sizeof(struct sockaddr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sockfd, (struct sockaddr *) &server_addr, sizeof(struct sockaddr)) == -1) {
        perror("bind error\n");
	//printf("bind error\n");
        return 1;
    }
    if (listen(sockfd, MAXCONN_NUM) == -1) {
        perror("listen error\n");
	//printf("listen error\n");
        return 1;
    }
    //printf("Before accept \n");
	while (1) {
        sin_size = sizeof(struct sockaddr_in);
	//printf("Entering accept : %s \n",client_addr);
        if ((*socknew = accept(sockfd, (struct sockaddr *)&client_addr, &sin_size)) == -1) {
            perror("accept error\n");
	    //printf("accept error\n");
            continue;
        }
        else {
	  //printf("Accepted : %s \n",client_addr);
            printf("accepted\n");
	    //printf("server: got connection from %s\n",client_addr.sin_addr);
	    //            printf("server: got connection from %s\n", inet_ntoa(client_addr.sin_addr));  //doesn't work in MacOS
            break;
        }
    }
return 0;
}

int SocketRecv(char* msgbody, int length, int sock) {
    char buf[MAXDATASIZE];
	char *pt = msgbody;
	memset (buf, 0, length);

	int n = 0;
	int res = length;
	while(1) {
		int rc = recv(sock, buf, res, 0);
		if (rc < 0) {
			//usleep(100000);
			//this->close();   //close the socket when recv fails. Not standard but just to prevent forgotting to close
			return -1;
		}
		else if ( rc == 0 ){
			//this->close();   //close the socket when recv fails. Not standard but just to prevent forgotting to close
			return 0;
		}
		n += rc;
		res -= rc;
		//printf("%d  %d  %d\n", rc, n, res);
		memcpy (pt, buf, rc);
		pt += rc;
		if (n >= length)
			break;
	}
    return 1;
}

int SocketCommunicate(Robot* roger, int sock)
{
    int size = sizeof(*roger);
   // printf("%d\n", size);

    //printf("send roger\n");
    if (send(sock, roger, size, 0) == -1) {
        perror("send error");
        return 1;
    }

    int rst = SocketRecv(bufData, size, sock);
   /* if (rst == 1)
        printf("received\n");
    else
        printf("receive error\n");
*/
    memcpy (roger, bufData, size);

/*    if ((numbytes = recv(sockfd, buf, MAXDATASIZE, 0)) == -1) {
        perror("recv error");
        return 1;
    }
    if (numbytes) {
        buf[numbytes] = '\0';
        printf("received: %s\n", buf);
    }
*/
    return 0;
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
/*	
	//call the reset method of the last mode --> clear all thing you modified
	switch ((roger->control_mode + N_CONTROL_MODES - 1) % N_CONTROL_MODES) 
	// "+ N_CONTROL_MODES" needed as modulo of negative numbers incorrect
	{
		case PROJECT3: 
			project3_reset(roger); 
			break;
		case PROJECT4: 
			project4_reset(roger); 
			break;
		case PROJECT5: 
			 project5_reset(roger); 
			 break;
		case PROJECT6: 
			project6_reset(roger); 
			break;
		default:
			break;
	}*/
	//init_control_flag = FALSE;  
}


/*
void initialize_mode(roger)
Robot * roger;
{
  double wTb[4][4],ref_w[4],ref_b[4];
  int i,j;

  // define all setpoints to current positions and zero velocities
  roger->base_setpoint[X] = roger->base_position[X] + 
    BASE_CONTROL_OFFSET*cos(roger->base_position[2]);
  roger->base_setpoint[Y] = roger->base_position[Y] + 
    BASE_CONTROL_OFFSET*sin(roger->base_position[2]);

  roger->arm_setpoint[LEFT][0] = roger->arm_theta[LEFT][0];
  roger->arm_setpoint[LEFT][1] = roger->arm_theta[LEFT][1];
  roger->arm_setpoint[RIGHT][0] = roger->arm_theta[RIGHT][0];
  roger->arm_setpoint[RIGHT][1] = roger->arm_theta[RIGHT][1];

  roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT];
  roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT];

  // make sure that mode changes don't refer to obsolete button inputs
  if (roger->mode == 0) { // mobile base teleop input
    roger->button_reference[X] = roger->base_setpoint[X];
    roger->button_reference[Y] = roger->base_setpoint[Y];
  } 
  else if (roger->mode == 1) { // left arm teleop input
    construct_wTb(roger->base_position, wTb);

    ref_b[0] = L1*cos(roger->arm_setpoint[LEFT][0]) + 
      L2*cos(roger->arm_setpoint[LEFT][0]+roger->arm_setpoint[LEFT][1]);
    ref_b[1] = L1*sin(roger->arm_setpoint[LEFT][0]) + 
      L2*sin(roger->arm_setpoint[LEFT][0]+roger->arm_setpoint[LEFT][1]) + 
      ARM_OFFSET;
    ref_b[2] = 0.0;
    ref_b[3] = 1.0;

    matXvec(wTb, ref_b, ref_w);
    roger->button_reference[X] = ref_w[0];
    roger->button_reference[Y] = ref_w[1];

  } 
  else if (roger->mode == 2) { // right arm teleop input
    construct_wTb(roger->base_position, wTb);

    ref_b[0] = L1*cos(roger->arm_setpoint[RIGHT][0]) + 
               L2*cos(roger->arm_setpoint[RIGHT][0] +
                      roger->arm_setpoint[RIGHT][1]);
    ref_b[1] = L1*sin(roger->arm_setpoint[RIGHT][0]) + 
                  L2*sin(roger->arm_setpoint[RIGHT][0] +
                         roger->arm_setpoint[RIGHT][1]) - ARM_OFFSET;
    ref_b[2] = 0.0;
    ref_b[3] = 1.0;

    matXvec(wTb, ref_b, ref_w);
    roger->button_reference[X] = ref_w[0];
    roger->button_reference[Y] = ref_w[1];
  } 
  else if (roger->mode == 3) { // stereo teleop
    construct_wTb(roger->base_position, wTb);

    // place a virtual cycloptic reference at a distance of 10 m
    // along the average gaze direction
    ref_b[0] = 10.0*cos((roger->eyes_setpoint[LEFT] +
                         roger->eyes_setpoint[RIGHT])/2.0);
    ref_b[1] = 10.0*sin((roger->eyes_setpoint[LEFT] +
                         roger->eyes_setpoint[RIGHT])/2.0);
    ref_b[2] = 0.0;
    ref_b[3] = 1.0;

    matXvec(wTb, ref_b, ref_w);
    roger->button_reference[X] = ref_w[0];
    roger->button_reference[Y] = ref_w[1];
  }
  else if (roger->mode == 4) { // Harmonic Function environment input
    roger->button_reference[X] = roger->base_setpoint[X];
    roger->button_reference[Y] = roger->base_setpoint[Y];
  }
  else if (roger->mode == 5) { 
    // Student's integrated APP
    // reset the occupancy grid left over from previous modes

    // THIS IS NOT THE CORRECT WAY TO RESET THE OCCUPANCY GRID

    for (i=0; i<NBINS; ++i) {
      for (j=0; j<NBINS; ++j) {
	  roger->world_map.occupancy_map[i][j] = FREESPACE;
	  roger->world_map.potential_map[i][j] = 1.0;
      }
    }
  }
}

*/
