#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h> 
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "Roger.h"
#include "simulate.h"
#include "control.h"

#define MAXDATASIZE 600000
Robot Roger;
void control_roger();
struct hostent;
struct hostent *gethostbyname();

char bufData[MAXDATASIZE];
void error(const char *msg)
{
    perror(msg);
    exit(0);
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
    //printf("%d\n", size);

    //printf("send roger\n");
    if (send(sock, roger, size, 0) == -1) {
        perror("send error");
        return 1;
    }

    // int rst = SocketRecv(bufData, size, sock);
    //if (rst == 1)
    //    printf("received\n");
    //else
    //    printf("receive error\n");

    //memcpy (roger, bufData, size);

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

/* place_object(x,y)
double x,y;
{
	object.mass = 0.2;
	object.position[X] = x;
	object.position[Y] = y;
	object.velocity[X] = object.velocity[Y] = 0.0;
	object.extForce[X] = object.extForce[Y] = 0.0;
 
} */


int main(argc, argv)
int argc;char **argv; 
{
  int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];
	if (argc != 3) {
		printf("Please input IP and port number\n");
		return 0;
	}

	portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    //    bzero((char *) &serv_addr, sizeof(serv_addr));
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
    
    while(1)
      {
	int size = sizeof(Roger);
	//	   printf("%d\n", size);
	int rst = SocketRecv(bufData, size, sockfd);
/*    if (rst == 1)
        printf("received\n");
    else
        printf("receive error\n");
*/	
	memcpy (&Roger, bufData, size);
	int control_mode = Roger.control_mode;
	int input_mode = Roger.input_mode;
	//printf("Mode : %d\n",Roger.mode);

	control_roger(&Roger, 0);
	//printf("New mode : %d\n",Roger.mode);
	Roger.control_mode = control_mode;
	Roger.input_mode = input_mode;
	SocketCommunicate(&Roger, sockfd);
	//printf("Got data \n");
      }
    /*
    printf("Please enter the message: ");
    bzero(buffer,256);
    fgets(buffer,255,stdin);
    n = write(sockfd,buffer,strlen(buffer));
    if (n < 0) 
         error("ERROR writing to socket");
    bzero(buffer,256);
    n = read(sockfd,buffer,255);
    if (n < 0) 
         error("ERROR reading from socket");
    printf("%s\n",buffer);
    */
    close(sockfd);
    return 0;
}
