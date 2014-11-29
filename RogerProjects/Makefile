# replace this with the top of your X11 tree
# X11 = /exp/rcf/share/X11R5
X11 = /usr/X11R6

############## do not change below this line ####################

ROGERINCDIR = ./include

XINCDIR = $(X11)/include
XLIBDIR = $(X11)/lib 
EDLAB_XINCDIR = /usr/include/X11
EDLAB_XLIBDIR = /usr/lib/i386-linux-gnu

XAWLIB = -lXaw
XMULIB = -lXmu
XTOOLLIB = -lXt
XLIB = -lX11
XEXTLIB = -lXext
MATHLIB = -lm

LIBS =  -L$(XLIBDIR) $(XAWLIB) $(XMULIB) $(XTOOLLIB) $(XLIB) $(XEXTLIB) \
	$(MATHLIB)

RM = rm -f
CC = gcc
CCFLAGS = -c -g -I. -I$(XINCDIR) -I$(EDLAB_XINCDIR) -I$(ROGERINCDIR)

.SUFFIXES:	.c	.o

.c.o:	
	$(CC) $(CCFLAGS) $<

############## do not change above this line ####################

PROG = x

PROJECTOFILES = project1-MotorUnits/project1.o \
		project2-Kinematics/project2.o \
		project2-Kinematics/matrix_math.o \
                project2-Kinematics/vision.o \
		project3-SearchTrack/project3.o \
                project3-SearchTrack/sampling.o \
                project4-ChasePunch/project4.o \
		project5-Kalman/project5.o \
		project5-Kalman/kalman.o \
		project6-PathPlanning/project6.o \
		project7-Grasp/project7.o \
                project8-EnvModel/project8.o \
		project9-Belief/project9.o

#HFILES = Roger.h simulate.h $(ROGERINCDIR)/control.h

all:  subdirs  $(PROG)

subdirs:
	cd project1-MotorUnits; make; cd ..; \
	cd project2-Kinematics; make; cd ..; \
	cd project3-SearchTrack; make; cd ..; \
	cd project4-ChasePunch; make; cd ..;
	cd project5-Kalman; make; cd ..; \
	cd project6-PathPlanning; make; cd ..;
	cd project7-Grasp; make; cd ..;
	cd project8-EnvModel; make; cd ..; \
	cd project9-Belief; make; cd ..;

$(PROG): $(PROJECTOFILES) 
	$(CC) -o $@ lib/simulator.a $(LIBS) $^

clean:
	$(RM) $(PROJECTOFILES) $(PROG) *~


