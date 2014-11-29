# replace this with the top of your X11 tree
# X11 = /exp/rcf/share/X11R5
X11 = /usr/X11R6

############## do not change below this line ####################

ROGERINCDIR = ./include

XINCDIR = $(X11)/include
XLIBDIR = $(X11)/lib 

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
#CCFLAGS = -c -O $(OPT) -I. -I$(XINCDIR)
CCFLAGS = -c -g -I. -I$(XINCDIR) -I$(ROGERINCDIR)

.SUFFIXES:	.c	.o

.c.o:
	$(CC) $(CCFLAGS) $<
############## do not change above this line ####################

PROG = socketcomm

ROGERLIB = lib/SocketComm.a

OFILES1 = 4D_math.o \
         SocketComm.o

HFILES = Roger.h simulate.h $(ROGERINCDIR)/control.h


all:  $(OFILES1) $(ROGERLIB)

$(PROG):	$(OFILES1)
	$(CC) -o $@ $(OFILES1) $(XKWOFILES) $(PROOFILES) $(LIBS)

$(ROGERLIB):	$(OFILES1)
	ar r $(ROGERLIB) $(OFILES1)

clean:
	$(RM) $(OFILES1) $(XKWOFILES) $(PROG) *~

