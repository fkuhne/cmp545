CC = g++

CFLAGS  = -O2 -I. -I../../include
LDFLAGS	= -L. -L../../lib
LDLIBS  = -lPQP -lm      

.SUFFIXES: .cpp

SRCS    = main.cpp

OBJECTS	= main.o

CLEAN   = $(OBJECTS) sample

.cpp.o:
	$(CC) ${CFLAGS} -c $<

default: $(OBJECTS)
	$(CC) $(CFLAGS) -o sample $(OBJECTS) -L. $(LDFLAGS) $(LDLIBS)

clean:
	/bin/rm -f $(CLEAN)

