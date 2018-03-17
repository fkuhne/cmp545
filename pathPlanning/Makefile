#
# Felipe Kuhne
# fkuhne at gmail dot com
#

default: all

CPP = g++

CFLAGS = $(shell sdl2-config --cflags) -I../LMPL -I../LMPL/misc -Imisc -I.
LDFLAGS = $(shell sdl2-config --libs) -L/usr/local/lib -lSDL2_ttf -lm -L../LMPL -L../LMPL/MotionPlanning/objs

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o) misc/objs/AABB2D.o misc/objs/Primitives.o misc/objs/Line2D.o misc/objs/Sample.o misc/objs/Clip.o misc/objs/Segment2D.o misc/objs/Plane2D.o
EXEC = pathPlanning

all: $(SRCS) $(EXEC)

%.o: %.cpp
	make -C misc
	$(CPP) $(CFLAGS) -c -o $@ $<

$(EXEC): $(OBJS)
	$(CPP) $(CFLAGS) -o $@ $^ ../LMPL/LMPL.a $(LDFLAGS)

clean:
	make -C misc clean
	rm -rf $(OBJS) $(EXEC)
