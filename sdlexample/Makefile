#
# Felipe Kuhne
# fkuhne at gmail dot com
#

default: all

CPP = g++

CFLAGS = -Wall $(shell sdl2-config --cflags)
LDFLAGS = $(shell sdl2-config --libs)

SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)
EXEC = sdlExample

all: $(SRCS) $(EXEC)

%.o: %.cpp
	$(CPP) $(CFLAGS) -c -o $@ $<

$(EXEC): $(OBJS)
	$(CPP) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf $(OBJS) $(EXEC)
