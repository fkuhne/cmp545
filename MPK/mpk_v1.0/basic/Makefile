include ../Make.config

LIB =	../lib/libbasic.a

SRC =	mpkBaseRobot.cpp \
	mpkCollPair.cpp \
	mpkIncludeFile.cpp \
	mpkObstacle.cpp \
	mpkObstacleCollection.cpp \
	mpkPathSmoother.cpp \
	mpkRobot.cpp \
	mpkRobotCollection.cpp \
	mpkCollDistAlgo.cpp \
	mpkConfigChecker.cpp \
	mpkSimpleSegmentChecker.cpp \
	mpkAdaptSegmentChecker.cpp

OBJ =	$(SRC:.cpp=.o)

.SUFFIXES : .cpp .H

INCPATH = -I. -I../robots $(INCALL)

$(LIB):	$(OBJ)
	ar ruv $(LIB) $(OBJ)
	ranlib $(LIB)

.cpp.o:    
	$(CC) -c $(CFLAGS) $(INCPATH) $*.cpp -o $*.o

clean:
	rm -f *.o *~

depend: 
	makedepend -Y. $(SRC)

# DO NOT DELETE


mpkAdaptSegmentChecker.c.o: mpkAdaptSegmentChecker.H mpkBaseRobot.H
mpkAdaptSegmentChecker.c.o: mpkCollDistAlgo.H mpkCollPair.H mpkConfig.H
mpkAdaptSegmentChecker.c.o: mpkConfigChecker.H mpkRobotCollection.H
mpkAdaptSegmentChecker.c.o: mpkTransform.H mpk_defs.h mpk_inventor.h
mpkBaseRobot.c.o: ../gui/mpkGUI.H /usr/include/GL/gl.h /usr/include/GL/glext.h
mpkBaseRobot.c.o: mpkBaseRobot.H mpkConfig.H mpkTransform.H mpk_defs.h
mpkBaseRobot.c.o: mpk_inventor.h
mpkCollDistAlgo.c.o: mpkCollDistAlgo.H mpk_defs.h
mpkCollPair.c.o: mpkBaseRobot.H mpkCollPair.H mpkConfig.H mpkTransform.H
mpkCollPair.c.o: mpk_defs.h mpk_inventor.h mpk_rand.h
mpkConfigChecker.c.o: mpkBaseRobot.H mpkCollDistAlgo.H mpkCollPair.H
mpkConfigChecker.c.o: mpkConfig.H mpkConfigChecker.H mpkRobotCollection.H
mpkConfigChecker.c.o: mpkTransform.H mpk_defs.h mpk_inventor.h
mpkIncludeFile.c.o: mpkIncludeFile.H mpk_inventor.h
mpkObstacle.c.o: mpkObstacle.H mpk_inventor.h
mpkObstacleCollection.c.o: mpkObstacle.H mpkObstacleCollection.H
mpkObstacleCollection.c.o: mpkTransform.H mpk_inventor.h
mpkPathSmoother.c.o: mpkBaseRobot.H mpkCollDistAlgo.H mpkCollPair.H
mpkPathSmoother.c.o: mpkConfig.H mpkConfigChecker.H mpkPathSmoother.H
mpkPathSmoother.c.o: mpkRobotCollection.H mpkSimpleSegmentChecker.H
mpkPathSmoother.c.o: mpkTransform.H mpk_defs.h mpk_inventor.h mpk_rand.h
mpkRobot.c.o: mpkBaseRobot.H mpkConfig.H mpkRobot.H mpkTransform.H mpk_defs.h
mpkRobot.c.o: mpk_inventor.h mpk_robot_type.h
mpkRobotCollection.c.o: mpkBaseRobot.H mpkConfig.H mpkRobot.H
mpkRobotCollection.c.o: mpkRobotCollection.H mpkTransform.H mpk_defs.h
mpkRobotCollection.c.o: mpk_inventor.h
mpkSimpleSegmentChecker.c.o: mpkBaseRobot.H mpkCollDistAlgo.H mpkCollPair.H
mpkSimpleSegmentChecker.c.o: mpkConfig.H mpkConfigChecker.H
mpkSimpleSegmentChecker.c.o: mpkRobotCollection.H mpkSimpleSegmentChecker.H
mpkSimpleSegmentChecker.c.o: mpkTransform.H mpk_defs.h mpk_inventor.h
