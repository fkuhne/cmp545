include ../Make.config

LIB =	../lib/libgui.a

SRC1 =	mpkBVControl.cpp \
	mpkMouseControl.cpp \
	mpkTraceVis.cpp \
	mpkGUI.cpp

SRC  =	$(SRC1) mpk_opt.cpp

OBJ1 =	$(SRC1:.cpp=.o)
OBJ  =	$(SRC:.cpp=.o)

.SUFFIXES : .cpp .H

INCPATH = -I. -I../basic -I../robots $(INCALL)

$(LIB):	$(OBJ)
	ar ruv $(LIB) $(OBJ)
	ranlib $(LIB)

noopt:   $(OBJ1)
	ar ruv $(LIB) $(OBJ1)
	ranlib $(LIB)

.cpp.o:    
	$(CC) -c $(CFLAGS) $(INCPATH) $*.cpp -o $*.o

clean:
	rm -f *.o *~

depend: 
	makedepend -Y -I. -I../basic -I../robots $(SRC)

# DO NOT DELETE


mpkBVControl.c.o: ../basic/mpkBaseRobot.H ../basic/mpkConfig.H
mpkBVControl.c.o: ../basic/mpkObstacleCollection.H
mpkBVControl.c.o: ../basic/mpkRobotCollection.H ../basic/mpkTransform.H
mpkBVControl.c.o: ../basic/mpk_defs.h ../basic/mpk_inventor.h
mpkBVControl.c.o: ../basic/mpk_rand.h /usr/include/GL/gl.h
mpkBVControl.c.o: mpkBVControl.H
mpkGUI.c.o: ../basic/mpkBaseRobot.H ../basic/mpkConfig.H
mpkGUI.c.o: ../basic/mpkIncludeFile.H ../basic/mpkObstacle.H
mpkGUI.c.o: ../basic/mpkObstacleCollection.H ../basic/mpkRobot.H
mpkGUI.c.o: ../basic/mpkRobotCollection.H ../basic/mpkTransform.H
mpkGUI.c.o: ../basic/mpk_defs.h ../basic/mpk_inventor.h ../basic/mpk_rand.h
mpkGUI.c.o: mpkGUI.H
mpkMouseControl.c.o: ../basic/mpkBaseRobot.H ../basic/mpkConfig.H
mpkMouseControl.c.o: ../basic/mpkObstacleCollection.H
mpkMouseControl.c.o: ../basic/mpkRobotCollection.H ../basic/mpkTransform.H
mpkMouseControl.c.o: ../basic/mpk_defs.h ../basic/mpk_inventor.h
mpkMouseControl.c.o: mpkMouseControl.H
mpkTraceVis.c.o: ../basic/mpkBaseRobot.H ../basic/mpkConfig.H
mpkTraceVis.c.o: ../basic/mpkRobotCollection.H ../basic/mpkTransform.H
mpkTraceVis.c.o: ../basic/mpk_defs.h ../basic/mpk_inventor.h
mpkTraceVis.c.o: mpkTraceVis.H
