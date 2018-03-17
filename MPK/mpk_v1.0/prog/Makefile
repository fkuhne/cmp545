include ../Make.config

SRC =	demo.cpp \
	fmstudio.cpp \
	randwalk.cpp

PROGS = $(SRC:.cpp=)


.SUFFIXES : .out .a .o .c .cpp .C .h .H

INCPATH = $(INCALL) -I../basic -I../robots -I../gui -I../sbl

LIBPATH = $(LIBALL) -L../lib

LIBS =	-lgui -lsbl -lbasic -lrob \
	-lInventor -lInventorXt -lXm -lGL -lXext -lSM -lICE -lX11 \
	-ldl -lXt -lXp -lXmu -lPQP

LIBF =	../lib/libbasic.a \
	../lib/libgui.a \
	../lib/libsbl.a \
	../lib/librob.a

demo:	demo.o $(LIBF)
	$(CC) $(LFLAGS) -o demo demo.o $(LIBPATH) $(LIBS)

randwalk:	randwalk.o $(LIBF)
	$(CC) $(LFLAGS) -o randwalk randwalk.o $(LIBPATH) $(LIBS)

fmstudio:	fmstudio.o $(LIBF)
	$(CC) $(LFLAGS) -o fmstudio fmstudio.o $(LIBPATH) $(LIBS)


.C.o .cpp.o:
	$(CC) -c $(CFLAGS) $(INCPATH) $*.cpp -o $*.o

clean:
	rm -f *.o *~

cleanall: clean
	rm -f $(PROGS)

depend: 
	makedepend -Y -I. -I../basic -I../gui -I../sbl $(SRC)

# DO NOT DELETE


demo.c.o: ../basic/mpkAdaptSegmentChecker.H ../basic/mpkBaseRobot.H
demo.c.o: ../basic/mpkCollDistAlgo.H ../basic/mpkCollPair.H
demo.c.o: ../basic/mpkCollPairSet.H ../basic/mpkConfig.H
demo.c.o: ../basic/mpkConfigChecker.H ../basic/mpkObstacleCollection.H
demo.c.o: ../basic/mpkRobotCollection.H ../basic/mpkSimpleSegmentChecker.H
demo.c.o: ../basic/mpkTransform.H ../basic/mpk_defs.h ../basic/mpk_inventor.h
demo.c.o: ../basic/mpk_rand.h ../gui/mpkGUI.H /usr/include/GL/gl.h
fmstudio.c.o: ../basic/mpkBaseRobot.H ../basic/mpkCollDistAlgo.H
fmstudio.c.o: ../basic/mpkCollPair.H ../basic/mpkCollPairSet.H
fmstudio.c.o: ../basic/mpkConfig.H ../basic/mpkConfigChecker.H
fmstudio.c.o: ../basic/mpkObstacleCollection.H ../basic/mpkPathSmoother.H
fmstudio.c.o: ../basic/mpkRobotCollection.H ../basic/mpkSimpleSegmentChecker.H
fmstudio.c.o: ../basic/mpkTransform.H ../basic/mpk_defs.h
fmstudio.c.o: ../basic/mpk_inventor.h ../basic/mpk_rand.h
fmstudio.c.o: ../gui/mpkBVControl.H ../gui/mpkGUI.H ../gui/mpkMouseControl.H
fmstudio.c.o: ../gui/mpkTraceVis.H ../sbl/sblBin.H ../sbl/sblEdge.H
fmstudio.c.o: ../sbl/sblMilestone.H ../sbl/sblPlanner.H ../sbl/sblRandVal.H
fmstudio.c.o: ../sbl/sblRn.H ../sbl/sblTree.H /usr/include/GL/gl.h
randwalk.c.o: ../basic/mpkBaseRobot.H ../basic/mpkCollDistAlgo.H
randwalk.c.o: ../basic/mpkCollPair.H ../basic/mpkCollPairSet.H
randwalk.c.o: ../basic/mpkConfig.H ../basic/mpkConfigChecker.H
randwalk.c.o: ../basic/mpkIncludeFile.H ../basic/mpkObstacle.H
randwalk.c.o: ../basic/mpkObstacleCollection.H ../basic/mpkRobot.H
randwalk.c.o: ../basic/mpkRobotCollection.H ../basic/mpkSimpleSegmentChecker.H
randwalk.c.o: ../basic/mpkTransform.H ../basic/mpk_defs.h
randwalk.c.o: ../basic/mpk_inventor.h ../basic/mpk_rand.h
randwalk.c.o: ../gui/mpkBVControl.H ../gui/mpkGUI.H ../gui/mpkTraceVis.H
