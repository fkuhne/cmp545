include ../Make.config

LIB =	../lib/libsbl.a

SRC =	sblPlanner.cpp \
	sblRandVal.cpp \
	sblRn.cpp \
	sblTree.cpp

OBJ =	$(SRC:.cpp=.o)

.SUFFIXES : .cpp .H

INCPATH = -I. -I../basic -I../robots -I../segcheck $(INCALL)

$(LIB):	$(OBJ)
	ar ruv $(LIB) $(OBJ)
	ranlib $(LIB)

.cpp.o:    
	$(CC) -c $(CFLAGS) $(INCPATH) $*.cpp -o $*.o

clean:
	rm -f *.o *~

depend: 
	makedepend -Y -I. -I../basic -I../robots -I../segcheck $(SRC)

# DO NOT DELETE


sblPlanner.c.o: ../basic/mpkBaseRobot.H ../basic/mpkCollDistAlgo.H
sblPlanner.c.o: ../basic/mpkCollPair.H ../basic/mpkConfig.H
sblPlanner.c.o: ../basic/mpkConfigChecker.H ../basic/mpkRobotCollection.H
sblPlanner.c.o: ../basic/mpkSimpleSegmentChecker.H ../basic/mpkTransform.H
sblPlanner.c.o: ../basic/mpk_defs.h ../basic/mpk_inventor.h
sblPlanner.c.o: ../basic/mpk_rand.h /usr/include/GL/gl.h
sblPlanner.c.o: sblBin.H sblEdge.H sblMilestone.H sblPlanner.H sblRandVal.H
sblPlanner.c.o: sblRn.H sblTree.H
sblRandVal.c.o: sblRandVal.H
sblRn.c.o: ../basic/mpkConfig.H ../basic/mpk_rand.h /usr/include/_G_config.h
sblRn.c.o: sblRandVal.H sblRn.H
sblTree.c.o: ../basic/mpkBaseRobot.H ../basic/mpkCollDistAlgo.H
sblTree.c.o: ../basic/mpkCollPair.H ../basic/mpkConfig.H
sblTree.c.o: ../basic/mpkConfigChecker.H ../basic/mpkRobotCollection.H
sblTree.c.o: ../basic/mpkSimpleSegmentChecker.H ../basic/mpkTransform.H
sblTree.c.o: ../basic/mpk_defs.h ../basic/mpk_inventor.h ../basic/mpk_rand.h
sblTree.c.o: sblBin.H sblEdge.H sblMilestone.H sblTree.H
