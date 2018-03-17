include ../Make.config

LIB =	../lib/librob.a

SRC =	mpkFreeFlyingObject.cpp \
	mpkDemoRobot.cpp

OBJ =	$(SRC:.cpp=.o)

.SUFFIXES : .cpp .H

INCPATH = -I. -I../basic $(INCALL)

$(LIB):	$(OBJ)
	ar ruv $(LIB) $(OBJ)
	ranlib $(LIB)

.cpp.o:    
	$(CC) -c $(CFLAGS) $(INCPATH) $*.cpp -o $*.o

clean:
	rm -f *.o *~

depend: 
	makedepend -Y -I. -I../basic $(SRC)

# DO NOT DELETE


mpkDemoRobot.c.o: ../basic/mpkBaseRobot.H ../basic/mpkConfig.H
mpkDemoRobot.c.o: ../basic/mpkTransform.H ../basic/mpk_defs.h
mpkDemoRobot.c.o: ../basic/mpk_inventor.h /usr/include/GL/gl.h
