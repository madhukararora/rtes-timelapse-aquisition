INCLUDES=	-I/usr/local/opencv/include

CC=	g++

CFLAGS=	-O0	-g	-std=c++11	

CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lpthread -lrt 
 
CPPFILES= code_1Hz.cpp

CPPOBJS= ${CPPFILES:.cpp=.o}

build:	code_1Hz code_10Hz

clean:
	-rm -f *.o *.d *.jpg *.ppm
	-rm -f code_1Hz code_10Hz

code_1Hz:	code_1Hz.o	
	$(CC)	$(LDFLAGS)	$(INCLUDES)	$(CFLAGS)	-o	code_1Hz	code_1Hz.o	`pkg-config --libs opencv`	$(CPPLIBS)

code_10Hz:	code_10Hz.o	
	$(CC)	$(LDFLAGS)	$(INCLUDES)	$(CFLAGS)	-o	code_10Hz	code_10Hz.o	`pkg-config --libs opencv`	$(CPPLIBS)
