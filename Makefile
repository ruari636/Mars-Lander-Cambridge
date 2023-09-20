CC = g++
CCSW = -O3 -Wno-deprecated-declarations
PLATFORM = `uname`

all:	lander spring

lander: lander.o lander_graphics.o lander_special_func.o orbit_transfers.o
	@if [ "${PLATFORM}" = "Linux" ]; \
	then \
		$(CC) -o lander lander.o lander_graphics.o lander_special_func.o orbit_transfers.o ${CCSW} -lGL -lGLU -lglut; \
		echo Linking for Linux; \
	else \
		echo "Unable to make outside of Linux"; \
	fi

lander_graphics.o lander.o: lander.h

spring: spring.o

.cpp.o:
	$(CC) ${CCSW} -c $<

clean:
	echo cleaning up; /bin/rm -f core *.o lander spring
