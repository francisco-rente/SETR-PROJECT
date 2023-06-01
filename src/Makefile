realtime: Realtime.cpp Realtime.h
	g++ -c Realtime.cpp

alphabot: Alphabot.cpp Alphabot.h
	g++ -c Alphabot.cpp

all: Realtime.o Alphabot.o
	g++ -std=c++11 -Wall -Wextra tasks.cpp Realtime.o Alphabot.o -o tasks -pthread -lpigpio `pkg-config --cflags --libs opencv4`

clean:
	rm tasks