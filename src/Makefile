realtime: Realtime.cpp Realtime.h
	g++ -c Realtime.cpp -O2

alphabot: Alphabot.cpp Alphabot.h
	g++ -c Alphabot.cpp -O2

all: Realtime.o Alphabot.o
	g++ -std=c++11 -Wall -Wextra tasks.cpp Realtime.o Alphabot.o -o tasks -pthread -lpigpio `pkg-config --cflags --libs opencv4` -O2

clean:
	rm tasks