all: 
	g++ -std=c++11 -Wall -Wextra tasks.cpp -o tasks -pthread -lpigpio `pkg-config --cflags --libs opencv4`

clean:
	rm tasks