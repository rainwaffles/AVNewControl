VERSION = 2015-10

all: clean pid

pid:
	g++ Kalman.cpp pid_library.cpp main.cpp -o pid 
clean:
	rm -f pid
