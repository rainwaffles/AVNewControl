VERSION = 2015-10

all: pid

pid:
	g++ ./src/Kalman.cpp ./src/pid_library.cpp ./src/main.cpp ./src/simulator.cpp -o pid 
clean:
	rm -f pid
