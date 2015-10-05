all: 
	g++ Kalman.cpp main.cpp pid_library.cpp -o pid 
clean:
	rm pid
