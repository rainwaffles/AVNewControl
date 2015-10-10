#ifndef PID_H
#define PID_H

#include "Kalman.h"
#include "pid_library.h"

//Include: motor interface, simulator

//Info about pitch heading depth acceleration etc

//Instances of the pid class (from pid_library) for each p/h/d

//Volatile vars that the pid receives from EVA to control it
extern volatile int desHead, desDepth, desPower;
extern volatile int depth;
extern volatile bool isAlive;

//Some kind of motor array

void reset_pid();
void init_pid();
void do_pid();
void update_motors();
void give_data();

#endif
