#ifndef PID_H
#define PID_H

#include "Kalman.h"
#include "pid_library.h"

//Also include: motor interface, simulator

static const float PITCH_KP 0.012f 
static const float PITCH_KI 0.005f 
static const float PITCH_KD 0.007f 

static const float HEAD_KP 4.0f 
static const float HEAD_KI 0.75f 
static const float HEAD_KD 0.25f 
 
static const float DEPTH_KP 1.0f 
static const float DEPTH_KI 0.1f 
static const float DEPTH_KD 0.2f 

//Info about pitch heading depth acceleration etc

//Instances of the pid class (from pid_library) for each p/h/d
//Kalman filters for p/h
PID *pPitch, *pHead, *pDepth;
Kalman kPitch, kRoll;

//Volatile vars that the pid receives from EVA to control it
extern volatile int desHead, desDepth, desPower, desStrafe;
extern volatile int depth;
extern volatile bool isAlive;

//Some kind of motor array


void reset_pid();
void init_pid();
void do_pid();
void update_motors();
void update_data();

#endif
