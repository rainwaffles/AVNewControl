#ifndef PID_H
#define PID_H

#include <iostream>
#include <fstream>
#include "Kalman.h"
#include "pid_library.h"

//Also include: motor interface, simulator

const float PITCH_KP = 0.012f;
const float PITCH_KI = 0.005f;
const float PITCH_KD = 0.007f;

const float HEAD_KP = 4.0f;
const float HEAD_KI = 0.75f;
const float HEAD_KD = 0.25f;
 
const float DEPTH_KP = 1.0f;
const float DEPTH_KI = 0.1f;
const float DEPTH_KD = 0.2f;

enum MOTOR
{
	SRGE_L,
	SRGE_R,
	DIAG_L,
	DIAG_R,
	VERT_FL,
	VERT_FR,
	VERT_BL,
	VERT_BR,
	STRAFE
};

//Info about pitch heading depth acceleration etc

//Instances of the pid class (from pid_library) for each p/h/d
//Kalman filters for p/h
PID *pPitch, *pHead, *pDepth;

//Volatile vars that the pid receives from EVA to control it
volatile int desHead, desDepth, desPower, desStrafe;
volatile bool isAlive;

//Some kind of motor array


void reset_pid();
void init_pid();
void do_pid();
void update_motors(float pPid, float hPid, float dPid);

#endif
