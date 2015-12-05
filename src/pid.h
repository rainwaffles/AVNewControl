#ifndef PID_H
#define PID_H

#include <iostream>
#include <fstream>
#include "Kalman.h"
#include "pid_library.h"

//Also include: motor interface, simulator

#define PITCH_KP 0.012f
#define PITCH_KI 0.005f
#define PITCH_KD 0.007f

#define HEAD_KP 4.0f
#define HEAD_KI 0.75f
#define HEAD_KD 0.25f

#define DEPTH_KP 1.0f
#define DEPTH_KI 0.1f
#define DEPTH_KD 0.2f

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

void reset_pid();
void init_pid();
void do_pid();
void update_motors(float pPid, float hPid, float dPid);

#endif
