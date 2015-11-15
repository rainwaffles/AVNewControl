#include "pid.h"

Kalman kPitch;
Kalman kRoll;

float roll, pitch, yaw, depth, accX, accY;

float fRoll, fPitch, fYaw, fDepth, fAccX, fAccY;

int motorPowers[9];

std::ofstream log;

void reset_pid()
{
	pPitch->reset();
	pHead->reset();
	pDepth->reset();
	
	fRoll = fPitch = fYaw = fDepth = fAccX = fAccY = 0;
}

void init_pid()
{
	if(pPitch != NULL)
	{
		delete pPitch;
		delete pHead;
		delete pDepth;
	}
	pPitch = new PID();
	pHead  = new PID();
	pDepth = new PID();
	
	//These gains change how quickly the pid can zero in on the right value.
	//Look up pid gains if you need to know.
	pPitch->setGains(PITCH_KP, PITCH_KI, PITCH_KD);
	pHead->setGains(HEAD_KP, HEAD_KI, HEAD_KD);
	pDepth->setGains(DEPTH_KP, DEPTH_KI, DEPTH_KD);
	
	//Don't try to overcompensate if the sub is too far off
	pPitch->setBounds(-30.0f/180,30.0f/180);
	pHead->setBounds(-1,1);
	pDepth->setBounds(-6,6);
	
	//The point that the pid is trying to reach
	pPitch->setSetpoint(0.0f);
	pHead->setSetpoint(desHead);  
	pDepth->setSetpoint(desDepth);
	
	//I have no idea
	pPitch->setScale(100.0f/35);
	pHead->setScale(1);
	pDepth->setScale(1.0f/3);
	
	//Mumble mumble pid thing
	pPitch->setDt(DT);
	pHead->setDt(DT);
	pDepth->setDt(DT);
	pPitch->setBias(0.0f);
	pHead->setBias(0.0f);
	pDepth->setBias(0.0f);
	pPitch->setIntegralRegion(-1000.0f/35,1000.0f/35);
	pHead->setIntegralRegion(1.0f/6,1.0f/6);
	pDepth->setIntegralRegion(-12.0f, 8.0f);
	
	//TODO: Calibrate, set all that stuff
	
	reset_pid();
}

void do_pid()
{
	//TODO: Use Kalman filter for this stuff
	float pVal = pitch;
	float rVal = roll;
	float heading = yaw; //TODO: Figure out if this is right (it isn't) and how heading works
	
	//TODO: This year we might need to control pitch somewhat for torpedo task
	float pPid = pPitch->update(pVal, 0.0f); 
	float hPid = pHead->update(heading, desHead);
	float dPid = pDepth->update(depth, desDepth);
	
	update_motors(pPid, hPid, dPid);
}

void update_motors(float pPid, float hPid, float dPid)
{
	//This is roughly how it should work
	motorPowers[SRGE_L] = desPower + hPid;
	motorPowers[SRGE_R] = desPower - hPid;
	motorPowers[DIAG_L] = -desStrafe;
	motorPowers[DIAG_R] = desStrafe;
	motorPowers[STRAFE] = -desStrafe;
	motorPowers[VERT_FL] = dPid - pPid;
	motorPowers[VERT_FR] = dPid - pPid;
	motorPowers[VERT_BL] = dPid + pPid;
	motorPowers[VERT_BR] = dPid + pPid;
	std::cout << motorPowers[SRGE_L] << " " << motorPowers[SRGE_R] << " " << motorPowers[DIAG_L] << " " << motorPowers[DIAG_R] << " " << motorPowers[VERT_FL] << " " << motorPowers[VERT_FR] << " " << motorPowers[VERT_BL] << " " << motorPowers[VERT_BR] << " " << motorPowers[STRAFE] << " " << 20 << std::endl;
	log << motorPowers[SRGE_L] << "\t" << motorPowers[SRGE_R] << "\t" << motorPowers[DIAG_L] << "\t" << motorPowers[DIAG_R] << "\t" << motorPowers[VERT_FL] << "\t" << motorPowers[VERT_FR] << "\t" << motorPowers[VERT_BL] << "\t" << motorPowers[VERT_BR] << "\t" << motorPowers[STRAFE] << "\t" << 20 << std::endl;
	
}

int main()
{
	//Just for testing
	desHead = 0;
	desDepth = 0;
	desPower = 100;
	desStrafe = 0;
	//Pretty self-explanatory
	init_pid();
	//Just to record what's going on
	log.open("log.txt");
	//Get initial state of sub from simulator/controller
	std::cout << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
	while(true)
	{
		//Get the state of the sub
		std::cin >> yaw >> pitch >> roll >> depth >> accX >> accY;
		//Record state for fun
		log << roll << "\t" << pitch << "\t" << yaw << "\t" << depth << "\t\t" << accX << "\t" << accY;
		//Use the info we got to update the pid and output the motor configurations
		do_pid();
	}
	//What else would this do
	log.close();
}
