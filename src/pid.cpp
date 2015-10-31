//Template for what a pid would look like, basically copied from existing AVNavControl
#include "pid.h"

Kalman kPitch;
Kalman kRoll;

void reset_pid()
{
	pPitch->reset();
	pHead->reset();
	pDepth->reset();
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
	
	pPitch->setGains(PITCH_KP, PITCH_KI, PITCH_KD);
	pHead->setGains(HEAD_KP, HEAD_KI, HEAD_KD);
	pDepth->setGains(DEPTH_KP, DEPTH_KI, DEPTH_KD);
	
	//Double check this
	pPitch->setBounds(-30,30);
	pHead->setBounds(-180,180);
	pDepth->setBounds(-6,6);
	
	pPitch->setSetpoint(0.0f);
	pHead->setSetpoint(desHead);  
	pDepth->setSetpoint(desDepth);
	
	//Double check this
	pPitch->setScale(100.0f/35);
	pHead->setScale(1.0f/180);
	pDepth->setScale(1.0f/3);
	
	pPitch->setDT(DT);
	pHead->setDT(DT);
	pDepth->setDT(DT);
	
	//Double check this
	pPitch->setBias(0.0f);
	pHead->setBias(0.0f);
	pDepth->setBias(0.0f);
	
	//Double check this
	pPitch->setIntegralRegion(-1000.0f/35,1000.0f/35);
	pHead->setIntegralRegion(1.0f/6,1.0f/6);
	pDepth->setIntegralRegion(-12.0f, 8.0f);
	
	//Calibrate, set all that stuff
	
	reset_pid();
}

void do_pid()
{
	float pVal = kPitch.calculate();
	float rVal = kRoll.calculate();
	
	float pPid = pPitch->update();
	float hPid = pHead->update();
	float dPid = pDepth->update();
	
	update_motors(pPid, hPid, dPid);
}

void update_motors(float pPid, float hPid, float dPid)
{
	
}

void update_data()
{
	
}
