//Template for what a pid would look like, basically copied from existing AVNavControl
#include "pid.h"
#include "motor.h"
#include "simulator.cpp"

void reset_pid()
{
	
}

void init_pid()
{
	//this is actually new and created by me; don't remove it; you know this is recent because for some reason
	//our forefathers never actually bothered to comment any of their code
	//sets inputs and motor outputs to 0
	sim_input.fill(0);
	motorPower.fill(0);

	if(pPitch != NULL)
	{
		delete pPitch;
		delete pHead;
		delete pDepth;
	}
	pPitch = new PID();
	pHead  = new PID();
	pDepth = new PID();
	
	//pPitch->setGains(); //I need to find out where these values come from
	//pHead->setGains();
	//pDepth->setGains();

	pPitch->setBounds(-30,30);
	pHead->setBounds(-180,180);
	pDepth->setBounds(-6,6);
	
	pPitch->setSetpoint(0.0f);
	pHead->setSetpoint(desHead);  
	pDepth->setSetpoint(desDepth);
	
	pPitch->setScale(100.0f/35);
	pHead->setScale(1.0f/180);
	pDepth->setScale(1.0f/3);
	
	
}

void do_pid()
{
	
}

void update_motors()
{
	
}

void update_data()
{
	
}
