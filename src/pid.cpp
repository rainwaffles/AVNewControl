//Template for what a pid would look like, basically copied from existing AVNavControl
#include "pid.h"



void reset_pid()
{
	
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
