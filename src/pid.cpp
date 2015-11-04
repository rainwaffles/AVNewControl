#include "pid.h"

Kalman kPitch;
Kalman kRoll;

float roll, pitch, yaw, depth, accX, accY;

float fRoll, fPitch, fYaw, fDepth, fAccX, fAccY;

int motorPowers[9];

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
	
	pPitch->setDt(DT);
	pHead->setDt(DT);
	pDepth->setDt(DT);
	
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
	float pVal = pitch;//kPitch.calculate();
	float rVal = roll;//kRoll.calculate();
	float heading = yaw;	

	float pPid = pPitch->update(pVal, 0.0f);
	float hPid = pHead->update(heading, desHead);
	float dPid = pDepth->update(depth, desDepth);
	
	update_motors(pPid, hPid, dPid);
}

void update_motors(float pPid, float hPid, float dPid)
{
	motorPowers[SRGE_L] = desPower + hPid;
	motorPowers[SRGE_R] = desPower - hPid;
	motorPowers[DIAG_L] = desStrafe;
	motorPowers[DIAG_R] = -desStrafe;
	motorPowers[STRAFE] = desStrafe;
	motorPowers[VERT_FL] = dPid - pPid;
	motorPowers[VERT_FR] = dPid - pPid;
	motorPowers[VERT_BL] = dPid + pPid;
	motorPowers[VERT_BR] = dPid + pPid;
	std::cout << motorPowers[VERT_FL] << " " << motorPowers[VERT_FR] << " " << motorPowers[VERT_BR] << " " << motorPowers[VERT_BL] << " " << motorPowers[DIAG_L] << " " << motorPowers[DIAG_R] << " " << motorPowers[SRGE_L] << " " << motorPowers[SRGE_R] << " " << motorPowers[STRAFE] << std::endl;
}

int main()
{
	desHead = 15;
	desDepth = 50;
	desPower = 100;
	desStrafe = 10;
	init_pid();
	std::ofstream log;
	log.open("log.txt");
	std::cout << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
	for(int i = 0; i < 10; i++)
	{
		std::cin >> roll >> pitch >> yaw >> depth >> accX >> accY;
		log << roll << pitch << yaw << depth << accX << accY;
		do_pid();
	}
	log.close();
}
