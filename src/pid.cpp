#include "pid.h"

//Kalman kPitch;
//Kalman kRoll;

//roll, pitch, yaw, depth, accX, accY;

double attitude_desired[3];
double attitude_measured[3];
enum {PITCH, ROLL, YAW};

double surge;
double strafe;
double velocity[3];
enum {VX, VY, VZ};

double desdepth;
double depth;

int motorPowers[9];

std::ofstream log;

PID pPitch, pHead, pDepth;

bool SIM = true;

void reset_pid()
{
	pPitch.reset();
	pHead.reset();
	pDepth.reset();
}

void init_pid()
{
	//NOTE: The pid here calculates in degrees.

	//These gains change how quickly the pid can zero in on the right value.
	//Look up pid gains if you need to know.
	pPitch.setGains(PITCH_KP, PITCH_KI, PITCH_KD);
	pHead.setGains(HEAD_KP, HEAD_KI, HEAD_KD);
	pDepth.setGains(DEPTH_KP, DEPTH_KI, DEPTH_KD);
	
	//Don't try to overcompensate if the sub is too far off
	pPitch.setBounds(-30.0,30); //30 degrees up/down
	pHead.setBounds(-180,180);  //180 degrees around
	pDepth.setBounds(-6,6);     //6 inches up/down
	
	//The point that the pid is trying to reach
	pPitch.setSetpoint(attitude_desired[PITCH]);
	pHead.setSetpoint(attitude_desired[YAW]);  
	pDepth.setSetpoint(desdepth);
	
	//The scale is applied AFTER the motor values are calculated and are applied to them.
	pPitch.setScale(100.0f/35);
	pHead.setScale(1.0f/180);
	pDepth.setScale(1.0f/3);
	
	//A number that depends on the sample rate of the sensors
	pPitch.setDt(DT);
	pHead.setDt(DT);
	pDepth.setDt(DT);

	//TODO: These values are set via calibration file
	pPitch.setBias(0.0f);
	pHead.setBias(0.0f);
	pDepth.setBias(0.0f);

	pPitch.setIntegralRegion(-1000.0f/35,1000.0f/35);	//-10 to 10 degrees
	pHead.setIntegralRegion(1.0f/6,1.0f/6);		//-30 to 30 degrees
	pDepth.setIntegralRegion(-12.0f, 8.0f);		//-3 to 2 feet
	
	//TODO: Calibrate, set all that stuff
	
	reset_pid();
}

void do_pid()
{
	//TODO: Use Kalman filter for this stuff
	float pVal = attitude_measured[PITCH]*(180/3.14);

	//TODO: We will need to control for roll
	//float rVal = attitude_measured[ROLL];

	float head = attitude_measured[YAW]*(180/3.14); //TODO: Figure out if this is right and how heading works
	
	float pPid = pPitch.update(pVal, attitude_desired[PITCH]); 
	float hPid = pHead.update(head, attitude_desired[YAW]);
	float dPid = pDepth.update(depth, desdepth);
	
	update_motors(pPid, hPid, dPid);
}

void update_motors(float pPid, float hPid, float dPid)
{
	//This is roughly how it should work
	motorPowers[SRGE_L] = surge + hPid*100;
	motorPowers[SRGE_R] = surge - hPid*100;
	motorPowers[DIAG_L] = -strafe;
	motorPowers[DIAG_R] = strafe;
	motorPowers[STRAFE] = -strafe;
	motorPowers[VERT_FL] = dPid - pPid;
	motorPowers[VERT_FR] = dPid - pPid;
	motorPowers[VERT_BL] = dPid + pPid;
	motorPowers[VERT_BR] = dPid + pPid;
	if(SIM)
	{
		std::cout << motorPowers[SRGE_L] << " " << motorPowers[SRGE_R] << " " << motorPowers[DIAG_L] << " " << motorPowers[DIAG_R] << " " << motorPowers[VERT_FL] << " " << motorPowers[VERT_FR] << " " << motorPowers[VERT_BL] << " " << motorPowers[VERT_BR] << " " << motorPowers[STRAFE] << " " << 20 << std::endl;
		log << "\t" << motorPowers[SRGE_L] << "\t" << motorPowers[SRGE_R] << "\t" << motorPowers[DIAG_L] << "\t" << motorPowers[DIAG_R] << "\t" << motorPowers[VERT_FL] << "\t" << motorPowers[VERT_FR] << "\t" << motorPowers[VERT_BL] << "\t" << motorPowers[VERT_BR] << "\t" << motorPowers[STRAFE] << "\t" << 20 << std::endl;
	}
}

int main()
{
	//Pretty self-explanatory
	init_pid();
	
	attitude_desired[PITCH] = 0;
	attitude_desired[ROLL]  = 0;
	attitude_desired[YAW]   = 0;
	desdepth = 10;
	surge = 0;

	if(SIM)
	{
		//Just to record what's going on
		log.open("log.txt");
		//Give 0 to simulator first
		std::cout << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
	}

	while(true)
	{
		if(SIM)
		{
			//Get the state of the sim
			std::cin >> attitude_measured[YAW] >> attitude_measured[PITCH] >> attitude_measured[ROLL] >> depth >> velocity[VX] >> velocity[VY];
			log << attitude_measured[ROLL] << "\t" << attitude_measured[PITCH] << "\t" << attitude_measured[YAW] << "\t" << depth << "\t\t" << velocity[VX] << "\t" << velocity[VY];
		}
		//Use the info we got to update the pid and output the motor configurations
		do_pid();
	}
	if(SIM)
	{
		//What else would this do
		log.close();
	}
}
