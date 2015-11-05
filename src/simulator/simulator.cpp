#include "simulator.h"
#include "motor.h"
#include <iostream>
#include <string.h>
//this holds the inputs for varius IMU inputs
//time to make up some value!!
//////////////////////////////////////////////////////
//IMPORTANT: DEAR FUTURE AVBOTZ MEMBER,
//motorPower is found in motor.h
//	it contains all desired motor settings
//	it has 9 values, all enumerated
//	SRGE_L, SRGE_R, DIAG_L, DIAG_R, VERT_FL, VERT_FR, VERT_BL, VERT_BRSTRAFE
//	in that order from 0 to 8
//simInput is found int simulator.h
//	it contains all simulated IMU inputs
//	it has 6 values, all enumerated
//	X_ANGLE, Y_ANGLE, Z_ANGLE, X_POS, Y_POS, Z_POS
//	in that order, from 0 to 5
//
//I hope this will aid you in your future endeavors. You're welcome
/////////////////////////////////////////////////////////
float calculateZAngle()
{
	//assuming it updates every 1/4 of a second
	//angles/second for diag and strafe motors 
	//positive clockwise
	int diagTotal = ((motorPower[DIAG_L] - motorPower[DIAG_R]) / 510) * 15;
	int strafeTotal = (motorPower[STRAFE]/255) * 8;
	return ( simInput[Z_ANGLE] + diagTotal + strafeTotal);
}

float calculateXAngle()
{
	//assuming it updates every 1/4 of a second
	//angles/second for diag and strafe motors 
	//positive when front points up
	int frontTotal = (motorPower[VERT_FL] + motorPower[VERT_FR]) / 40 ;
	int backTotal = (motorPower[VERT_BL] + motorPower[VERT_BR]) / 40;
	return (simInput[X_ANGLE] + (backTotal - frontTotal));
}

float calculateYAngle()
{
	//assuming it updates every 1/4 of a second
	//angles/second for diag and strafe motors 
	int leftTotal = (motorPower[VERT_FL] + motorPower[VERT_BL])/40;
	int rightTotal = (motorPower[VERT_FR] + motorPower[VERT_BR])/40;
	return (simInput[Y_ANGLE] + (leftTotal - rightTotal));
}

//velocity is the sum of all past acceleration
//position is the sum of all past velocity
//
float ZVelocity;
float ZAccel;
float calculateZPos()
{
	//estimated acceleration: .5 m/s/s at full thrust
	//divide by a constant to estimate acceleration better
	ZAccel = (motorPower[VERT_FL] + motorPower[VERT_FR] + motorPower[VERT_BR] + motorPower[VERT_BL]) * (4/512);
	ZVelocity += ZAccel;
	//std::cout << " Z Accel: " << ZAccel << " Z Velocity: " << ZVelocity;
	return simInput[Z_POS] + ZVelocity;
}	

float XVelocity;
float XAccel;
float calculateXPos()
{
	//estimated acceleration: .5 m/s/s at full thrust
	//divide by a constant to estimate acceleration better
	XAccel = ((motorPower[SRGE_L] + motorPower[SRGE_R]) + (motorPower[DIAG_L]/2) + (motorPower[DIAG_R]/2))/64;
	XVelocity += XAccel;
	return simInput[X_POS] + XVelocity;
}

float YVelocity;
float YAccel;
float calculateYPos()
{
	//divide by a constant to estimate acceleration better
	YAccel = motorPower[STRAFE]/1024;
	YVelocity += YAccel;
	return simInput[Y_POS] + YVelocity;
}

void update()
{
	// sequences functions 
	// updates all input and output values
	simInput[Z_ANGLE] = calculateZAngle();
	simInput[X_ANGLE] = calculateXAngle();
	simInput[Y_ANGLE] = calculateYAngle();
	simInput[Z_POS] = calculateZPos();
	simInput[X_POS] = calculateXPos();
	simInput[Y_POS] = calculateYPos();
	
}
void output()
{
	//'locks in' results and displays results and predicted effects/rotations
	std::cout << simInput[Y_ANGLE] << " " << simInput[X_ANGLE] << " " << simInput[Z_ANGLE] << " " << 50 << " " << simInput[X_POS] << " " << simInput[Y_POS] << std::endl;
	//outputs results
	//remove the 'angle' markers when we have a program that can work with this one
	/*
	std::cout << " Z Angle: " << simInput[Z_ANGLE] << " X Angle: " << simInput[X_ANGLE] << " Y Angle: " << simInput[Y_ANGLE];
	std::cout << " Z Pos: " << simInput[Z_POS] << " X Pos: " << simInput[X_POS] << " Y Pos: " << simInput[Y_POS];
	*/
} 
int main()
{
	//initializes motors and inputs at zero
	memset(motorPower, 0, sizeof(int) * 9);
	memset(simInput , 0, sizeof(int) * 9);
	XVelocity = 0;
	YVelocity = 0;
	ZVelocity = 0;
	while(true)
	{
		//std::cout <<  " \n Input motor variables manually .\n" <<" VERT_FL, VERT_FR, VERT_BR, VERT_BL, DIAG_L, DIAG_R, SRGE_L, SRGE_R, STRAFE \n";
		//manual input; remove when we actually have a program that can work with this one
		std::cin >> motorPower[VERT_FL] >> motorPower[VERT_FR] >> motorPower[VERT_BR] >> motorPower[VERT_BL]>> motorPower[DIAG_L] >> motorPower[DIAG_R] >> motorPower[SRGE_L] >> motorPower[SRGE_R] >> motorPower[STRAFE];
		update();
		output();	
	}
}
