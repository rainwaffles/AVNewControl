#include "simulator.h"
#include "motor.h"
#include <iostream>
#include <string.h>
//this holds the inputs for varius IMU inputs
//time to make up some value!!
//
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
	int frontTotal = (motorPower[VERT_FL] + motorPower[VERT_FR]) / 10 ;
	int backTotal = (motorPower[VERT_BL] + motorPower[VERT_BR]) / 10;
	return (simInput[X_ANGLE] + (backTotal - frontTotal));
}

float calculateYAngle()
{
	//assuming it updates every 1/4 of a second
	//angles/second for diag and strafe motors 
	int leftTotal = (motorPower[VERT_FL] + motorPower[VERT_BL])/10;
	int rightTotal = (motorPower[VERT_FR] + motorPower[VERT_BR])/10;
	return (simInput[Y_ANGLE] + (leftTotal - rightTotal));
}

void update()
{
	// sequences functions 
	// updates all input and output values
	simInput[Z_ANGLE] = calculateZAngle();
	simInput[X_ANGLE] = calculateXAngle();
	simInput[Y_ANGLE] = calculateYAngle();
	
}
void output()
{
	//'locks in' results and displays results and predicted effects/rotations
	std::cout << " Z Angle: " << simInput[Z_ANGLE] << " X Angle: " << simInput[X_ANGLE] << " Y Angle: " << simInput[Y_ANGLE];
} 
int main()
{
	memset(motorPower, 0, sizeof(int) * 9);
	memset(simInput , 0, sizeof(int) * 9);
	while(true)
	{
		std::cout <<  " \n Input motor variables manually .\n" <<" VERT_FL, VERT_FR, VERT_BR, VERT_BL, DIAG_L, DIAG_R, SRGE_L, SRGE_R, STRAFE \n";
		std::cin >> motorPower[VERT_FL] >> motorPower[VERT_FR] >> motorPower[VERT_BR] >> motorPower[VERT_BL]>> motorPower[DIAG_L] >> motorPower[DIAG_R] >> motorPower[SRGE_L] >> motorPower[SRGE_R] >> motorPower[STRAFE];
		update();
		output();	
	}
}
