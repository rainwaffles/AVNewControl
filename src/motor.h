
class Motor
{
	public:
		Motor();
		//contains motor power values
		//motor definitions; 0,1:surge; 2,3:diagonal surge; 4-7: vertical; 8:back strafe
		const int SRGE_L  = 0;
		const int SRGE_R  = 1;
		const int DIAG_L  = 2;
	       	const int DIAG_R  = 3;	
		const int VERT_FL = 4;
		const int VERT_FR = 5;
		const int VERT_BL = 6;
		const int VERT_BR = 7;
		const int STRAFE  = 8;
		int motorPower[9];
		

}
