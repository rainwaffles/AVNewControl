//Array simInput simulates our expected inputs from Angle in degrees and position in meters.
float simInput[6];
//enmerating angles and stuff
enum angles {X_ANGLE, Y_ANGLE, Z_ANGLE, X_POS, Y_POS, Z_POS};
void update();
//intializes calculation functions
float calculateZAngle(), calculateXAngle(), calculateYAngle(), calculateZPos(), calculateXPos(), calculateYPos();
//intializes overarching functions
void output(), update();

