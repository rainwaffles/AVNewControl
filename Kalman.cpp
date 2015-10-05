#include "Kalman.h"


// Initialize a kalman filter with a bias (or the gyroscope's offset from 0).
Kalman::Kalman()
{
	angle = 0;
	
	/*
	 * We initialize the covariance matrix to a zero matrix
	 * because we assume that we know the exact measurements
	 * of our angle and gyroscope bias and that all measurements
	 * are independent of one another.
	 *
	 * The elements on the main diagonal (top left to bottom right)
	 * represent the variances, and the other elements represent
	 * correlations. There's something about eigenvalues and error
	 * ellipses that apply to these; we'll see if I can learn
	 * what those are.
	 */
	P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0f;

}

Kalman::~Kalman()
{

}

void Kalman::setBias(float g_bias)
{
	bias = g_bias;
}

/*
 * Evaluate a kalman filter. The math here has been proven solid. Don't touch
 * it unless you really need to.
 */
float Kalman::calculate(int gyroReading, float accTheta)
{
	/*
	 * Update the predicted angle based on the previous
	 * angle and the new gyroscope reading.
	 */
	//GYRO_SCALE and DT are from mbed and defs.h, they are defined as 1 in Kalman.h and we should probs figure out
	//what they do.
	angle = angle - (bias - gyroReading) * DT  * GYRO_SCALE;

	/*
	 * angle_err is the actual error between the predicted
	 * and measured angle.
	 */
	angle_err = accTheta - angle;
	
	/*
	 * To calculate the predicted error in measuring
	 * the angle, we add the maximum possible
	 * jitter to the current covariance. From the Kalman paper:
	 *
	 *	  E = C P C' + R
	 *
	 * Dimensionally,
	 *
	 *	  E<1,1> = C<1,2> P<2,2> C'<2,1> + R<1,1>
	 *
	 * Since C = [1, 0], we don't compute the C[1] term
	 * to save a floating-point operation.
	 */
	E = P[0][0] + R_angle;

	/*
	 * Compute the Kalman filter gains. From the Kalman paper:
	 *
	 *	  K = P C' inv(E)
	 *
	 * Dimensionally:
	 *
	 *	  K<2,1> = P<2,2> C'<2,1> inv(E)<1,1>
	 *
	 * Luckily, E has dimensions of <1,1>, so the inverse of E is just 1/E.
	 *
	 * The transposition of C is [1]
	 *						   [0]
	 *
	 * The vanilla extended Kalman filter does not update the gains
	 * in the prediction set of equations. However, Daniel does.
	 * His equation:
	 *
	 *	  K = K - A P C' inv(E)
	 *
	 * Dimensionally:
	 *
	 *	  K<2,1> = K<2,1> - A<2,2> P<2,2> C'<2,1> inv(E)<1,1>
	 *
	 * I don't know why he does this.
	 */
	
	gain[0] = P[0][0] * DT / E;
	gain[1] = P[1][0] / E;
	

	/*
	 * Update our state estimate. Again, from the Kalman paper:
	 *
	 *	  X += K * err
	 *
	 * And, dimensionally,
	 *
	 *	  X<2> = X<2> + K<2,1> * err<1,1>
	 *
	 * err is a measurement of the difference in the measured state
	 * and the estimate state. In our case, it is just the difference
	 * between the two accelerometer measured angle and our estimated
	 * angle.
	 */
	angle += gain[0] * angle_err;
	bias += gain[1] * angle_err;
	
	
	/* The covariance matrix is defined as the covariances between the 
	 * nonlinear and linearized functions of the data points. 
	 * Ihe covariance matrix is updated with two equations. First, we have
	 *
	 * Pdot = A*P + P*A' + Q
	 * 
	 * A is the Jacobian of the state function:
	 *
	 * Xdot = [angle_dot, gyro_bias_dot]
	 *
	 * with respect to the states:
	 *
	 * X = [angle, gyro_bias].
	 *
	 * Xdot and X are related in this way:
	 * 
	 * angle_dot = gyro - gyro_bias
	 * gyro_bias_dot = 0
	 *
	 * Therefore,
	 *
	 * A = [d(angle_dot)/d(angle)	   d(angle_dot)/d(gyro_bias)]
	 *	 [d(gyro_bias_dot)/d(angle)   d(gyro_bias_dot)/d(gyro)bias)]
	 *
	 *   = [0 -1]
	 *	 [0  0]
	 *
	 * This is the predict step of the Kalman filter.
	 * To save floating point operations,
	 * we have calculated Pdot = A*P + P*A' + Q manually.
	 *
	 * Pdot = [Q_angle - P[0][1] - P[1][0],  -P[1][1]]
	 *		[-P[1][1]				   ,  Q_gyro]
	 * 
	 * This is multiplied by DT and added to P.
	 *
	 * The second equation from the Kalman paper is as follows:
	 *
	 *	  P = P - K C P
	 *
	 * Dimensionally:
	 *
	 *	  P<2,2> -= K<2,1> C<1,2> P<2,2>
	 *
	 * We first compute t<1,2> = C P.
	 * 
	 * Since C is [1, 0], t[0] = C[0] * P[0][0] ( + C[1] * P[1][0] = 0) = P[0][0]
	 * and				t[1] = C[0] * P[0][1] ( + C[1] * P[1][1] = 0) = P[0][1]
	 *
	 * We then multiply t by K0 and subtract the result from P.
	 */

	//These are ars_micropilot.c's equations for updating the covariance matrix.
	//This is an extended Kalman filter.
	P[0][0] = P[0][0] - DT * (P[0][1] + P[1][0]) - gain[0] * P[0][0] + Q_angle;
	P[0][1] = P[0][1] - DT * P[1][1] - gain[0] * P[0][1];
	P[1][0] = P[1][0] - DT * P[1][1] - gain[1] * P[0][0];
	P[1][1] = P[1][1] - gain[1] * P[0][1] + Q_gyro;
	
	return angle;
}
