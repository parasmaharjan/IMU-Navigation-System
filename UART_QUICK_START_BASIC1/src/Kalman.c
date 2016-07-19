#include <asf.h>
#include "kalman.h"

float Q_angle = .01; // Process noise variance for the accelerometer
float Q_bias = .003; // Process noise variance for the gyro bias
float R_measure = .003; // Measurement noise variance - this is actually the variance of the measurement noise

float angle = 0; // The angle calculated by the Kalman filter - part of the 2x1 state vector
float KFangleX = 0.0;
float KFangleY = 0.0;
float KFangleZ = 0.0;

float bias = 0; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float x_bias = 0;
float y_bias = 0;
float z_bias = 0;

float rate = 0; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

float P[2][2] = {{0,0},{0,0}}; // Error covariance matrix - This is a 2x2 matrix
float K[2]; // Kalman gain - This is a 2x1 vector
float y; // Angle difference
float S; // Estimate error

float XP_00 = 0, XP_01 = 0, XP_10 = 0, XP_11 = 0;
float YP_00 = 0, YP_01 = 0, YP_10 = 0, YP_11 = 0;
float ZP_00 = 0, ZP_01 = 0, ZP_10 = 0, ZP_11 = 0;

char buffer[10];

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
double getAngle(double newAngle, double newRate, double dt) {
	/* Step 1 */
	rate = newRate - bias;
	angle += dt * rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	S = P[0][0] + R_measure;
	
	/* Step 5 */
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	gcvt(K[0], 5, buffer);
	printf("K: %s\n\r",buffer);
	
	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	y = newAngle - angle;
	
	/* Step 6 */
	angle += K[0] * y;
	bias += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	P[0][0] -= K[0] * P[0][0];
	P[0][1] -= K[0] * P[0][1];
	P[1][0] -= K[1] * P[0][0];
	P[1][1] -= K[1] * P[0][1];

	return angle;
}

float kalmanFilterX(float accAngle, float gyroRate, float dt)
{
	float  y, S;
	float K_0, K_1;

	KFangleX += dt * (gyroRate - x_bias);

	XP_00 +=  - dt * (XP_10 + XP_01) + Q_angle * dt;
	XP_01 +=  - dt * XP_11;
	XP_10 +=  - dt * XP_11;
	XP_11 +=  + Q_bias * dt;

	y = accAngle - KFangleX;
	S = XP_00 + R_measure;
	K_0 = XP_00 / S;
	K_1 = XP_10 / S;

	KFangleX +=  K_0 * y;
	x_bias  +=  K_1 * y;
	XP_00 -= K_0 * XP_00;
	XP_01 -= K_0 * XP_01;
	XP_10 -= K_1 * XP_00;
	XP_11 -= K_1 * XP_01;

	return KFangleX;
}


float kalmanFilterY(float accAngle, float gyroRate, float dt)
{
	float  y, S;
	float K_0, K_1;

	KFangleY += dt * (gyroRate - y_bias);

	YP_00 +=  - dt * (YP_10 + YP_01) + Q_angle * dt;
	YP_01 +=  - dt * YP_11;
	YP_10 +=  - dt * YP_11;
	YP_11 +=  + Q_bias * dt;

	y = accAngle - KFangleY;
	S = YP_00 + R_measure;
	K_0 = YP_00 / S;
	K_1 = YP_10 / S;

	KFangleY +=  K_0 * y;
	y_bias  +=  K_1 * y;
	YP_00 -= K_0 * YP_00;
	YP_01 -= K_0 * YP_01;
	YP_10 -= K_1 * YP_00;
	YP_11 -= K_1 * YP_01;

	return KFangleY;
}

float kalmanFilterZ(float accAngle, float gyroRate, float dt)
{
	float  y, S;
	float K_0, K_1;

	KFangleZ += dt * (gyroRate - z_bias);

	ZP_00 +=  - dt * (ZP_10 + ZP_01) + Q_angle * dt;
	ZP_01 +=  - dt * ZP_11;
	ZP_10 +=  - dt * ZP_11;
	ZP_11 +=  + Q_bias * dt;

	y = accAngle - KFangleZ;
	S = ZP_00 + R_measure;
	K_0 = ZP_00 / S;
	K_1 = ZP_10 / S;

	KFangleZ +=  K_0 * y;
	z_bias  +=  K_1 * y;
	ZP_00 -= K_0 * ZP_00;
	ZP_01 -= K_0 * ZP_01;
	ZP_10 -= K_1 * ZP_00;
	ZP_11 -= K_1 * ZP_01;

	return KFangleZ;
}

void setAngle(double newAngle) { 
	angle = newAngle; 
} // Used to set angle, this should be set as the starting angle

double getRate() { 
	return rate; 
} // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangle(double newQ_angle) { 
	Q_angle = newQ_angle; 
}

void setQbias(double newQ_bias) { 
	Q_bias = newQ_bias;
}

void setRmeasure(double newR_measure) { 
	R_measure = newR_measure; 
}

double getQangle() { 
	return Q_angle; 
}

double getQbias() { 
	return Q_bias; 
}

double getRmeasure() { 
	return R_measure; 
}
