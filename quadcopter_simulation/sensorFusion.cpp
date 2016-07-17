/*

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <math.h>
#include "sensorFusion.h"

#define ROLL 0
#define PITCH 1
#define YAW 2
#define HEIGHTDOT 3

sensorFusion::sensorFusion()
{
	sensorTiltCalib = Vector3d(SFUS_SENSOR_TILT_R, SFUS_SENSOR_TILT_P, SFUS_SENSOR_TILT_Y);
	initSensorTiltAccelSum = Vector3d(0.0,0.0,0.0);
	initSensorTiltMagneSum = Vector3d(0.0,0.0,0.0);
	initSensorTiltReadsCnt = 0;
	
	initAccelSum = Vector3d(0.0,0.0,0.0);
	initGyrosSum = Vector3d(0.0,0.0,0.0);
	initMagneSum = Vector3d(0.0,0.0,0.0);
	initBaromSum = Vector3d(0.0,0.0,0.0);
	initAccelReadsCnt = 0;
	initGyrosReadsCnt = 0;
	initMagneReadsCnt = 0;
	initBaromReadsCnt = 0;
	
	// couple ring buffers with stack memory
	accelData = ringBuffer<Vector3d>(accelDataStack, SFUS_ACCEL_BUF);
	gyrosData = ringBuffer<Vector3d>(gyrosDataStack, SFUS_GYROS_BUF);
	magneData = ringBuffer<Vector3d>(magneDataStack, SFUS_MAGNE_BUF);
	baromData = ringBuffer<Vector3d>(baromDataStack, SFUS_BAROM_BUF);
	
	this->dT = QS_SENSOR_INPUT_PERIOD/1000000000.0;

	// the factor tau is variable depending on quadcopter angle
	rollPitchA_minimum = SFUS_COMPF_RP_TAU_MIN/(SFUS_COMPF_RP_TAU_MIN + dT);
	rollPitchA_maximum = SFUS_COMPF_RP_TAU_MAX/(SFUS_COMPF_RP_TAU_MAX + dT);

	cf[ROLL].setTauViaA(rollPitchA_minimum);
	cf[PITCH].setTauViaA(rollPitchA_minimum);
	cf[YAW].setTauViaA(SFUS_COMPF_YAW_A);
	cf[HEIGHTDOT].setTauViaA(SFUS_COMPF_HEIGHTDOT_A);
}

sensorFusion::~sensorFusion()
{
}

bool sensorFusion::writeSensorTiltCalibData(Vector3d accel, Vector3d magne)
{
	initSensorTiltReadsCnt++;
	if(initSensorTiltReadsCnt > SFUS_SENSOR_TILT_DROPS && initSensorTiltReadsCnt <= (SFUS_SENSOR_TILT_DROPS + SFUS_SENSOR_TILT_READS))
	{
		initSensorTiltAccelSum += accel;
		initSensorTiltMagneSum += magne;
	}

	// data collection finished
	if(initSensorTiltReadsCnt >= (SFUS_SENSOR_TILT_DROPS + SFUS_SENSOR_TILT_READS))
	{
		initSensorTiltAccelSum /= SFUS_SENSOR_TILT_READS;
		initSensorTiltMagneSum /= SFUS_SENSOR_TILT_READS;
		
		// reset calibration to zero
		sensorTiltCalib = Vector3d(0.0,0.0,0.0);

		// quadcopter stands flat on the ground and is directed to magnetic north
		RPY = Vector3d(0.0,0.0,0.0);
		
		// calculate roll and pitch compensation from accelerometer
		sensorTiltCalib = getEulerAnglesFromAccel(initSensorTiltAccelSum);
		sensorTiltCalib(2) = 0.0;
		
		// compensate magnetometer roll and pitch tilt on quadcopter
		Vector3d initSensorTiltMagneSumRPCompensated = getVectorFromBody2EarthFrame(initSensorTiltMagneSum, sensorTiltCalib);
		
		// calculate yaw tilt of magnetometer relative to quadcopter
		sensorTiltCalib(2) = getEulerYawFromMagne(initSensorTiltMagneSumRPCompensated);

#ifdef QS_DEBUG_WITH_CONSOLE
		printf("*********************************************\n");
		printf("Sensor chip tilt calibrated:\n");
		printf("roll:  %f\n", RAD2DEG(sensorTiltCalib(0)));
		printf("pitch: %f\n", RAD2DEG(sensorTiltCalib(1)));
		printf("yaw:   %f\n\n", RAD2DEG(sensorTiltCalib(2)));
#endif
		
		return true;		
	}	
	
	return false;
}

bool sensorFusion::writeInitData(Vector3d accel, Vector3d gyros, Vector3d magne, Vector3d barom)
{
	initAccelReadsCnt++;
	if(initAccelReadsCnt > SFUS_INIT_ACCEL_DROPS && initAccelReadsCnt <= (SFUS_INIT_ACCEL_DROPS + SFUS_INIT_ACCEL_READS))
		initAccelSum += accel;
		
	initGyrosReadsCnt++;
	if(initGyrosReadsCnt > SFUS_INIT_GYROS_DROPS && initGyrosReadsCnt <= (SFUS_INIT_GYROS_DROPS + SFUS_INIT_GYROS_READS))
		initGyrosSum += gyros;
	
	initMagneReadsCnt++;
	if(initMagneReadsCnt > SFUS_INIT_MAGNE_DROPS && initMagneReadsCnt <= (SFUS_INIT_MAGNE_DROPS + SFUS_INIT_MAGNE_READS))
		initMagneSum += magne;
	
	initBaromReadsCnt++;
	if(initBaromReadsCnt > SFUS_INIT_BAROM_DROPS && initBaromReadsCnt <= (SFUS_INIT_BAROM_DROPS + SFUS_INIT_BAROM_READS))
		initBaromSum += barom;

	// data collection finished
	if(initAccelReadsCnt >= (SFUS_INIT_ACCEL_DROPS + SFUS_INIT_ACCEL_READS)
		&& initGyrosReadsCnt >= (SFUS_INIT_GYROS_DROPS + SFUS_INIT_GYROS_READS)
			&& initMagneReadsCnt >= (SFUS_INIT_MAGNE_DROPS + SFUS_INIT_MAGNE_READS)
				&& initBaromReadsCnt >= (SFUS_INIT_BAROM_DROPS + SFUS_INIT_BAROM_READS))
	{
		initAccelSum /= SFUS_INIT_ACCEL_READS;
		initGyrosSum /= SFUS_INIT_GYROS_READS;
		initMagneSum /= SFUS_INIT_MAGNE_READS;
		initBaromSum /= SFUS_INIT_BAROM_READS;
		
		// compensate chip tilt
		initAccelSum = getVectorFromBody2EarthFrame(initAccelSum, sensorTiltCalib);
		initMagneSum = getVectorFromBody2EarthFrame(initMagneSum, sensorTiltCalib);
		
		// determine initial values
		RPY = getEulerAnglesFromAccel(initAccelSum);
		RPY(2) = getEulerYawFromMagne(initMagneSum);
		RPYDot = Vector3d(0.0, 0.0, 0.0);
		height = initBaromSum(2);
		heightDot = 0.0;
		heightDotDot = 0.0;

#ifdef QS_DEBUG_WITH_CONSOLE
		printf("*********************************************\n");
		printf("Initial attitude:\n");
		printf("roll:  %f\n", RAD2DEG(RPY(0)));
		printf("pitch: %f\n", RAD2DEG(RPY(1)));
		printf("yaw:  %f\n\n", RAD2DEG(RPY(2)));
#endif
		
		// the gyroscope is recalibrated at each start up
		#ifdef SFUS_USE_GYROS_CALIB
		gyrosCalib = initGyrosSum;
		#else
		gyrosCalib = Vector3d(0.0, 0.0, 0.0);
		#endif

#ifdef QS_DEBUG_WITH_CONSOLE
		printf("*********************************************\n");
		printf("Gyroscope calibrated in sensor fusion:\n");
		printf("gyros x offset: %f\n", RAD2DEG(gyrosCalib(0)));
		printf("gyros x offset: %f\n", RAD2DEG(gyrosCalib(1)));
		printf("gyros x offset: %f\n\n", RAD2DEG(gyrosCalib(2)));
#endif
		
		// push values into buffers
		for(int i=0; i<SFUS_ACCEL_BUF; i++) accelData.pushNewElem(initAccelSum);
		for(int i=0; i<SFUS_GYROS_BUF; i++) gyrosData.pushNewElem(initGyrosSum);
		for(int i=0; i<SFUS_MAGNE_BUF; i++) magneData.pushNewElem(initMagneSum);
		for(int i=0; i<SFUS_BAROM_BUF; i++) baromData.pushNewElem(initBaromSum);

		// init complementary filter A/Tau factor
		float currentA = getTiltProportionalA();
		cf[ROLL].setTauViaA(currentA);
		cf[PITCH].setTauViaA(currentA);
		
		// init complementary filter angles
		cf[ROLL].setCombinedEstimation(RPY(0));
		cf[PITCH].setCombinedEstimation(RPY(1));
		cf[YAW].setCombinedEstimation(RPY(2));
		cf[HEIGHTDOT].setCombinedEstimation(heightDot);
		
		return true;		
	}	
	
	return false;
}

void sensorFusion::writeData(Vector3d accel, Vector3d gyros, Vector3d magne, Vector3d barom)
{
	// compensate chip tilt for all sensors
	accel = getVectorFromBody2EarthFrame(accel, sensorTiltCalib);
	gyros = getRatesFromBody2EarthFrame(gyros-gyrosCalib, sensorTiltCalib);
	magne = getVectorFromBody2EarthFrame(magne, sensorTiltCalib);
	
	// push tilt compensated values to container
	accelData.pushNewElem(accel);
	gyrosData.pushNewElem(gyros);
	magneData.pushNewElem(magne);
	baromData.pushNewElem(barom);
	
	// transform input from sensors into earth frame
	Vector3d accelEulertmp = getEulerAnglesFromAccel(accel);
	Vector3d gyrosEulerRatestmp = getRatesFromBody2EarthFrame(gyros, RPY);
	Vector3d accelAccelerationtmp = getVectorFromBody2EarthFrame(accel, RPY);

	// adjust complementary filter
	double currentA = getTiltProportionalA();
	cf[ROLL].setTauViaA(currentA);
	cf[PITCH].setTauViaA(currentA);
	
	// filtering for roll and pitch
	RPY(0) = cf[ROLL].getCombinedEstimation(accelEulertmp(0), gyrosEulerRatestmp(0));
	RPY(1) = cf[PITCH].getCombinedEstimation(accelEulertmp(1), gyrosEulerRatestmp(1));
	RPY(2) = cf[YAW].getCombinedEstimation(getEulerYawFromMagne(magne), gyrosEulerRatestmp(2));
	
	// yaw is +/- Pi
	RPY(2) = wrap_Pi(RPY(2));
	cf[YAW].setCombinedEstimation(RPY(2));
	
	// rotation rates from calibrated gyroscope
	RPYDot = gyrosEulerRatestmp;
	
	// apply moving average filter for height
	height = 0.0;
	double height_old = 0.0;
	for(int i=0; i<SFUS_MA_HEIGHT; i++)
	{
		Vector3d tmp;
		
		baromData.getNthElem(tmp, i);
		height += tmp(2);
		
		baromData.getNthElem(tmp, i+1);
		height_old += tmp(2);
	}
	height /= SFUS_MA_HEIGHT;
	height_old /= SFUS_MA_HEIGHT;
	
	// vertical speed
	heightDot = cf[HEIGHTDOT].getCombinedEstimation((height - height_old)/dT, accelAccelerationtmp(2) - GRAVITY);

	// vertical acceleration
	heightDotDot = accelAccelerationtmp(2) - GRAVITY;
}

Vector3d sensorFusion::getRPY()
{
	return RPY;
}

Vector3d sensorFusion::getRPYDot()
{
	return RPYDot;
}

double sensorFusion::getHeight()
{
	return height;
}

double sensorFusion::getHeightDot()
{
	return heightDot;
}

double sensorFusion::getHeightDotDot()
{
	return heightDotDot;
}

double sensorFusion::getTiltProportionalA()
{	
	double T_total = RAD2DEG((acos(cos(RPY(0))*cos(RPY(1)))));
	T_total = constrainn<double>(T_total, 0.0f, SFUS_COMPF_RP_LIM_TILT);
	
	return mapp<double>(T_total, 0.0f, SFUS_COMPF_RP_LIM_TILT, rollPitchA_minimum, rollPitchA_maximum);
}

Vector3d sensorFusion::getEulerAnglesFromAccel(Vector3d accelBodyFrame)
{
	// Source: https://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
	double roll_accel = (accelBodyFrame(2) != 0.0) ? atan(accelBodyFrame(1)/accelBodyFrame(2)) : 0.0;
	double tmp = sqrt(accelBodyFrame(1)*accelBodyFrame(1) + accelBodyFrame(2)*accelBodyFrame(2));
	double pitch_accel = (tmp != 0.0) ? atan(-accelBodyFrame(0)/tmp) : 0.0;	

	return Vector3d(roll_accel, pitch_accel, 0.0);	
}

double sensorFusion::getEulerYawFromMagne(Vector3d magneBodyFrame)
{
	// tilt compensation without yaw
	// Source: http://www.chrobotics.com/library/understanding-euler-angles
	// Source: http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
	Vector3d magneEarthFrame = getVectorFromBody2EarthFrame(magneBodyFrame, Vector3d(RPY(0), RPY(1), 0.0f));
	
	// Source: https://www.adafruit.com/datasheets/AN203_Compass_Heading_Using_Magnetometers.pdf
	double orientation_tmp;
	if(magneEarthFrame(1) == 0.0)
	{
		if(magneEarthFrame(0) > 0.0)
			orientation_tmp = 0.0;
		else if(magneEarthFrame(0) < 0.0)
			orientation_tmp = Pi;
	}
	else if(magneEarthFrame(1) > 0.0)
	{
		orientation_tmp = Pi/2.0 - atan(magneEarthFrame(0)/magneEarthFrame(1));
	}
	else if(magneEarthFrame(2) < 0.0)
	{
		orientation_tmp = 1.5*Pi - atan(magneEarthFrame(0)/magneEarthFrame(1));
	}

	// transform from 0-2*Pi to +/-Pi
	return wrap_Pi(-orientation_tmp);
}

Vector3d sensorFusion::getVectorFromBody2EarthFrame(Vector3d vecInBodyFrame, Vector3d attitude)
{
	// The angles are negative, but the minuses are itegrated into the matrix.
	// Source: http://www.chrobotics.com/library/understanding-euler-angles
	double cos_roll = cos(attitude(0));
	double sin_roll = sin(attitude(0));
	double cos_pitch = cos(attitude(1));
	double sin_pitch = sin(attitude(1));
	double cos_yaw = cos(attitude(2));
	double sin_yaw = sin(attitude(2));
	
	Vector3d vecInEarthFrame;
	
	vecInEarthFrame(0) = cos_yaw*cos_pitch*vecInBodyFrame(0) + (cos_yaw*sin_pitch*sin_roll-sin_yaw*cos_roll)*vecInBodyFrame(1) + (cos_yaw*sin_pitch*cos_roll+sin_yaw*sin_roll)*vecInBodyFrame(2);
	vecInEarthFrame(1) = sin_yaw*cos_pitch*vecInBodyFrame(0) + (sin_yaw*sin_pitch*sin_roll+cos_yaw*cos_roll)*vecInBodyFrame(1) + (sin_yaw*sin_pitch*cos_roll-cos_yaw*sin_roll)*vecInBodyFrame(2);
	vecInEarthFrame(2) =        -sin_pitch*vecInBodyFrame(0) +          cos_pitch*sin_roll                  *vecInBodyFrame(1) +          cos_pitch*cos_roll                  *vecInBodyFrame(2);
	
	return vecInEarthFrame;	
}

Vector3d sensorFusion::getRatesFromBody2EarthFrame(Vector3d ratesInBodyFrame, Vector3d attitude)
{
	/* Transform the rates from the body frame into the earth frame.
	 * To do so, you need to know the current roll and pitch.
	 * The angles are negative, but the minuses are itegrated in the matrix.
	 * Can be ignored in a small angle approximation.
	 * Source: http://www.chrobotics.com/library/understanding-euler-angles
	 * Source: http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf
	 */
	
	double sin_roll = sin(attitude(0));
	double cos_roll = cos(attitude(0));
	double tan_pitch = tan(attitude(1));
	double cos_pitch = cos(attitude(1));
	
	Vector3d ratesInEarthFrame;
	
	ratesInEarthFrame(0) = ratesInBodyFrame(0) + sin_roll*tan_pitch*ratesInBodyFrame(1) + cos_roll*tan_pitch*ratesInBodyFrame(2);
	ratesInEarthFrame(1) =                       cos_roll          *ratesInBodyFrame(1) - sin_roll          *ratesInBodyFrame(2);
	ratesInEarthFrame(2) =                       sin_roll/cos_pitch*ratesInBodyFrame(2) + cos_roll/cos_pitch*ratesInBodyFrame(2);

	return ratesInEarthFrame;
}