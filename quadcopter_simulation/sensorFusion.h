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

#ifndef QS_SENSORFUSION_H
#define QS_SENSORFUSION_H

#include <Eigen/Dense>
#include "config.h"
#include "mathHelp.h"
#include "complementaryFilter.h"

#define SFUS_SENSOR_TILT_DROPS	200
#define SFUS_SENSOR_TILT_READS	600

#define SFUS_INIT_ACCEL_DROPS	50
#define SFUS_INIT_ACCEL_READS	300
#define SFUS_INIT_GYROS_DROPS	50
#define SFUS_INIT_GYROS_READS	300
#define SFUS_INIT_MAGNE_DROPS	50
#define SFUS_INIT_MAGNE_READS	300
#define SFUS_INIT_BAROM_DROPS	50
#define SFUS_INIT_BAROM_READS	300

#define SFUS_ACCEL_BUF			10
#define SFUS_GYROS_BUF			10
#define SFUS_MAGNE_BUF			10
#define SFUS_BAROM_BUF			10

// Roll and pitch are adjusted proportionately to the total tilt angle.
// Sampling period is a privat class member.
#define SFUS_COMPF_RP_TAU_MIN	2.5f//0.0001//0.999999//
#define SFUS_COMPF_RP_TAU_MAX	20.0f//0.001//0.99999999//
#define SFUS_COMPF_RP_LIM_TILT	10.0
#define SFUS_COMPF_YAW_A		1.0
#define SFUS_COMPF_HEIGHTDOT_A	0.99667777
#define SFUS_MA_HEIGHT			20

// measured on xx.xx.xxxx
// Fixed values on Arduino, here the values are recalculated each time the simulation starts.
#define SFUS_SENSOR_TILT_R		0.0
#define SFUS_SENSOR_TILT_P		0.0
#define SFUS_SENSOR_TILT_Y		0.0

#define SFUS_USE_GYROS_CALIB

using namespace Eigen;

class sensorFusion
{
	public:

		sensorFusion();
		~sensorFusion();

		// chip on quadcopter tilt compensation
		bool writeSensorTiltCalibData(Vector3d accel, Vector3d magne);
		
		// initial attitude estimation
		bool writeInitData(Vector3d accel, Vector3d gyros, Vector3d magne, Vector3d barom);
		
		// input of raw sensor data
		void writeData(Vector3d accel, Vector3d gyros, Vector3d magne, Vector3d barom);
		
		Vector3d getRPY();
		Vector3d getRPYDot();
		double getHeight();
		double getHeightDot();
		double getHeightDotDot();
		
	private:
	
		Vector3d initSensorTiltAccelSum, initSensorTiltMagneSum;
		int initSensorTiltReadsCnt;
	
		Vector3d initAccelSum, initGyrosSum, initMagneSum, initBaromSum;
		int initAccelReadsCnt, initGyrosReadsCnt, initMagneReadsCnt, initBaromReadsCnt;
	
		// memory on stack -> do not touch!
		// Arduino does no like heap memory, on windows it does no matter.
		// But it is kept for simplicity.
		Vector3d accelDataStack[SFUS_ACCEL_BUF];
		Vector3d gyrosDataStack[SFUS_GYROS_BUF];
		Vector3d magneDataStack[SFUS_MAGNE_BUF];
		Vector3d baromDataStack[SFUS_BAROM_BUF];	
	
		// ring buffers will be connected to the stack memory
		ringBuffer<Vector3d> accelData, gyrosData, magneData, baromData;
		
		// time between two calls
		double dT;
		
		// filters for roll, pitch, yaw and heightdot
		double rollPitchA_minimum;
		double rollPitchA_maximum;
		complementaryFilter cf[4];
		
		// accel tilt compensation in rad
		Vector3d sensorTiltCalib;
		
		// gyro calibration in degrees/s
		Vector3d gyrosCalib;
		
		// fused attitude and altitude
		Vector3d RPY;
		Vector3d RPYDot;
		double height;
		double heightDot;
		double heightDotDot;
		
		// transformation functions
		double getTiltProportionalA();
		Vector3d getEulerAnglesFromAccel(Vector3d accelBodyFrame);
		double getEulerYawFromMagne(Vector3d magneBodyFrame);
		Vector3d getVectorFromBody2EarthFrame(Vector3d vecInBodyFrame, Vector3d attitude);
		Vector3d getRatesFromBody2EarthFrame(Vector3d ratesInBodyFrame, Vector3d attitude);
};

#endif