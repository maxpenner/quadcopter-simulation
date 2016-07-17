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

// source 0: http://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf
// source 1: http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf

#ifndef QS_CONFIG_H
#define QS_CONFIG_H

#include <math.h>



//======================================================================
//===================== LOOP TIMING ====================================
//======================================================================

// debugging with console
//#define QS_DEBUG_WITH_CONSOLE

// timer specific
#define QS_TIMER_TIME_TYPE unsigned long long
#define QS_TIMER_TIME_TYPE_SUFFIX_TMP(x) x ## ull
#define QS_TIMER_TIME_TYPE_SUFFIX(x) QS_TIMER_TIME_TYPE_SUFFIX_TMP(x)
#define QS_TIMER_STATISTICS_TYPE unsigned long long

// global simulation times in nanoseconds
#define QS_SIMULATION_END		QS_TIMER_TIME_TYPE_SUFFIX(200000000000)
#define QS_TIME_DELTA			QS_TIMER_TIME_TYPE_SUFFIX(500000)
#define QS_USER_INPUT_PERIOD	QS_TIMER_TIME_TYPE_SUFFIX(20000000)
#define QS_SENSOR_INPUT_PERIOD	QS_TIMER_TIME_TYPE_SUFFIX(5000000)
#define QS_SLEEP_SIMLATION		QS_TIMER_TIME_TYPE_SUFFIX(0)				// should be zero to run cpu at 100%, best real-time result

//#define QS_SET_THREAD_PRIORITY

//======================================================================
//===================== PHYSICAL PARAMETERS ============================
//======================================================================

// physical constants
#define GRAVITY					9.81	// m/sec^2

// magnetic field in central europe according to wikipedia
#define MAGFIELD_EARTH_X		0.2		// Gauss
#define MAGFIELD_EARTH_Y		0.0
#define MAGFIELD_EARTH_Z		-0.44 



//======================================================================
//===================== INITIAL SIMULATION CONDITIONS ==================
//======================================================================

// velocity, acceleration, angular rotation and angular acceleration are set to 0.0 at the beginning

// position and attitude initialization
#define X_START					0.0		// m
#define Y_START					0.0
#define Z_START					0.0
#define ROLL_START				0.0		// degrees
#define PITCH_START				0.0
#define YAW_START				0.0



//======================================================================
//===================== QUADCOPTER MECHANICS ===========================
//======================================================================

// frame mode (plus or x/h) -> only x/h frame is implemented, but plus can be added easily
enum QS_FRAME_MODE
{
	QS_FRAME_MODE_PL,
	QS_FRAME_MODE_XH
};
#define QS_FRAME_MODE_DEFAULT QS_FRAME_MODE_XH

// arm length
#define LENGTH_ARM				0.225	// m

// Inertia matrix and mass.
// source: http://www8.informatik.uni-wuerzburg.de/fileadmin/10030800/user_upload/quadcopter/Abschlussarbeiten/6DOF_Control_Lebedev_MA.pdf)
#define RECALCULATE_I
#ifdef RECALCULATE_I
#define CENTRAL_MASS			0.7		// kg
#define CENTRAL_MASS_RADIUS		0.1		// m
#define MOTOR_MASS				0.075	// kg
#define MASS					(CENTRAL_MASS + 4.0*MOTOR_MASS)
#else
#define MASS					0.5		// kg
#define INERTIA_X				5e-3	
#define INERTIA_Y				5e-3
#define INERTIA_Z				10e-3
#endif

// quadcopter specific air resistance
#define DRAG_CONSTANT_X			0.2
#define DRAG_CONSTANT_Y			0.2
#define DRAG_CONSTANT_Z			0.2



//======================================================================
//===================== MOTOR ELECTRONICS ==============================
//======================================================================

/* There are two possible relations between pwm duty cycle at the esc-input and rpm/thrust.
 * 
 *		duty cycle 2 rpm: linear     ----> duty cycle 2 thrust: non-linear
 *
 *		duty cycle 2 thrust: linear  ----> duty cycle 2 rpm: non-linear
 *
*/
#define MOTOR_ESC_PWM_MIN		1000.0		// pwm at min, motors not spinning
#define MOTOR_ESC_PWM_STA		1100.0		// pwm at min rpm and min thrust (motor start spinning)
#define MOTOR_ESC_PWM_MAX		2000.0		// pwm at max rpm and max thrust
enum QS_ESC_DUTY_CYCLE_THRUST_RELATION
{
	QS_ESC_DUTY_CYCLE_2_RPM_LINEAR,
	QS_ESC_DUTY_CYCLE_2_THRUST_LINEAR
};
#define QS_ESC_DUTY_CYCLE_THRUST_RELATION_DEFAULT QS_ESC_DUTY_CYCLE_2_THRUST_LINEAR

/*
 * I assume a non-linear relation beetween rpm and thrust:
 *
 *		Thrust = motor_constant * rpm ^ Q	-> 'motor_constant' and 'Q' should be aligned to measured point in equilibrium.
 *											-> Single motor equation.
 *											-> The unit is Newton.
 *											-> Example: If the quadcopter weights 0.5kg you would need a thrust of 0.5*9.81 = 4.905 Newton (all 4 motors) to hover.
 *
 * I assume a non-linear relation beetween rpm and torque:
 *
 *		Torque = torque_yaw_constant * rpm ^ QQ	-> 'torque_yaw_constant' can't be measuered easily, but estimated.
 *												-> Single motor equation.
*/
#define MOTOR_CONSTANT			9.6549e-8
#define MOTOR_EXPONENT_Q		2.1
#define TORQUE_YAW_CONSTANT		4.8274e-9
#define TORQUE_YAW_EXPONENT_QQ	2.0

// A quadcopter should be able to generate a thrust to lift twice it's own weight.
// With the motor equation which is defined above the required rpm's can be calculated.
#define MOTOR_THRUST_MIN		MASS*GRAVITY/4.0*0.0
#define MOTOR_THRUST_MAX		MASS*GRAVITY/4.0*2.0
#define MOTOR_THRUST_EQU		MASS*GRAVITY/4.0
#define MOTOR_RPM_MIN			pow(1.0/MOTOR_CONSTANT*MOTOR_THRUST_MIN, 1.0/MOTOR_EXPONENT_Q)
#define MOTOR_RPM_MAX			pow(1.0/MOTOR_CONSTANT*MOTOR_THRUST_MAX, 1.0/MOTOR_EXPONENT_Q)
#define MOTOR_RPM_EQU			pow(1.0/MOTOR_CONSTANT*MOTOR_THRUST_EQU, 1.0/MOTOR_EXPONENT_Q)

// the hoovering pwm has to be known approximately, here an offset can be defined
#define MOTOR_PWM_EQU_OFFSET	10.0

// when mode switches, where is the assumed point of equilibrium/hoovering?
inline double getPWMinPointOfEquilibirum()
{
	double pwm_equib;
	double scale = (MOTOR_ESC_PWM_MAX - MOTOR_ESC_PWM_MIN);

	if(QS_ESC_DUTY_CYCLE_THRUST_RELATION_DEFAULT == QS_ESC_DUTY_CYCLE_2_RPM_LINEAR)
		pwm_equib = (MOTOR_RPM_EQU - MOTOR_RPM_MIN) * scale / (MOTOR_RPM_MAX - MOTOR_RPM_MIN) + MOTOR_ESC_PWM_MIN;
	
	if(QS_ESC_DUTY_CYCLE_THRUST_RELATION_DEFAULT == QS_ESC_DUTY_CYCLE_2_THRUST_LINEAR)
		pwm_equib = (MOTOR_THRUST_EQU - MOTOR_THRUST_MIN) * scale / (MOTOR_THRUST_MAX - MOTOR_THRUST_MIN) + MOTOR_ESC_PWM_MIN;

	return pwm_equib + MOTOR_PWM_EQU_OFFSET;
}



//======================================================================
//===================== RECEIVER =======================================
//======================================================================

// I assume a linear relation between stick position at the transmitter and pwm duty cycle at the receiver.
// If a stick is in the lowest/leftist position		-> RECEIVER_PWM_MIN is received.
// If a stick is in the middle position				-> REICEVER_PWM_ZERO_SIGNAL is received.
// If a stick is in the highest/rightest position	-> RECEIVER_PWM_MAX is received.

// pwm settings
#define RECEIVER_PWM_MIN			1000.0
#define RECEIVER_PWM_MAX			2000.0
#define RECEIVER_PWM_ZERO_SIGNAL	(RECEIVER_PWM_MAX+RECEIVER_PWM_MIN)/2.0	

// A keyboard is non-analog input, so a key can represent only a specific pwm value.
// RECEIVER_PWM_ZERO_SIGNAL is added in all cases except for throttle.
#define RECEIVER_ROLL_KEY_PWM			300.0
#define RECEIVER_PITCH_KEY_PWM			300.0
#define RECEIVER_YAW_KEY_PWM			250.0
#define RECEIVER_THROTTLE_KEY_PWM		100.0



//======================================================================
//===================== SENSORS ========================================
//======================================================================

// sensor noise and offset parameters

// ideal sensors
/*
#define BAROMETER_OFFSET		0.0		// meters
#define BAROMETER_STANDEV		0.0	
#define MAGNETOMETER_OFFSET_X	0.0		// gauss (see definition of magnetic earth field at top)
#define MAGNETOMETER_OFFSET_Y	0.0
#define MAGNETOMETER_OFFSET_Z	0.0
#define MAGNETOMETER_STANDEV_X	0.0
#define MAGNETOMETER_STANDEV_Y	0.0
#define MAGNETOMETER_STANDEV_Z	0.0
#define GYROSCOPE_OFFSET_R		0.0		// degrees/s
#define GYROSCOPE_OFFSET_P		0.0
#define GYROSCOPE_OFFSET_Y		0.0
#define GYROSCOPE_STANDEV_R		0.0
#define GYROSCOPE_STANDEV_P		0.0
#define GYROSCOPE_STANDEV_Y		0.0
#define ACCELEROMETER_OFFSET_X	0.0		// m/s^2
#define ACCELEROMETER_OFFSET_Y	0.0
#define ACCELEROMETER_OFFSET_Z	0.0
#define ACCELEROMETER_STANDEV_X	0.0
#define ACCELEROMETER_STANDEV_Y	0.0
#define ACCELEROMETER_STANDEV_Z	0.0

#define CHIP_OFFSET_R			0.0		// degrees
#define CHIP_OFFSET_P			0.0
#define CHIP_OFFSET_Y			0.0
*/
// non-ideal sensors
#define BAROMETER_OFFSET		10.0	// meters
#define BAROMETER_STANDEV		0.5
#define MAGNETOMETER_OFFSET_X	0.01	// Gauss (see definition of magnetic earth field at top)
#define MAGNETOMETER_OFFSET_Y	0.01
#define MAGNETOMETER_OFFSET_Z	0.01
#define MAGNETOMETER_STANDEV_X	0.003
#define MAGNETOMETER_STANDEV_Y	0.003
#define MAGNETOMETER_STANDEV_Z	0.003
#define GYROSCOPE_OFFSET_R		4.0		// degrees/s
#define GYROSCOPE_OFFSET_P		2.0
#define GYROSCOPE_OFFSET_Y		3.0
#define GYROSCOPE_STANDEV_R		1.4
#define GYROSCOPE_STANDEV_P		2.5
#define GYROSCOPE_STANDEV_Y		0.5
#define ACCELEROMETER_OFFSET_X	0.2		// m/s^2
#define ACCELEROMETER_OFFSET_Y	0.1
#define ACCELEROMETER_OFFSET_Z	0.3
#define ACCELEROMETER_STANDEV_X	0.2
#define ACCELEROMETER_STANDEV_Y	0.25
#define ACCELEROMETER_STANDEV_Z	0.23

#define CHIP_OFFSET_R			-3.0	// degrees
#define CHIP_OFFSET_P			2.0
#define CHIP_OFFSET_Y			-3.5

// sensor calibration readings from horizontal position before simulation start
#define BAROMETER_CAL_READS		1000
#define MAGNETOMETER_CAL_READS	1000
#define GYROSCOPE_CAL_READS		1000
#define ACCELEROMETER_CAL_READS	1000

#endif