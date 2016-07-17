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

#include "mathHelp.h"
#include "stabilizer.h"

#define STAB 0
#define RATE 1

#define HEIGHT_M 0
#define HEIGHTDOT_M_S 1

stabilizer::stabilizer()
{
	flight_mode = STABILIZE_HEIGHT;
	
	// all PID values are defined here:

	// roll, pitch
	pidRoll[STAB].setGains(4.5, 0.0, 0.0);
	pidRoll[RATE].setGains(1.5, 1.0, 0.0);
	pidRoll[RATE].setiTermLimit(50.0);
	pidPitch[STAB].setGains(4.5, 0.0, 0.0);
	pidPitch[RATE].setGains(1.5, 1.0, 0.0);
	pidPitch[RATE].setiTermLimit(50.0);
	
	// yaw
	pidYaw[STAB].setGains(1.5, 0.0, 0.0);
	pidYaw[RATE].setGains(2.7, 0.0, 0.0);
	pidYaw[RATE].setiTermLimit(0.0);
	yawLock = 0.0;

	// height
	pidHeight[HEIGHT_M].setGains(1.0, 0.1, 0.0);
	pidHeight[HEIGHT_M].setiTermLimit(0.5);
	pidHeight[HEIGHTDOT_M_S].setGains(200.0, 1.0, 0.0);
	pidHeight[HEIGHTDOT_M_S].setiTermLimit(100.0);
	heightLock = 0.0;
}

stabilizer::~stabilizer()
{	
}

void stabilizer::setInitYawLock(double yawInitlock)
{
	this->yawLock = RAD2DEG(yawInitlock);
}

void stabilizer::setInitHeightLock(double heightInitlock)
{
	this->heightLock = heightInitlock;
}

void stabilizer::resetIntegrals()
{
	pidRoll[STAB].zeroErrorIntegral();
	pidRoll[RATE].zeroErrorIntegral();
	pidPitch[STAB].zeroErrorIntegral();
	pidPitch[RATE].zeroErrorIntegral();
	pidYaw[STAB].zeroErrorIntegral();
	pidYaw[RATE].zeroErrorIntegral();
	pidHeight[HEIGHT_M].zeroErrorIntegral();
	pidHeight[HEIGHTDOT_M_S].zeroErrorIntegral();
}

void stabilizer::setFlightMode(stable_flight_mode flight_mode_arg)
{
	flight_mode = flight_mode_arg;
}

void stabilizer::compute_pwmDutyCycle( 	Vector4d *pwmDutyCycle,
										Vector3d RPY_is,
										Vector3d RPYDot_is,
										double height_is,
										double heightDot_is,
										double heightDotDot_is,
										double roll_rx,
										double pitch_rx,
										double yaw_rx,
										double thrust_rx)
{	
	// map rx input to target angles
	roll_rx = mapp<double>(roll_rx, RECEIVER_PWM_MIN, RECEIVER_PWM_MAX, -30.0, 30.0);
	pitch_rx = mapp<double>(pitch_rx, RECEIVER_PWM_MIN, RECEIVER_PWM_MAX, -30.0, 30.0);
	yaw_rx = mapp<double>(yaw_rx, RECEIVER_PWM_MIN, RECEIVER_PWM_MAX, -180.0, 180.0);
	yaw_rx = -yaw_rx;

	// limit roll and pitch sum to maximum
	double tilt_uncorrected = RAD2DEG(acos(cos(DEG2RAD(roll_rx))*cos(DEG2RAD(pitch_rx))));
	if(tilt_uncorrected > 30.0)
	{
		roll_rx *= 30.0/tilt_uncorrected;
		pitch_rx *= 30.0/tilt_uncorrected;
	}
	
	// convert fused angles to degrees
	Vector3d RPY_is_deg = RPY_is;
	Vector3d RPYDot_is_deg = RPYDot_is;
	for (int i = 0; i < 3; i++)
	{
		RPY_is_deg(i) = RAD2DEG(RPY_is_deg(i));
		RPYDot_is_deg(i) = RAD2DEG(RPYDot_is_deg(i));
	}

	// values to be determined
	double roll_out = 0.0, pitch_out = 0.0, yaw_out = 0.0, thrust_out = 0.0;

	// first attitude control stage
	double roll_stab_output = constrainn<double>(pidRoll[STAB].compute(roll_rx, RPY_is_deg(0)), -250.0, 250.0);
	double pitch_stab_output = constrainn<double>(pidPitch[STAB].compute(pitch_rx, RPY_is_deg(1)), -250.0, 250.0);
	double yaw_error = wrap_180(yawLock - RPY_is_deg(2));
	double yaw_stab_output = constrainn<double>(pidYaw[STAB].compute(yaw_error, 0.0), -360.0, 360.0);

	// if pilot is asking for yaw change feed signal directly to rate pid
	if(fabs(deadband<double>(yaw_rx, 5.0)) > 0.0)
	{
		yaw_stab_output = yaw_rx;
		yawLock = RPY_is_deg(2);
	}
	
	// second attitude control stage with or without body rate conversion
	#ifdef STABILIZER_BODY_RATES_CONVERSION
	// rates in body frame
	Vector3d RPYDot_is_deg_body = getBodyRatesFromEulerRates(RPY_is, RPYDot_is);
	for(int i = 0; i < 3; i++)
		RPYDot_is_deg_body(i) = RAD2DEG(RPYDot_is_deg_body(i));
	
	// target rates in body frame
	Vector3d target_body_rates = getBodyRatesFromEulerRates(RPY_is, Vector3d(DEG2RAD(roll_stab_output), DEG2RAD(pitch_stab_output), DEG2RAD(yaw_stab_output)));
	roll_stab_output = RAD2DEG(target_body_rates(0));
	pitch_stab_output = RAD2DEG(target_body_rates(1));
	yaw_stab_output = RAD2DEG(target_body_rates(2));

	roll_out = constrainn<double>(pidRoll[RATE].compute(roll_stab_output, RPYDot_is_deg_body(0)), - 500.0, 500.0);  
	pitch_out = constrainn<double>(pidPitch[RATE].compute(pitch_stab_output, RPYDot_is_deg_body(1)), -500.0, 500.0);
	yaw_out = constrainn<double>(pidYaw[RATE].compute(yaw_stab_output, RPYDot_is_deg_body(2)), -500.0, 500.0);
	#else
	roll_out = constrainn<double>(pidRoll[RATE].compute(roll_stab_output, RPYDot_is_deg(0)), - 500.0, 500.0);  
	pitch_out = constrainn<double>(pidPitch[RATE].compute(pitch_stab_output, RPYDot_is_deg(1)), -500.0, 500.0);
	yaw_out = constrainn<double>(pidYaw[RATE].compute(yaw_stab_output, RPYDot_is_deg(2)), -500.0, 500.0);
	#endif
	
	// height stabilization
	if(flight_mode == STABILIZE)
	{
		thrust_out = mapp<double>(thrust_rx, RECEIVER_PWM_MIN, RECEIVER_PWM_MAX, MOTOR_ESC_PWM_MIN, MOTOR_ESC_PWM_MAX);
		#ifdef STABILIZER_THRUST_COMPENSATION
		thrust_out = getBodyVerticalFromEarthVertical(RPY_is, thrust_out);
		#endif
		thrust_out = constrainn<double>(thrust_out, 0.0, MOTOR_ESC_PWM_MAX - 200.0);
	}
	else if(flight_mode == STABILIZE_HEIGHT)
	{
		thrust_out = STABILIZER_HOVER_THRUST_PWM;
		
		// map thrust from rx to zero centered signal (throttle stick must be moved into hover position)
		thrust_rx -= STABILIZER_HOVER_THRUST_PWM;
		
		// if pilot is asking for height change
		if(fabs(deadband<double>(thrust_rx, 10.0)) > 0.0)
		{
			thrust_out += thrust_rx;
			heightLock = height_is;
			pidHeight[HEIGHT_M].zeroErrorIntegral();
			pidHeight[HEIGHTDOT_M_S].zeroErrorIntegral();
		}
		else
		{
			// height controlled only in discrete levels
			double height_step = 0.01;
			int height_level_difference = (int) ((height_is - heightLock)/height_step);
			double height_discrete = heightLock + ((double) height_level_difference)*height_step;

			// first stage; controll height difference
			double additional_thrust = constrainn<double>(pidHeight[HEIGHT_M].compute(heightLock, height_discrete), -2.0, 2.0);

			// second state: controll climb/sink rate
			additional_thrust = pidHeight[HEIGHTDOT_M_S].compute(additional_thrust, heightDot_is);

			thrust_out += additional_thrust;
		}

		#ifdef STABILIZER_THRUST_COMPENSATION
		thrust_out = getBodyVerticalFromEarthVertical(RPY_is, thrust_out);
		#endif
		thrust_out = constrainn<double>(thrust_out, STABILIZER_HOVER_THRUST_PWM - 300.0, STABILIZER_HOVER_THRUST_PWM + 300.0);
	}
	
	// translate into pwm signal (x mode)
	double pwm0 = thrust_out + roll_out - pitch_out + yaw_out;
	double pwm1 = thrust_out - roll_out - pitch_out - yaw_out;
	double pwm2 = thrust_out - roll_out + pitch_out + yaw_out;
	double pwm3 = thrust_out + roll_out + pitch_out - yaw_out;
	
	// during flight motors cannot be turned off
	(*pwmDutyCycle)(0) = constrainn<double>(pwm0, MOTOR_ESC_PWM_STA, MOTOR_ESC_PWM_MAX);
	(*pwmDutyCycle)(1) = constrainn<double>(pwm1, MOTOR_ESC_PWM_STA, MOTOR_ESC_PWM_MAX);
	(*pwmDutyCycle)(2) = constrainn<double>(pwm2, MOTOR_ESC_PWM_STA, MOTOR_ESC_PWM_MAX);
	(*pwmDutyCycle)(3) = constrainn<double>(pwm3, MOTOR_ESC_PWM_STA, MOTOR_ESC_PWM_MAX);
}

Vector3d stabilizer::getBodyRatesFromEulerRates(Vector3d eulerAngles, Vector3d eulerRatesDesired)
{
	// Source: http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf
	double sin_phi = sin(eulerAngles(0));
	double cos_phi = cos(eulerAngles(0));
	double sin_theta = sin(eulerAngles(1));
	double cos_theta = cos(eulerAngles(1));
	
	Vector3d BodyRates;

	BodyRates(0) = eulerRatesDesired(0)                               -         sin_theta*eulerRatesDesired(2);
	BodyRates(1) =                       cos_phi*eulerRatesDesired(1) + sin_phi*cos_theta*eulerRatesDesired(2);
	BodyRates(2) =                     - sin_phi*eulerRatesDesired(1) + cos_phi*cos_theta*eulerRatesDesired(2);
	
	return BodyRates;
}

double stabilizer::getBodyVerticalFromEarthVertical(Vector3d eulerAngles, double earthVertical)
{
	return earthVertical/(cos(eulerAngles(0))*cos(eulerAngles(1)));
}