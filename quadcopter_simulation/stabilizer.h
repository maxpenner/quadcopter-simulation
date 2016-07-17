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

#ifndef QS_CONTROL_CONTROLLER_H
#define QS_CONTROL_CONTROLLER_H

#include <vector>
#include <Eigen/Dense>
#include "config.h"
#include "PID.h"

using namespace Eigen;

// measured on xx.03.2016
#define STABILIZER_HOVER_THRUST_PWM			getPWMinPointOfEquilibirum() //1500.0f

#define STABILIZER_BODY_RATES_CONVERSION
#define STABILIZER_THRUST_COMPENSATION

typedef enum
{
	STABILIZE = 0,
	STABILIZE_HEIGHT = 1
}stable_flight_mode;

class stabilizer
{
	public:
	
		stabilizer();
		~stabilizer();
		
		void setInitYawLock(double yawInitlock);
		void setInitHeightLock(double heightInitlock);
		void resetIntegrals();
		void setFlightMode(stable_flight_mode flight_mode_arg);
		void stabilizer::compute_pwmDutyCycle( 	Vector4d *pwmDutyCycle,
												Vector3d RPY_is,
												Vector3d RPYDot_is,
												double height_is,
												double heightDot_is,
												double heightDotDot_is,
												double roll_rx,
												double pitch_rx,
												double yaw_rx,
												double thrust_rx);
		
	private:
	
		stable_flight_mode flight_mode;

		// two stage PID cascade
		PID pidRoll[2];
		PID pidPitch[2];
		
		// first pid for locked yaw, second pid for yaw rotation
		PID pidYaw[2];
		double yawLock;

		// first pid for locked height
		PID pidHeight[2];
		double heightLock;
		
		// transformation function
		Vector3d getBodyRatesFromEulerRates(Vector3d eulerAngles, Vector3d eulerRatesDesired);
		double getBodyVerticalFromEarthVertical(Vector3d eulerAngles, double earthVertical);
};

#endif