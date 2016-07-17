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

#include "config.h"
#include "mathHelp.h"
#include "accelerometer.h"
#include "utils_diffequation.h"

accelerometer::accelerometer()
{
	this->calibrated_offsets = Vector3d(0.0,0.0,0.0);
	this->calibrated_offsets_sum = Vector3d(0.0,0.0,0.0);
	this->chip_tilt = Vector3d(DEG2RAD(CHIP_OFFSET_R), DEG2RAD(CHIP_OFFSET_P), DEG2RAD(CHIP_OFFSET_Y));
	this->mag_randomGen = std::default_random_engine(rand());
}

accelerometer::~accelerometer()
{
}

void accelerometer::read_calibration_value(Vector3d xdotdot_ideal, Vector3d attitude_ideal)
{
	Vector3d xdotdot_bf_new_cal_val;
	this->get_corrupted_accelerations(&xdotdot_bf_new_cal_val, xdotdot_ideal, attitude_ideal);
	this->calibrated_offsets_sum += xdotdot_bf_new_cal_val;
}

void accelerometer::calibrate()
{
	this->calibrated_offsets = this->calibrated_offsets_sum/ACCELEROMETER_CAL_READS;
	this->calibrated_offsets_sum = Vector3d(0.0,0.0,0.0);

	// remove gravity from axis z
	this->calibrated_offsets(2) -= GRAVITY;

#ifdef QS_DEBUG_WITH_CONSOLE
	printf("*********************************************\n");
	printf("Accelerometer calibrated:\n");
	printf("accel x offset: %f meters/(s*s)\n", this->calibrated_offsets(0));
	printf("accel y offset: %f meters/(s*s)\n", this->calibrated_offsets(1));
	printf("accel z offset: %f meters/(s*s)\n\n", this->calibrated_offsets(2));
#endif
}

void accelerometer::take_chip_off_quadcopter()
{
	this->chip_tilt = Vector3d(0.0, 0.0, 0.0);
}

void accelerometer::place_chip_on_quadcopter()
{
	this->chip_tilt = Vector3d(DEG2RAD(CHIP_OFFSET_R), DEG2RAD(CHIP_OFFSET_P), DEG2RAD(CHIP_OFFSET_Y));
}

void accelerometer::get_corrupted_accelerations(Vector3d *xdotdot_bf_corrupted, Vector3d xdotdot_ideal, Vector3d attitude_ideal)
{
	// The accelerations in the earth frame are perfectly known from the differential equation.
	// But the accelerometer returns the accelerations in the body frame, therefore a transformation is needed.
	// R is a transformation matrix from the body to the earth frame, so R.inverse() transforms from earth to body frame.
	Matrix3d R;
	rotation(&R, attitude_ideal);
	*xdotdot_bf_corrupted = R.inverse()*xdotdot_ideal;

	// If there is no acceleration in the earth frame (xdotdot_ideal = 0), the accelerometer would still measure the gravity acceleration.
	// Therefore the gravity vector has to be added.
	(*xdotdot_bf_corrupted) = (*xdotdot_bf_corrupted) + R.inverse()*Vector3d(0,0,GRAVITY);

	// add chip tilt
	rotation(&R, this->chip_tilt);
	*xdotdot_bf_corrupted = R.inverse()*(*xdotdot_bf_corrupted);

	// standart normal distribution generator
	std::normal_distribution<double> distribution(0.0, 1.0);

	// add noise and an offset
	(*xdotdot_bf_corrupted)(0) += distribution(this->mag_randomGen)*ACCELEROMETER_STANDEV_X + ACCELEROMETER_OFFSET_X;
	(*xdotdot_bf_corrupted)(1) += distribution(this->mag_randomGen)*ACCELEROMETER_STANDEV_Y + ACCELEROMETER_OFFSET_Y;
	(*xdotdot_bf_corrupted)(2) += distribution(this->mag_randomGen)*ACCELEROMETER_STANDEV_Z + ACCELEROMETER_OFFSET_Z;

	// remove the offset from the calibration
	*xdotdot_bf_corrupted -= this->calibrated_offsets;
}