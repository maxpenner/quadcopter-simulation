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
#include "utils_diffequation.h"
#include "gyroscope.h"

gyroscope::gyroscope()
{	
	this->calibrated_offsets = Vector3d(0.0,0.0,0.0);
	this->calibrated_offsets_sum = Vector3d(0.0,0.0,0.0);
	this->chip_tilt = Vector3d(DEG2RAD(CHIP_OFFSET_R), DEG2RAD(CHIP_OFFSET_P), DEG2RAD(CHIP_OFFSET_Y));
	this->mag_randomGen = std::default_random_engine(rand());
}

gyroscope::~gyroscope()
{
}

void gyroscope::read_calibration_value(Vector3d xdotdot_ideal, Vector3d attitude_ideal)
{
	Vector3d thetadot_bf_new_cal_val;
	this->get_corrupted_angveloc(&thetadot_bf_new_cal_val, xdotdot_ideal, attitude_ideal);
	this->calibrated_offsets_sum += thetadot_bf_new_cal_val;
}

void gyroscope::calibrate()
{
	this->calibrated_offsets = this->calibrated_offsets_sum/GYROSCOPE_CAL_READS;
	this->calibrated_offsets_sum = Vector3d(0.0,0.0,0.0);

#ifdef QS_DEBUG_WITH_CONSOLE
	printf("*********************************************\n");
	printf("Gyroscope calibrated:\n");
	printf("gyros roll offset:  %f degrees/sec\n", RAD2DEG(this->calibrated_offsets(0)));
	printf("gyros pitch offset: %f degrees/sec\n", RAD2DEG(this->calibrated_offsets(1)));
	printf("gyros yaw offset:   %f degrees/sec\n\n", RAD2DEG(this->calibrated_offsets(2)));
#endif
}

void gyroscope::take_chip_off_quadcopter()
{
	this->chip_tilt = Vector3d(0.0, 0.0, 0.0);
}

void gyroscope::place_chip_on_quadcopter()
{
	this->chip_tilt = Vector3d(DEG2RAD(CHIP_OFFSET_R), DEG2RAD(CHIP_OFFSET_P), DEG2RAD(CHIP_OFFSET_Y));
}

void gyroscope::get_corrupted_angveloc(Vector3d *thetadot_bf_corrupted, Vector3d thetadot_ideal, Vector3d attitude_ideal)
{	
	// The angular velocities in the earth frame are perfectly known from the differential equation.
	// But the gyroscope returns angular accelerations in the body frame, therefore a transformation is needed.
	// The easiest way is to use the functions for the differential equations.
	thetadot2omega(thetadot_bf_corrupted, thetadot_ideal, attitude_ideal);

	// add chip tilt
	thetadot2omega(thetadot_bf_corrupted, *thetadot_bf_corrupted, this->chip_tilt);

	// standart normal distribution generator
	std::normal_distribution<double> distribution(0.0, 1.0);

	// add noise and an offset
	(*thetadot_bf_corrupted)(0) += distribution(this->mag_randomGen)*DEG2RAD(GYROSCOPE_STANDEV_R) + DEG2RAD(GYROSCOPE_OFFSET_R);
	(*thetadot_bf_corrupted)(1) += distribution(this->mag_randomGen)*DEG2RAD(GYROSCOPE_STANDEV_P) + DEG2RAD(GYROSCOPE_OFFSET_P);
	(*thetadot_bf_corrupted)(2) += distribution(this->mag_randomGen)*DEG2RAD(GYROSCOPE_STANDEV_Y) + DEG2RAD(GYROSCOPE_OFFSET_Y);

	// remove the offset from the calibration
	*thetadot_bf_corrupted -= this->calibrated_offsets;
}