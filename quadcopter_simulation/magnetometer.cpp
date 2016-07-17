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
#include "magnetometer.h"

magnetometer::magnetometer()
{
	this->calibrated_offsets = Vector3d(0.0,0.0,0.0);
	this->calibrated_offsets_sum = Vector3d(0.0,0.0,0.0);
	this->chip_tilt = Vector3d(DEG2RAD(CHIP_OFFSET_R), DEG2RAD(CHIP_OFFSET_P), DEG2RAD(CHIP_OFFSET_Y));
	this->magneticEarthFieldVector = Vector3d(MAGFIELD_EARTH_X, MAGFIELD_EARTH_Y, MAGFIELD_EARTH_Z);
	this->mag_randomGen = std::default_random_engine(rand());
}

magnetometer::~magnetometer()
{
}

void magnetometer::read_calibration_value(Vector3d attitude_ideal)
{
	Vector3d magField_new_cal_val;
	this->get_corrupted_MagneticVectorBodyFrame(&magField_new_cal_val, attitude_ideal);
	this->calibrated_offsets_sum += magField_new_cal_val;
}

void magnetometer::calibrate()
{
	this->calibrated_offsets = this->calibrated_offsets_sum/MAGNETOMETER_CAL_READS;
	this->calibrated_offsets -= this->magneticEarthFieldVector;
	this->calibrated_offsets_sum = Vector3d(0.0,0.0,0.0);

#ifdef QS_DEBUG_WITH_CONSOLE
	printf("*********************************************\n");
	printf("Magnetometer calibrated:\n");
	printf("magne x offset: %f gauss\n", this->calibrated_offsets(0));
	printf("magne y offset: %f gauss\n", this->calibrated_offsets(1));
	printf("magne z offset: %f gauss\n\n", this->calibrated_offsets(2));
#endif
}

void magnetometer::take_chip_off_quadcopter()
{
	this->chip_tilt = Vector3d(0.0, 0.0, 0.0);
}

void magnetometer::place_chip_on_quadcopter()
{
	this->chip_tilt = Vector3d(DEG2RAD(CHIP_OFFSET_R), DEG2RAD(CHIP_OFFSET_P), DEG2RAD(CHIP_OFFSET_Y));
}

void magnetometer::get_corrupted_MagneticVectorBodyFrame(Vector3d *magField_bf_corrupted, Vector3d attitude_ideal)
{		
	Matrix3d R;
	rotation(&R, attitude_ideal);

	// from earth to body frame
	*magField_bf_corrupted = R.inverse()*magneticEarthFieldVector;

	// add chip tilt
	rotation(&R, this->chip_tilt);
	*magField_bf_corrupted = R.inverse()*(*magField_bf_corrupted);

	// standart normal distribution generator
	std::normal_distribution<double> distribution(0.0, 1.0);

	(*magField_bf_corrupted)(0) += distribution(this->mag_randomGen)*MAGNETOMETER_OFFSET_X + MAGNETOMETER_STANDEV_X;
	(*magField_bf_corrupted)(1) += distribution(this->mag_randomGen)*MAGNETOMETER_OFFSET_Y + MAGNETOMETER_STANDEV_Y;
	(*magField_bf_corrupted)(2) += distribution(this->mag_randomGen)*MAGNETOMETER_OFFSET_Z + MAGNETOMETER_STANDEV_Z;
}