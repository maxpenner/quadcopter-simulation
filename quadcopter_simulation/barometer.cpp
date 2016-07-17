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
#include "barometer.h"

barometer::barometer()
{
	this->calibrated_offset = 0.0;
	this->calibrated_offset_sum = 0.0;
	this->mag_randomGen = std::default_random_engine(rand());
}

barometer::~barometer()
{
}

void barometer::read_calibration_value(double height_ideal)
{
	double height_new_cal_val;
	this->get_corrupted_height(&height_new_cal_val, height_ideal);
	this->calibrated_offset_sum += height_new_cal_val;
}

void barometer::calibrate()
{
	this->calibrated_offset = this->calibrated_offset_sum/BAROMETER_CAL_READS;
	this->calibrated_offset_sum = 0.0;

#ifdef QS_DEBUG_WITH_CONSOLE
	printf("*********************************************\n");
	printf("Barometer calibrated:\n");
	printf("barom height offset: %f meters\n\n", this->calibrated_offset);
#endif
}

void barometer::get_corrupted_height(double *height_corrupted, double height_ideal)
{
	// standart normal distribution generator
	std::normal_distribution<double> distribution(0.0, 1.0);
	
	// add noise and an offset
	*height_corrupted = height_ideal + distribution(this->mag_randomGen)*BAROMETER_STANDEV + BAROMETER_OFFSET;

	// remove the offset from the calibration
	*height_corrupted -= this->calibrated_offset;
}