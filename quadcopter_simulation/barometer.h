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

#ifndef QS_SENSOR_BAROMETER_H
#define QS_SENSOR_BAROMETER_H

#include <random>
#include <Eigen/Dense>

using namespace Eigen;

class barometer
{
	public:
	
		barometer();
		~barometer();

		// calibration
		void read_calibration_value(double height_ideal);
		void calibrate();
		
		void get_corrupted_height(double *height_corrupted, double height_ideal);
		
	private:

		double calibrated_offset;
		double calibrated_offset_sum;
	
		std::default_random_engine mag_randomGen;
};

#endif
