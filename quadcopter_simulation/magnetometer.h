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

#ifndef QS_SENSOR_MAGNETOMETER_H
#define QS_SENSOR_MAGNETOMETER_H

#include <random>
#include <Eigen/Dense>

using namespace Eigen;

class magnetometer
{
	public:
	
		magnetometer();
		~magnetometer();

		// calibration
		void read_calibration_value(Vector3d attitude_ideal);
		void calibrate();

		// chip tilt calibration
		void take_chip_off_quadcopter();
		void place_chip_on_quadcopter();

		void get_corrupted_MagneticVectorBodyFrame(Vector3d *magField_bf_corrupted, Vector3d attitude_ideal);
		
	private:

		Vector3d calibrated_offsets;
		Vector3d calibrated_offsets_sum;

		Vector3d chip_tilt;

		Vector3d magneticEarthFieldVector;

		std::default_random_engine mag_randomGen;
};

#endif