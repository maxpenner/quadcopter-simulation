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

#ifndef QS_RECEIVER_H
#define QS_RECEIVER_H

#include <Eigen/Dense>

using namespace Eigen;

class receiver
{
	public:
	
		receiver();
		~receiver();
		
		// in radians
		void get_desired_theta(Vector3d &theta_d);

		// in meter per second
		void get_desired_throttle(double &throttle);

		// always return zero
		void block_receiver(bool blocked);
		
	private:
	
		// pwm signals when a certain key is pressed
		double roll_pwm;
		double pitch_pwm;
		double yaw_pwm;
		double throttle_pwm;

		// always return zero
		bool output_blocked;

		// functions
		bool keypressed(int keyvalue);
};

#endif