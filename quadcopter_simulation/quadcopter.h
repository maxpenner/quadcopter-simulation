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

#ifndef QS_QUADCOPTER_H
#define QS_QUADCOPTER_H

class quadcopter
{
	public:
	
		quadcopter();
		~quadcopter();

		// function to extract frame mode: 0 is + mode and 1 is x/h mode
		int get_frame_mode();

		// functions to extract attitude and other data of quadcopter: index mapping is 0 -> x, 1 -> y, 2 -> z
		double get_position(int index);
		double get_speed(int index);
		double get_attitude(int index);
		double get_motor_rpm(int index);
		double get_up_vector(int index);
		double get_direction_vector(int index);

		// functions to change and test simulation parameters
		bool startSimulation();
		bool stopSimulation();
		bool simulationRunning();
		
	private:

		// implementation class
		class quadcopterImpl;
		quadcopterImpl* qcimpl;
};

#endif