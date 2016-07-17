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

#ifndef QS_TIMER_TIMER_H
#define QS_TIMER_TIMER_H

#include <time.h>
#include "config.h"

enum SIMULATION_STATE
{
	USER_INPUT = 0,
	SENSOR_INPUT = 1,
	SOLVE_DIFF_EQUA = 2,
	SLEEP_SIMULATION = 3,
};

class timer
{
	public:
	
		timer(QS_TIMER_TIME_TYPE user_input_period_arg, QS_TIMER_TIME_TYPE sensor_input_period_arg);
		~timer();
		
		// elapsed time since timer initialization
		QS_TIMER_TIME_TYPE get_simulationTimeElapsed();

		// time to be calculate
		QS_TIMER_TIME_TYPE get_simulationTime2compute();

		// get current state of timer
		enum SIMULATION_STATE get_currState();

		// set next state of timer
		void set_nextState();

		// set to true at the beginning
		void set_realTimeAlignement(bool real_time_aligned_arg);

		// print data to check if simulation is correct
		void print_statistics();
		
	private:
		
		enum SIMULATION_STATE STATE;
	
		// simulation timing
		QS_TIMER_TIME_TYPE user_input_period;
		QS_TIMER_TIME_TYPE sensor_input_period;

		// time counters
		QS_TIMER_TIME_TYPE start_time;
		QS_TIMER_TIME_TYPE simulation_time;
		QS_TIMER_TIME_TYPE next_user_input;
		QS_TIMER_TIME_TYPE next_sensor_input;
		QS_TIMER_TIME_TYPE simulation_time2compute;

		// variable that can be used for debugging
		QS_TIMER_TIME_TYPE old_now;
		
		// variables for statistics
		QS_TIMER_STATISTICS_TYPE user_input_called;
		QS_TIMER_STATISTICS_TYPE sensor_input_called;
		QS_TIMER_STATISTICS_TYPE sleep_called;

		// get current absolute time since epoch in nanoseconds
		QS_TIMER_TIME_TYPE get_curr_ntime();

		// for optimazation real time alignement is not needed
		bool real_time_aligned;
};

#endif