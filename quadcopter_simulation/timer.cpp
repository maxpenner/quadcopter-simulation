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

#include <stdio.h>
#include <chrono>
#include "timer.h"

timer::timer(QS_TIMER_TIME_TYPE user_input_period_arg, QS_TIMER_TIME_TYPE sensor_input_period_arg)
{
	this->STATE	= USER_INPUT;

	this->user_input_period = user_input_period_arg;
	this->sensor_input_period = sensor_input_period_arg;

	this->start_time = get_curr_ntime();
	this->simulation_time = start_time;
	this->next_user_input = start_time;
	this->next_sensor_input = start_time;
	this->simulation_time2compute = 0;

	this->old_now = start_time;
	
	this->user_input_called = 0;
	this->sensor_input_called = 0;
	this->sleep_called = 0;

	this->real_time_aligned	= true;
};

timer::~timer()
{
};

QS_TIMER_TIME_TYPE timer::get_simulationTimeElapsed()
{
	return this->simulation_time - this->start_time;
};

QS_TIMER_TIME_TYPE timer::get_simulationTime2compute()
{
	return this->simulation_time2compute;
};

enum SIMULATION_STATE timer::get_currState(void)
{
	return this->STATE;
};

void timer::set_nextState(void)
{
	QS_TIMER_TIME_TYPE now_temp = get_curr_ntime();
	
	// if real time is ahead of simulation time, try to catch up by advancing simulation
	if(now_temp >= this->simulation_time || this->real_time_aligned == false)
	{
		QS_TIMER_TIME_TYPE time_to_next_user_input = this->next_user_input - this->simulation_time;
		QS_TIMER_TIME_TYPE time_to_next_sensor_input = this->next_sensor_input - this->simulation_time;
		
		// decide which state comes next
		if(time_to_next_user_input == 0)
		{
			// uncomment to check actual call rate
			/*
			QS_TIMER_TIME_TYPE int_diff = now_temp - old_now;
			double double_diff = (double)int_diff;
			double freq = 1e9 / double_diff;
			this->old_now = now_temp;
			printf("f: %f\n\n", freq);
			*/

			this->STATE = USER_INPUT;
			this->next_user_input += this->user_input_period;
			this->user_input_called++;
		}
		else if(time_to_next_sensor_input == 0)
		{
			// uncomment to check actual call rate
			/*
			QS_TIMER_TIME_TYPE int_diff = now_temp - old_now;
			double double_diff = (double) int_diff;
			double freq = 1e9 / double_diff;
			this->old_now = now_temp;
			printf("f: %f\n\n", freq);
			*/

			this->STATE = SENSOR_INPUT;
			this->next_sensor_input += this->sensor_input_period;
			this->sensor_input_called++;
		}
		else
		{
			this->STATE = SOLVE_DIFF_EQUA;

			// minimum time that has to be computed until next user/sensor input
			if(time_to_next_user_input < time_to_next_sensor_input)
				this->simulation_time2compute = time_to_next_user_input;
			else
				this->simulation_time2compute = time_to_next_sensor_input;

			this->simulation_time += simulation_time2compute;
		}
	}
	// If simulation time is ahead of real time, simulation cannot be progressed, therefore sleep.
	// This will take some time and after one or more sleep-calls, the real time will be ahead of simulation time.
	else
	{
		this->STATE = SLEEP_SIMULATION;
		this->sleep_called++;
	}
};

void timer::set_realTimeAlignement(bool real_time_aligned_arg)
{
	this->real_time_aligned = real_time_aligned_arg;
}

void timer::print_statistics(void)
{	
	printf("\n=========================================\n");
	printf("Statistics for Timer:\n");
	printf("elapsed simulation time:  %llu\n", get_simulationTimeElapsed());
	printf("user_input_called:        %d\n", user_input_called);
	printf("sensor_input_called:      %d\n", sensor_input_called);
	printf("sleep_called:             %d\n", sleep_called);
}

QS_TIMER_TIME_TYPE timer::get_curr_ntime(void)
{
	auto time_now = std::chrono::high_resolution_clock::now();
	auto curr_nsec_temp= std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	QS_TIMER_TIME_TYPE curr_nsec = (QS_TIMER_TIME_TYPE) curr_nsec_temp;
	
	return curr_nsec;
}