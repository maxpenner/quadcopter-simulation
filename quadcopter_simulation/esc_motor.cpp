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
#include <math.h>
#include "config.h"
#include "mathHelp.h"
#include "esc_motor.h"

esc_motor::esc_motor()
{
}

esc_motor::~esc_motor()
{
}

void esc_motor::esc_set_inputPWMDutyCycle(double pwm_dutyCycle)
{	 
	this->input_pwmDutyCycle = pwm_dutyCycle;
}

void esc_motor::solve_diff_equation_step(double *rpm_arg, double dt)
{
	// At this point the relation between pwm and rpm comes into play.
	if(QS_ESC_DUTY_CYCLE_THRUST_RELATION_DEFAULT == QS_ESC_DUTY_CYCLE_2_RPM_LINEAR)
	{
		*rpm_arg = mapp<double>(this->input_pwmDutyCycle, MOTOR_ESC_PWM_MIN, MOTOR_ESC_PWM_MAX, MOTOR_RPM_MIN, MOTOR_RPM_MAX);
	}
	
	if(QS_ESC_DUTY_CYCLE_THRUST_RELATION_DEFAULT == QS_ESC_DUTY_CYCLE_2_THRUST_LINEAR)
	{
		double thrust_wanted = mapp<double>(this->input_pwmDutyCycle, MOTOR_ESC_PWM_MIN, MOTOR_ESC_PWM_MAX, MOTOR_THRUST_MIN, MOTOR_THRUST_MAX);
		*rpm_arg = pow(thrust_wanted/MOTOR_CONSTANT, 1.0/MOTOR_EXPONENT_Q);
	}
}