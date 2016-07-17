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

#ifndef QS_ESC_MOTOR_MOTOR_H
#define QS_ESC_MOTOR_MOTOR_H

class esc_motor
{
	public:
	
		esc_motor();
		~esc_motor();
		
		void esc_set_inputPWMDutyCycle(double pwm_dutyCycle);
		void solve_diff_equation_step(double *rpm_arg, double dt);
		
	private:
	
		double input_pwmDutyCycle;
};

#endif