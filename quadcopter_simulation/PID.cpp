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

#include <math.h>

#include "mathHelp.h"
#include "PID.h"

PID::PID()
{
	// fixed during runtime
	dT = QS_SENSOR_INPUT_PERIOD/1000000000.0;
	
	setGains(0.0f, 0.0f, 0.0f);
	
	zeroErrorIntegral();
	setiTermLimit(0.0f);
	
	lastError = 0.0f;	
}

PID::~PID()
{
}

void PID::setGains(double kp_arg, double ki_arg, double kd_arg)
{
	kp = kp_arg;
	ki = ki_arg;
	kd = kd_arg;
}

void PID::setiTermLimit(double iTermLimit_arg)
{
	iTermLimit = fabs(iTermLimit_arg);
}

void PID::zeroErrorIntegral()
{
	errorIntegral = 0.0f;
}

double PID::compute(double target, double is)
{
	// error, integral of error and derivative of error
	double error = target - is;
	errorIntegral += error*dT;
	double errorDerivative = (error - lastError)/dT;
	
	// PID values
	double pTerm = kp*error;
	double iTerm = ki*errorIntegral;
	double dTerm = kd*errorDerivative;
	
	// limit iTerm
	iTerm = (iTerm > iTermLimit) ? iTermLimit : iTerm;
	iTerm = (iTerm <-iTermLimit) ?-iTermLimit : iTerm;
	
	// save error for next call
	lastError = error;
	
	return pTerm + iTerm + dTerm;
}

double PID::getErrorIntegral()
{
	return errorIntegral;
}