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

#ifndef QC_PID_H
#define QC_PID_H

#include "config.h"

class PID
{
	public:

		PID();
		~PID();
		
		void setGains(double kp_arg, double ki_arg, double kd_arg);
		void setiTermLimit(double iTermLimit_arg);
		void zeroErrorIntegral();
		
		double compute(double target, double is);

		double getErrorIntegral();
		
	private:
	
		// time between two calls of the filter
		double dT;
	
		// gains for P, I and D
		double kp, ki, kd;
		
		// needed for I
		double errorIntegral;
		double iTermLimit;			// integral wind-up security
		
		// needed for D
		double lastError;
};

#endif