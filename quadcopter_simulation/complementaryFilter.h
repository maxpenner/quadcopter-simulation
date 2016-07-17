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

#ifndef QS_COMPLEMENTARYFILTER_H
#define QS_COMPLEMENTARYFILTER_H

#define COMPLEMENTARY_FILTER_TAU_UNDEFINED -1.0

class complementaryFilter
{
	public:

		complementaryFilter();
		~complementaryFilter();
		
		// set starting point
		void setCombinedEstimation(double combEstim);		
		
		// extract the most recent estimation
		double getCombinedEstimation(double estimation, double estimation_derivative);
		
		// Get and set parameter 'a' respectively 'tau'.
		// 0 <= tau < +infinity
		// 0 <= a <= 1
		double getTau();
		void setTau(double tau_arg);
		void setTauViaA(double a_arg);
		double getA();
		
	private:
	
		// combined estimatation of both inputs
		double combinedEstimation;
		
		// time between two samples
		double dT;
		
		// relationship: a = tau/(tau+dT) respectively tau = a*dT/(1-a)
		double tau;
		double a;
};

#endif