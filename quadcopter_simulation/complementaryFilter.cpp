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

#include "config.h"
#include "complementaryFilter.h"

complementaryFilter::complementaryFilter()
{	
	// starting point has to be set via 'setCombinedEstimation()'
	combinedEstimation = 0.0f;
	
	// the loop time is fixed during runtime
	dT = QS_SENSOR_INPUT_PERIOD/1000000000.0;

	// this setting means: trust the linear input, mistrust the derivative
	setTauViaA(1.0f);
}

complementaryFilter::~complementaryFilter()
{
}

void complementaryFilter::setCombinedEstimation(double combEstim)
{
	combinedEstimation = combEstim;
}

double complementaryFilter::getCombinedEstimation(double estimation, double estimation_derivative)
{
	combinedEstimation = a*(combinedEstimation + estimation_derivative*dT) + (1.0f-a)*estimation;
	
	return combinedEstimation;
}

double complementaryFilter::getTau()
{
	return tau;
}

void complementaryFilter::setTau(double tau_arg)
{
	tau = tau_arg;
	a = tau/(tau+dT);
}

void complementaryFilter::setTauViaA(double a_arg)
{
	a = a_arg;
	tau = (a != 1.0f) ? a*dT/(1.0f-a) : COMPLEMENTARY_FILTER_TAU_UNDEFINED;
}

double complementaryFilter::getA()
{
	return a;
}
