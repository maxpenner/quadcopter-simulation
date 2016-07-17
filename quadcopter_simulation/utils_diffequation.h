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

// source 0: http://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf
// source 1: http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf

#ifndef QS_UTILS_DIFFEQUATION_UTILS_DIFFEQUATION_H
#define QS_UTILS_DIFFEQUATION_UTILS_DIFFEQUATION_H

#include <Eigen/Dense>

using namespace Eigen;

void calc_inert_mat(Matrix3d *Inertia, double central_mass, double central_rad, double motor_mass, double L);

void acceleration(Vector3d *a, const Vector4d& speeds, Vector3d angles, Vector3d vels, double m, double g, double k, Vector3d kd);

void thetadot2omega(Vector3d *omega, Vector3d thetadot, Vector3d angles);

void angular_acceleration(Vector3d *omegadot, const Vector4d& speeds, Vector3d omega, Matrix3d I, double L, double b, double k, int frame_mode);

void omega2thetadot(Vector3d *thetadot, Vector3d omega, Vector3d angles);

void rotation(Matrix3d *R, Vector3d angles);

void thrust(Vector3d *T_body, const Vector4d& speeds, double k);

void torques_plus(Vector3d *tau, const Vector4d& speeds, double L, double b, double k);

void torques_xh(Vector3d *tau, const Vector4d& speeds, double L, double b, double k);

#endif