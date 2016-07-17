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
#include "config.h"
#include "utils_diffequation.h"

// same matrix in + and x/h mode
void calc_inert_mat(Matrix3d *Inertia, double central_mass, double central_rad, double motor_mass, double L)
{
	double xx = 0.0;
	double yy = 0.0;
	double zz = 0.0;

	double fac_central_mass = 2.0/5.0*central_mass*central_rad*central_rad;

	xx += fac_central_mass;
	yy += fac_central_mass;
	zz += fac_central_mass;

	xx += 2*L*L*motor_mass;
	yy += 2*L*L*motor_mass;
	zz += 4*L*L*motor_mass;
	
	// quadcopter symmetrical, so values other than zero only on diagonal
	(*Inertia) <<	 xx, 0.0, 0.0,
					0.0,  yy, 0.0,
					0.0, 0.0,  zz;
}

void acceleration(Vector3d *a, const Vector4d& speeds, Vector3d angles, Vector3d vels, double m, double g, double k, Vector3d kd)
{
	Vector3d gravity(0,0,g);
	
	Vector3d T_body;
	thrust(&T_body, speeds, k);
	
	Matrix3d R;
	rotation(&R, angles);
	
	Vector3d T_local;
	T_local = R * T_body;
	
	Vector3d Fd(kd(0)*vels(0), kd(1)*vels(1), kd(2)*vels(2));
	
	*a = -gravity + 1/m * (T_local - Fd);
}

// from earth frame to body frame
void thetadot2omega(Vector3d *omega, Vector3d thetadot, Vector3d angles)
{
	double phi   = angles(0);
    double theta = angles(1);
    
    Matrix3d W;
    W << 	1, 0, -sin(theta),
			0, cos(phi), cos(theta)*sin(phi),
			0, -sin(phi), cos(theta)*cos(phi);
    
    (*omega) = W * thetadot;
}

void angular_acceleration(Vector3d *omegadot, const Vector4d& speeds, Vector3d omega, Matrix3d I, double L, double b, double k, int frame_mode)
{
	Vector3d tau;

	if(frame_mode == QS_FRAME_MODE_PL)
		torques_plus(&tau, speeds, L, b, k);
	else if(frame_mode == QS_FRAME_MODE_XH)
		torques_xh(&tau, speeds, L, b, k);

    (*omegadot) = I.inverse() * (tau - omega.cross(I*omega));
}

// from body frame to earth frame
void omega2thetadot(Vector3d *thetadot, Vector3d omega, Vector3d angles)
{
    double phi = angles(0);
    double theta = angles(1);
    
	// Option A
	/*
    Matrix3d W;
    W << 	1, 0, -sin(theta),
			0, cos(phi), cos(theta)*sin(phi),
			0, -sin(phi), cos(theta)*cos(phi);

	(*thetadot) = W.inverse() * omega;
	*/

	// Option B
	Matrix3d W_invers;
    W_invers << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
				0, cos(phi), -sin(phi),
				0, sin(phi)/cos(theta), cos(phi)/cos(theta);
    
    
	(*thetadot) = W_invers * omega;
}

// from bodyframe to earth frame f_e = R * f_b
void rotation(Matrix3d *R, Vector3d angles)
{
	double phi = angles(0);		// roll
	double theta = angles(1);	// pitch
	double psi = angles(2);		// yaw
	
	(*R) << cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi),
			sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi),
			-sin(theta), cos(theta)*sin(phi), cos(theta) * cos(phi);
}

void thrust(Vector3d *T_body, const Vector4d& speeds, double k)
{
	Vector4d speeds_sqr_thrust;

	for(int i=0; i<4; i++)
		speeds_sqr_thrust(i) = pow(speeds(i), MOTOR_EXPONENT_Q);
	
	(*T_body)(0) = 0;
	(*T_body)(1) = 0;
	(*T_body)(2) = k*speeds_sqr_thrust.sum();
}

void torques_plus(Vector3d *tau, const Vector4d& speeds, double L, double b, double k)
{
	Vector4d speeds_sqr_thrust, speeds_sqr_torque;

	for(int i=0; i<4; i++)
	{
		speeds_sqr_thrust(i) = pow(speeds(i), MOTOR_EXPONENT_Q);	
		speeds_sqr_torque(i) = pow(speeds(i), TORQUE_YAW_EXPONENT_QQ);
	}
	
	(*tau)(0) = L*k*(speeds_sqr_thrust(3) - speeds_sqr_thrust(1));
	(*tau)(1) = L*k*(speeds_sqr_thrust(2) - speeds_sqr_thrust(0));

	//(*tau)(2) = b*(speeds_sqr_torque(1) + speeds_sqr_torque(3) - speeds_sqr_torque(0) - speeds_sqr_torque(2));
	(*tau)(2) = b*(-speeds_sqr_torque(1) - speeds_sqr_torque(3) + speeds_sqr_torque(0) + speeds_sqr_torque(2));
}

void torques_xh(Vector3d *tau, const Vector4d& speeds, double L, double b, double k)
{
	Vector4d speeds_sqr_thrust, speeds_sqr_torque;

	for(int i=0; i<4; i++)
	{
		speeds_sqr_thrust(i) = pow(speeds(i), MOTOR_EXPONENT_Q);	
		speeds_sqr_torque(i) = pow(speeds(i), TORQUE_YAW_EXPONENT_QQ);
	}
	
	(*tau)(0) = L/sqrt(2)*k*(speeds_sqr_thrust(0) + speeds_sqr_thrust(3) - speeds_sqr_thrust(1) - speeds_sqr_thrust(2));
	(*tau)(1) = L/sqrt(2)*k*(speeds_sqr_thrust(3) + speeds_sqr_thrust(2) - speeds_sqr_thrust(0) - speeds_sqr_thrust(1));

	//(*tau)(2) = b*(speeds_sqr_torque(1) + speeds_sqr_torque(3) - speeds_sqr_torque(0) - speeds_sqr_torque(2));
	(*tau)(2) = b*(-speeds_sqr_torque(1) - speeds_sqr_torque(3) + speeds_sqr_torque(0) + speeds_sqr_torque(2));
}