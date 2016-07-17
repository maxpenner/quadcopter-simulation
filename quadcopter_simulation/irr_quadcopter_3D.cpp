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
#include "irr_quadcopter_3D.h"

// one meter in simulation equals 100 units in Irrlicht
#define SCALE_FAC 100.0	
#define FOLLOW_CAM_DISTANCE 50.0f
#define FOLLOW_CAM_HEIGHT 20.0f
#define FOLLOW_CAM_STEP 1.5f

#define RAD2DEG_ 57.2957951f

quadcopter_3D::quadcopter_3D(scene::ISceneManager* smgr, scene::ICameraSceneNode *cam, int frame_mode, MyEventReceiver *recv)
{
	// set initial state of camera
	this->CAM_STATE = CAM_FOLLOW;
	
	// copy camera pointer
	this->passed_cam = cam;

	this->receiver = recv;

	this->cam_distance = FOLLOW_CAM_DISTANCE;
	this->cam_height = FOLLOW_CAM_HEIGHT;
	
	// let SceneManager load mesh (0 is + and 1 is x/h)
	if(frame_mode == 0)
		this->quadcopter_mesh = smgr->getMesh("media/renderer/quadcopter0/MQ-27.obj");
	else if(frame_mode == 1)
		this->quadcopter_mesh = smgr->getMesh("media/renderer/quadcopter1/MQ-27.obj");
	
	// create node from mesh
	this->quadcopter_node = 0;
	if(this->quadcopter_mesh)
	{
		this->quadcopter_node = smgr->addOctreeSceneNode(this->quadcopter_mesh->getMesh(0), 0, -1, 1024);
		this->quadcopter_node->setScale(core::vector3df(1/9.5f,1/9.5f,1/9.5f));
	}
		
	// turn of dynamic lights
	if(this->quadcopter_node)
		this->quadcopter_node->setMaterialFlag(video::EMF_LIGHTING, false);

	// set speficic time timings
	this->receiver->setKeyBlockTime(KEY_KEY_C, 1000);
	this->receiver->setKeyBlockTime(KEY_LEFT, 20);
	this->receiver->setKeyBlockTime(KEY_RIGHT, 20);
	this->receiver->setKeyBlockTime(KEY_UP, 20);
	this->receiver->setKeyBlockTime(KEY_DOWN, 20);
	this->receiver->setKeyBlockTime(KEY_KEY_V, 500);
};

// =================================================================
quadcopter_3D::~quadcopter_3D()
{
	// nothing to do here
};
 
// =================================================================
void quadcopter_3D::set_camera_state(enum CAMERA_STATE cam_state)
{
	this->CAM_STATE = cam_state;
};

// =================================================================
void quadcopter_3D::set_transformation()
{	
	// SET ATTITUDE: code from "http://irrlicht.sourceforge.net/forum/viewtopic.php?f=9&t=49676
	
	// note: this->direction_vector and this->up_vector are normalized when called here
	
	core::vector3df fromDirection = this->direction_vector;
	core::vector3df fromUp = this->up_vector;

	core::vector3df masterDirection(fromDirection);
	masterDirection.normalize();

	core::vector3df masterUp(fromUp);
	masterUp.normalize();

	// project the up vector onto a vector that's orthogonal to the direction
	core::vector3df realUp = masterDirection.crossProduct(masterUp).normalize();
	realUp = realUp.crossProduct(masterDirection);

	// Get the quaternion to rotate to the required direction
	core::quaternion quatDirection;
	quatDirection.rotationFromTo(core::vector3df(1, 0, 0), masterDirection);

	// Apply that rotation to the world up vector
	core::vector3df worldUp(0, 1, 0);
	core::matrix4 mat;
	quatDirection.getMatrix(mat);
	mat.rotateVect(worldUp);

	// Get the quaternion to rotate to the required up
	core::quaternion quatUp;
	quatUp.rotationFromTo(worldUp, realUp);

	// Concatenate them to get a total rotation
	core::quaternion quat = quatDirection * quatUp;

	// Convert to euler rotations
	core::vector3df eulers;
	quat.toEuler(eulers); //... in radians
	eulers *= core::RADTODEG;   //... and now in degrees
	// This is the euler rotations required. 

	// SET ROTATION
	this->quadcopter_node->setRotation(eulers);
	
	// SET POSITION
	this->quadcopter_node->setPosition(this->position);

	this->checkInput();

	// SET CAMERA
	switch(this->CAM_STATE)
	{
		case CAM_FOLLOW:
			{
				// calculated normalized direction vector in XZ-plane
				double x_plane = this->direction_vector.X;
				double z_plane = this->direction_vector.Z;
				double normalizer = sqrt(pow(x_plane, 2.0) + pow(z_plane, 2.0));
				x_plane = x_plane / normalizer;
				z_plane = z_plane / normalizer;
				
				// calculate camera position
				core::vector3df cam_position(	this->position.X - x_plane*this->cam_distance, 
												this->position.Y + this->cam_height, 
												this->position.Z - z_plane*this->cam_distance);
				
				this->passed_cam->setPosition(cam_position);
				this->passed_cam->setTarget(this->position);
			}
			break;
			
		case CAM_ORIGIN:
			this->passed_cam->setPosition(core::vector3df(0,0,0));
			this->passed_cam->setTarget(this->position);
			break;
	}
};

// =================================================================
void quadcopter_3D::set_position(double x_sim, double y_sim, double z_sim)
{
	this->position.X = x_sim*SCALE_FAC;
	this->position.Y = z_sim*SCALE_FAC;	// Z in simulation becomes Y in irrlicht
	this->position.Z = y_sim*SCALE_FAC;	// Y in simulation becomes Z in irrlicht
};

void quadcopter_3D::set_attitude(double r_sim, double p_sim, double y_sim)
{
	this->attitude.X = r_sim*RAD2DEG_;
	this->attitude.Y = p_sim*RAD2DEG_;
	this->attitude.Z = y_sim*RAD2DEG_;
}

// =================================================================
void quadcopter_3D::set_speed(double x_sim, double y_sim, double z_sim)
{
	double x = x_sim;
	double y = z_sim; // Z in simulation becomes Y in irrlicht
	double z = y_sim; // Y in simulation becomes Z in irrlicht
	
	this->climbrate = y;
	this->horizontal_speed = sqrt(pow(x, 2.0) + pow(z, 2.0));
	this->horizontal_speed *=3.6;		// m/s to km/h
};

// =================================================================
void quadcopter_3D::set_motorspeed(double mot1, double mot2, double mot3, double mot4)
{
	this->motorspeed[0] = mot1;
	this->motorspeed[1] = mot2;
	this->motorspeed[2] = mot3;
	this->motorspeed[3] = mot4;
};

// =================================================================
void quadcopter_3D::set_direction_vector(double x_sim, double y_sim, double z_sim)
{
	this->direction_vector.X = x_sim;
	this->direction_vector.Y = z_sim; 	// Z in simulation becomes Y in irrlicht
	this->direction_vector.Z = y_sim; 	// Y in simulation becomes Z in irrlicht
};

// =================================================================
void quadcopter_3D::set_up_vector(double x_sim, double y_sim, double z_sim)
{
	this->up_vector.X = x_sim;
	this->up_vector.Y = z_sim; 	// Z in simulation becomes Y in irrlicht
	this->up_vector.Z = y_sim; 	// Y in simulation becomes Z in irrlicht	
};

// =================================================================
double quadcopter_3D::get_attitude(int index)
{
	float retval = 0.0;
	switch (index)
	{
		case 0:
			retval = this->attitude.X;
			break;
		case 1:
			retval = this->attitude.Y;
			break;
		case 2:
			retval = this->attitude.Z;
			break;
	}

	return retval;
};
 
// =================================================================
double quadcopter_3D::get_climbrate()
{
	return this->climbrate;
};

// =================================================================
double quadcopter_3D::get_horizonalspeed()
{
	return this->horizontal_speed;
};

// =================================================================
double quadcopter_3D::get_motorspeed(int index)
{
	return this->motorspeed[index];
};

// =================================================================
void quadcopter_3D::checkInput()
{
	// camera type
	if (this->receiver->IsKeyDown(KEY_KEY_C))
	{
		if (this->CAM_STATE == CAM_FOLLOW)
			this->CAM_STATE = CAM_ORIGIN;
		else
			this->CAM_STATE = CAM_FOLLOW;
	}

	// camera distance
	if (this->receiver->IsKeyDown(KEY_LEFT))
		this->cam_distance += FOLLOW_CAM_STEP;
	else if (this->receiver->IsKeyDown(KEY_RIGHT))
		this->cam_distance -= FOLLOW_CAM_STEP;

	// camera heigth
	if (this->receiver->IsKeyDown(KEY_UP))
		this->cam_height += FOLLOW_CAM_STEP;
	else if (this->receiver->IsKeyDown(KEY_DOWN))
		this->cam_height -= FOLLOW_CAM_STEP;

	if (this->receiver->IsKeyDown(KEY_KEY_V))
	{
		this->cam_height = FOLLOW_CAM_HEIGHT;
		this->cam_distance = FOLLOW_CAM_DISTANCE;
	}
};