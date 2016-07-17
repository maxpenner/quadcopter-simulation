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

#ifndef QS_RENDERER_QUADCOPTER_3D_H
#define QS_RENDERER_QUADCOPTER_3D_H

#include <irrlicht.h>
#include "irr_event_receiver.h"

using namespace irr;

enum CAMERA_STATE
{
	CAM_FOLLOW	= 0,
	CAM_ORIGIN	= 1,
};

class quadcopter_3D
{

	public:
	
		quadcopter_3D(scene::ISceneManager* smgr, scene::ICameraSceneNode *cam, int frame_mode, MyEventReceiver *recv);
		~quadcopter_3D();
		
		void set_camera_state(enum CAMERA_STATE cam_state);

		// Simulation gives us the up_vector and the direction_vector.
		// These have to be tranformed to euler angles to set attitude in the world.
		void set_transformation();

		// interface from simulation to 3d object in scene
		void set_position(double x_sim, double y_sim, double z_sim);	
		void set_speed(double x_sim, double y_sim, double z_sim);
		void set_attitude(double r_sim, double p_sim, double y_sim);
		void set_motorspeed(double mot1, double mot2, double mot3, double mot4);
		void set_up_vector(double x_sim, double y_sim, double z_sim);			
		void set_direction_vector(double x_sim, double y_sim, double z_sim);	
		
		double get_attitude(int index);
		double get_climbrate();
		double get_horizonalspeed();
		double get_motorspeed(int index);
		
	private:
		
		// irrlicht variables
		scene::IAnimatedMesh *quadcopter_mesh;
		scene::ISceneNode *quadcopter_node;
		
		// camera passed at init
		scene::ICameraSceneNode *passed_cam;

		MyEventReceiver *receiver;
		
		// variables for the quadcopter
		core::vector3df position;			// absolute position in local frame
		core::vector3df attitude;
		core::vector3df direction_vector;	// direction of quadcopter z axis (0 0 1) in local frame
		core::vector3df up_vector;			// direction of quadcopter x axis (1 0 0) in local frame
		double motorspeed[4];
		double climbrate;					// m/s
		double horizontal_speed;			// km/h
		 
		// make camera follow
		enum CAMERA_STATE CAM_STATE;
		float cam_distance;
		float cam_height;

		void checkInput();
};

#endif