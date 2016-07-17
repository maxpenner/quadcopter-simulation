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

/****************************************
* Coordinate system in Irrlicht:
* 
* 						+Y (green)
*                      |
*                      |
*                      |
*                      |
*                      |
*                      |(0,0,0)
*                      /----------------- +Z (blue)
*                     /
*                    /
*                   /
*                  /
*                 /
*                +X (red)
* 
* Rotation in Irrlicht:
* 
* 			You observe the rotation around the axises from the origin (0,0,0). 
* 
* 			As an oberserver you either look at +X, +Y or +Z.
* 
* 			Then, rotation is always anticlockwise.
* 
* 			Values range is from from 0 to 360 (full rotation).
* 
* 			e.g.: any_node->setRotation(core::vector3df(  around_x_anticlockwise  , around_y_anticlockwise  ,   around_z_anticlockwise));
* 
* Coordinate System for simulation in local frame:
* 
* 			Both coordinate system share the x-axis per definition.
* 			So Y for simulation becomes Z in Irrlicht and vice versa.
* 
*                      +Z
*                      |
*                      |
*                      |
*                      |
*                      |
*                      |(0,0,0)
*                      /----------------- +Y
*                     /
*                    /
*                   /
*                  /
*                 /
*                +X
* 
* Rotation in simulation viewed from:
* 
* 			The attitude is given is a 3-element vector (roll, pitch, yaw).
* 
* 			Again, rotation is viewed from the origin (0,0,0). You look at +X, +Y or +Z.
* 
* 			roll: rotation around x axis clockwise
* 
* 			pitch: rotation around x axis clockwise
* 
* 			yaw: rotation around z axis clockwise
* 
* IMPORTANT:
* 	
* 			The scaling is done by the class "quadcopter_3D" when receiving parameters from simulation.
* 
******************************************/

#ifndef QS_RENDERER_RENDERER_H
#define QS_RENDERER_RENDERER_H

#include <irrlicht.h>

#ifdef _IRR_WINDOWS_
#pragma comment(lib, "Irrlicht.lib")
#pragma comment(linker, "/subsystem:windows /ENTRY:mainCRTStartup")
#endif

#include "irr_event_receiver.h"
#include "irr_coordinate_axes.h"
#include "irr_quadcopter_3D.h"

using namespace irr;

class irr_renderer
{
	public:
	
		irr_renderer(int frame_mode, int X_windows_size, int Y_windows_size, int fps);
		~irr_renderer();
		
		// enable direct access to quadcopter object in scene
		quadcopter_3D *Quadcopter_3D;
		
		// functions
		void render();
		bool get_rendererRunning();
		
	private:
	
		// variables for the irrlicht engine
		IrrlichtDevice *device;
		video::IVideoDriver *driver;
		scene::ISceneManager *smgr; 
		gui::IGUIFont* font;
		MyEventReceiver receiver;
		
		// control variables
		bool device_dropped;
		int lastFPS;
		
		// axes cross for orientation
		coordinate_axes *axes_node;
		
		// function 
		void display_information(void);
};

#endif