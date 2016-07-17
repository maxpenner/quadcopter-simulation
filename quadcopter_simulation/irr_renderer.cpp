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

#include "irr_renderer.h"

#define FAR_SIGHT 100000.0f

irr_renderer::irr_renderer(int frame_mode, int X_windows_size, int Y_windows_size, int fps)
{
	// init irrlicht engine
	this->device = createDevice(video::EDT_OPENGL, core::dimension2d<u32>(X_windows_size, Y_windows_size), 16, false, false, false, &receiver);
	this->driver = device->getVideoDriver();
	this->smgr = device->getSceneManager();
	
	// init control variables
	this->device_dropped = false;
	this->lastFPS = -1;
	
	// add skybox to scene
	this->driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, false);
    this->smgr->addSkyBoxSceneNode(		driver->getTexture("media/renderer/skybox/irrlicht2_up.jpg"),
										driver->getTexture("media/renderer/skybox/irrlicht2_dn.jpg"),
										driver->getTexture("media/renderer/skybox/irrlicht2_lf.jpg"),
										driver->getTexture("media/renderer/skybox/irrlicht2_rt.jpg"),
										driver->getTexture("media/renderer/skybox/irrlicht2_ft.jpg"),
										driver->getTexture("media/renderer/skybox/irrlicht2_bk.jpg"));
    this->driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, true);
    
	// load font for displaying data
    this->font = this->device->getGUIEnvironment()->getFont("media/renderer/font/k.png");
    
    // add map to scene
    this->device->getFileSystem()->addFileArchive("media/renderer/quake/map-20kdm2.pk3");
	scene::IAnimatedMesh* mesh = this->smgr->getMesh("20kdm2.bsp");	
	scene::ISceneNode *node;
	if(mesh)
	{
		node = smgr->addOctreeSceneNode(mesh->getMesh(0), 0, -1, 1024);
		node->setPosition(core::vector3df(-1300,-700,-1300));
		node->setScale(core::vector3df(1.0f, 1.0f, 1.0f));
	}
	
	// add camera to scene
	scene::ICameraSceneNode *cam = this->smgr->addCameraSceneNode(0, core::vector3df(0,0,0), core::vector3df(0, 0, 0));	

	cam->setFarValue(FAR_SIGHT);

	// add stationary coordinate axes to scene
	this->axes_node = new coordinate_axes(this->smgr->getRootSceneNode(), this->smgr, -1);
	this->axes_node->setAxesScale(100, 2);
	
	// add quadcopter to scene (pass receiver to access key board)
	this->Quadcopter_3D = new quadcopter_3D(this->smgr, cam, frame_mode, &(this->receiver));
};

// =================================================================
irr_renderer::~irr_renderer(void)
{
	if(this->device_dropped == false)
		this->device->drop();

	delete this->axes_node;
	delete this->Quadcopter_3D;
	
	this->device_dropped = true;
	this->device = NULL;
	this->axes_node = NULL;
	this->Quadcopter_3D = NULL;
};

// =================================================================
void irr_renderer::render(void)
{
	if(this->device_dropped == true)
		return;
		
	// let the quadcopter set it's position in the world
	this->Quadcopter_3D->set_transformation();
	
	// draw
	if(this->device->run())
	{
		if (this->device->isWindowActive())
		{
			this->driver->beginScene(true, true, video::SColor(255,200,200,200));
			this->smgr->drawAll();
			
			// display quadcopter information
			display_information();
			
			this->driver->endScene();

			int fps = this->driver->getFPS();

			if (this->lastFPS != fps)
			{
				core::stringw str = L"Irrlicht Engine - Quake 3 Map example [";
				str += driver->getName();
				str += "] FPS:";
				str += fps;

				this->device->setWindowCaption(str.c_str());
				this->lastFPS = fps;
			}
		}
		else
			device->yield();
	}
	else
	{
		this->device->drop();
		this->device_dropped = true;
	}
};

bool irr_renderer::get_rendererRunning()
{
	return !(this->device_dropped);
}

// =================================================================
void irr_renderer::display_information(void)
{
	core::stringw roll = L"Roll: ";
	roll += this->Quadcopter_3D->get_attitude(0);
	roll += "deg";
	this->font->draw(roll.c_str(), core::rect<s32>(10, 12, 300, 60), video::SColor(255, 50 % 255, 50 % 255, 255));

	core::stringw pitch = L"Pitch: ";
	pitch += this->Quadcopter_3D->get_attitude(1);
	pitch += "deg";
	this->font->draw(pitch.c_str(), core::rect<s32>(10, 24, 300, 60), video::SColor(255, 50 % 255, 50 % 255, 255));

	core::stringw yaw = L"Yaw: ";
	yaw += this->Quadcopter_3D->get_attitude(2);
	yaw += "deg";
	this->font->draw(yaw.c_str(), core::rect<s32>(10, 36, 300, 60), video::SColor(255, 50 % 255, 50 % 255, 255));

	core::stringw climb = L"vert: ";
	climb += this->Quadcopter_3D->get_climbrate();
	climb += "m/s";
	this->font->draw(climb.c_str(), core::rect<s32>(10,48+10,300,60), video::SColor(255,50 % 255,50 % 255,255));
	
	core::stringw horispeed = L"hori: ";
	horispeed += this->Quadcopter_3D->get_horizonalspeed();
	horispeed += "km/h";
	this->font->draw(horispeed.c_str(), core::rect<s32>(10,60 + 10,300,60), video::SColor(255,50 % 255,50 % 255,255));
	
	core::stringw mot1 = L"motor 1: ";
	mot1 += this->Quadcopter_3D->get_motorspeed(0);
	mot1 += "rpm";
	this->font->draw(mot1.c_str(), core::rect<s32>(10,72+20,300,60), video::SColor(255,50 % 255,50 % 255,255));
	
	core::stringw mot2 = L"motor 2: ";
	mot2 += this->Quadcopter_3D->get_motorspeed(1);
	mot2 += "rpm";
	this->font->draw(mot2.c_str(), core::rect<s32>(10,84 + 20,300,60), video::SColor(255,50 % 255,50 % 255,255));
	
	core::stringw mot3 = L"motor 3: ";
	mot3 += this->Quadcopter_3D->get_motorspeed(2);
	mot3 += "rpm";
	this->font->draw(mot3.c_str(), core::rect<s32>(10,96 + 20,300,60), video::SColor(255,50 % 255,50 % 255,255));
	
	core::stringw mot4 = L"motor 4: ";
	mot4 += this->Quadcopter_3D->get_motorspeed(3);
	mot4 += "rpm";
	this->font->draw(mot4.c_str(), core::rect<s32>(10,108 + 20,300,60), video::SColor(255,50 % 255,50 % 255,255));
};