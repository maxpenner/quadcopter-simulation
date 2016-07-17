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

// copy-n-paste from:
// http ://irrlicht.sourceforge.net/forum/viewtopic.php?t=21993

#ifndef QS_RENDERER_COORDINATE_AXES_H
#define QS_RENDERER_COORDINATE_AXES_H

#include <irrlicht.h>

using namespace irr;

class coordinate_axes : public scene::ISceneNode
{
	core::aabbox3d<f32> Box;
	video::SMaterial Material;
	
	video::SColor ZColor;
	video::SColor YColor;
	video::SColor XColor;
	
	scene::SMeshBuffer ZMeshBuffer;
	scene::SMeshBuffer YMeshBuffer;
	scene::SMeshBuffer XMeshBuffer;

	public:

		coordinate_axes(scene::ISceneNode* parent, scene::ISceneManager* mgr, s32 id) : scene::ISceneNode(parent, mgr, id)
		{
			{
				// Color Settings
				ZColor = video::SColor(255,0,0,255);
				YColor = video::SColor(255,0,255,0);
				XColor = video::SColor(255,255,0,0);
				
				// index settings
				u16 u[36] = {0,2,1,  0,3,2,  1,5,4,  1,2,5,  4,6,7,  4,5,6,  7,3,0,  7,6,3,  3,5,2,  3,6,5,  0,1,4,  0,4,7,};
				ZMeshBuffer.Indices.set_used(36);
				YMeshBuffer.Indices.set_used(36);
				XMeshBuffer.Indices.set_used(36);
				for (s32 i=0; i<36; ++i)
				{
					ZMeshBuffer.Indices[i] = u[i];
					YMeshBuffer.Indices[i] = u[i];
					XMeshBuffer.Indices[i] = u[i];
				}

				// default position, rotation and scale
				this->setPosition(core::vector3df(0,0,0));
				this->setRotation(core::vector3df(0,0,0));
				this->setScale(core::vector3df(1,1,1));

				// axes box coordinates settings
				setAxesCoordinates();
			}
		}

		virtual void OnRegisterSceneNode()
		{
			if (IsVisible)
				SceneManager->registerNodeForRendering(this);

			ISceneNode::OnRegisterSceneNode();
		}

		virtual void render()
		{
			video::IVideoDriver* driver = SceneManager->getVideoDriver();
			
			driver->setMaterial(ZMeshBuffer.Material);
			driver->setTransform(video::ETS_WORLD, AbsoluteTransformation);
			driver->drawMeshBuffer(&ZMeshBuffer);

			driver->setMaterial(YMeshBuffer.Material);
			driver->setTransform(video::ETS_WORLD, AbsoluteTransformation);
			driver->drawMeshBuffer(&YMeshBuffer);

			driver->setMaterial(XMeshBuffer.Material);
			driver->setTransform(video::ETS_WORLD, AbsoluteTransformation);
			driver->drawMeshBuffer(&XMeshBuffer);
		}

		virtual const core::aabbox3d<f32>& getBoundingBox() const
		{
			return Box;
		}

		virtual u32 getMaterialCount() const
		{
			return 1;
		}

		virtual video::SMaterial& getMaterial(u32 i)
		{
			return Material;
		}
		
		void setAxesCoordinates()
		{
			ZMeshBuffer.Vertices.set_used(8);
			ZMeshBuffer.Material.Wireframe = false;
			ZMeshBuffer.Material.Lighting = false;
			ZMeshBuffer.Vertices[0] = video::S3DVertex(-0.25,-0.25,0, -1,-1,-1, ZColor, 0, 1); 
			ZMeshBuffer.Vertices[1] = video::S3DVertex(0.25,-0.25,0,  1,-1,-1, ZColor, 1, 1); 
			ZMeshBuffer.Vertices[2] = video::S3DVertex(0.25,0.25,0,  1, 1,-1, ZColor, 1, 0); 
			ZMeshBuffer.Vertices[3] = video::S3DVertex(-0.25,0.25,0, -1, 1,-1, ZColor, 0, 0); 
			ZMeshBuffer.Vertices[4] = video::S3DVertex(0.25,-0.25,25,  1,-1, 1, ZColor, 0, 1); 
			ZMeshBuffer.Vertices[5] = video::S3DVertex(0.25,0.25,25,  1, 1, 1, ZColor, 0, 0); 
			ZMeshBuffer.Vertices[6] = video::S3DVertex(-0.25,0.25,25, -1, 1, 1, ZColor, 1, 0); 
			ZMeshBuffer.Vertices[7] = video::S3DVertex(-0.25,-0.25,25, -1,-1, 1, ZColor, 1, 1); 
			ZMeshBuffer.BoundingBox.reset(0,0,0); 

			YMeshBuffer.Vertices.set_used(8);
			YMeshBuffer.Material.Wireframe = false;
			YMeshBuffer.Material.Lighting = false;
			YMeshBuffer.Vertices[0] = video::S3DVertex(-0.25,0,0.25, -1,-1,-1, YColor, 0, 1); 
			YMeshBuffer.Vertices[1] = video::S3DVertex(0.25,0,0.25,  1,-1,-1, YColor, 1, 1); 
			YMeshBuffer.Vertices[2] = video::S3DVertex(0.25,0,-0.25,  1, 1,-1, YColor, 1, 0); 
			YMeshBuffer.Vertices[3] = video::S3DVertex(-0.25,0,-0.25, -1, 1,-1, YColor, 0, 0); 
			YMeshBuffer.Vertices[4] = video::S3DVertex(0.25,25,0.25,  1,-1, 1, YColor, 0, 1); 
			YMeshBuffer.Vertices[5] = video::S3DVertex(0.25,25,-0.25,  1, 1, 1, YColor, 0, 0); 
			YMeshBuffer.Vertices[6] = video::S3DVertex(-0.25,25,-0.25, -1, 1, 1, YColor, 1, 0); 
			YMeshBuffer.Vertices[7] = video::S3DVertex(-0.25,25,0.25, -1,-1, 1, YColor, 1, 1); 
			YMeshBuffer.BoundingBox.reset(0,0,0); 

			XMeshBuffer.Vertices.set_used(8);
			XMeshBuffer.Material.Wireframe = false;
			XMeshBuffer.Material.Lighting = false;
			XMeshBuffer.Vertices[0] = video::S3DVertex(0,-0.25,0.25, -1,-1,-1, XColor, 0, 1); 
			XMeshBuffer.Vertices[1] = video::S3DVertex(0,-0.25,-0.25,  1,-1,-1, XColor, 1, 1); 
			XMeshBuffer.Vertices[2] = video::S3DVertex(0,0.25,-0.25,  1, 1,-1, XColor, 1, 0); 
			XMeshBuffer.Vertices[3] = video::S3DVertex(0,0.25,0.25, -1, 1,-1, XColor, 0, 0); 
			XMeshBuffer.Vertices[4] = video::S3DVertex(25,-0.25,-0.25,  1,-1, 1, XColor, 0, 1); 
			XMeshBuffer.Vertices[5] = video::S3DVertex(25,0.25,-0.25,  1, 1, 1, XColor, 0, 0); 
			XMeshBuffer.Vertices[6] = video::S3DVertex(25,0.25,0.25, -1, 1, 1, XColor, 1, 0); 
			XMeshBuffer.Vertices[7] = video::S3DVertex(25,-0.25,0.25, -1,-1, 1, XColor, 1, 1); 
			XMeshBuffer.BoundingBox.reset(0,0,0);
		}
		
		// set the sacele of the axis cross
		void setAxesScale(f32 scale, f32 width)
		{
			ZMeshBuffer.Vertices.set_used(8);
			ZMeshBuffer.Material.Wireframe = false;
			ZMeshBuffer.Material.Lighting = false;
			ZMeshBuffer.Vertices[0] = video::S3DVertex(-width,-width,0, -1,-1,-1, ZColor, 0, 1); 
			ZMeshBuffer.Vertices[1] = video::S3DVertex(width,-width,0,  1,-1,-1, ZColor, 1, 1); 
			ZMeshBuffer.Vertices[2] = video::S3DVertex(width,width,0,  1, 1,-1, ZColor, 1, 0); 
			ZMeshBuffer.Vertices[3] = video::S3DVertex(-width,width,0, -1, 1,-1, ZColor, 0, 0); 
			ZMeshBuffer.Vertices[4] = video::S3DVertex(width,-width,scale,  1,-1, 1, ZColor, 0, 1); 
			ZMeshBuffer.Vertices[5] = video::S3DVertex(width,width,scale,  1, 1, 1, ZColor, 0, 0); 
			ZMeshBuffer.Vertices[6] = video::S3DVertex(-width,width,scale, -1, 1, 1, ZColor, 1, 0); 
			ZMeshBuffer.Vertices[7] = video::S3DVertex(-width,-width,scale, -1,-1, 1, ZColor, 1, 1); 
			ZMeshBuffer.BoundingBox.reset(0,0,0); 

			YMeshBuffer.Vertices.set_used(8);
			YMeshBuffer.Material.Wireframe = false;
			YMeshBuffer.Material.Lighting = false;
			YMeshBuffer.Vertices[0] = video::S3DVertex(-width,0,width, -1,-1,-1, YColor, 0, 1); 
			YMeshBuffer.Vertices[1] = video::S3DVertex(width,0,width,  1,-1,-1, YColor, 1, 1); 
			YMeshBuffer.Vertices[2] = video::S3DVertex(width,0,-width,  1, 1,-1, YColor, 1, 0); 
			YMeshBuffer.Vertices[3] = video::S3DVertex(-width,0,-width, -1, 1,-1, YColor, 0, 0); 
			YMeshBuffer.Vertices[4] = video::S3DVertex(width,scale,width,  1,-1, 1, YColor, 0, 1); 
			YMeshBuffer.Vertices[5] = video::S3DVertex(width,scale,-width,  1, 1, 1, YColor, 0, 0); 
			YMeshBuffer.Vertices[6] = video::S3DVertex(-width,scale,-width, -1, 1, 1, YColor, 1, 0); 
			YMeshBuffer.Vertices[7] = video::S3DVertex(-width,scale,width, -1,-1, 1, YColor, 1, 1); 
			YMeshBuffer.BoundingBox.reset(0,0,0); 

			XMeshBuffer.Vertices.set_used(8);
			XMeshBuffer.Material.Wireframe = false;
			XMeshBuffer.Material.Lighting = false;
			XMeshBuffer.Vertices[0] = video::S3DVertex(0,-width,width, -1,-1,-1, XColor, 0, 1); 
			XMeshBuffer.Vertices[1] = video::S3DVertex(0,-width,-width,  1,-1,-1, XColor, 1, 1); 
			XMeshBuffer.Vertices[2] = video::S3DVertex(0,width,-width,  1, 1,-1, XColor, 1, 0); 
			XMeshBuffer.Vertices[3] = video::S3DVertex(0,width,width, -1, 1,-1, XColor, 0, 0); 
			XMeshBuffer.Vertices[4] = video::S3DVertex(scale,-width,-width,  1,-1, 1, XColor, 0, 1); 
			XMeshBuffer.Vertices[5] = video::S3DVertex(scale,width,-width,  1, 1, 1, XColor, 0, 0); 
			XMeshBuffer.Vertices[6] = video::S3DVertex(scale,width,width, -1, 1, 1, XColor, 1, 0); 
			XMeshBuffer.Vertices[7] = video::S3DVertex(scale,-width,width, -1,-1, 1, XColor, 1, 1); 
			XMeshBuffer.BoundingBox.reset(0,0,0);
		}	
};

#endif