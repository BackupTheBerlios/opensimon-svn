/**
 * Simon is the legal property of its developers, whose names are too
 * numerous to list here.  Please refer to the COPYRIGHT file
 * distributed with this source distribution.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef GRAPHICS_RIB_H
#define GRAPHICS_RIB_H

#include <simon/Vector3.h>
#include <simon/Quaternion.h>
#include "GraphicsBase.h"
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <iomanip>

class GraphicsRIB : public GraphicsBase{

public:

	GraphicsRIB();
    //GraphicsRIB(std::string fileName = std::string ("visualSimon"));
	virtual ~GraphicsRIB(){};

	//! call this if you want a new frame
	virtual void beginFrame(float camX = 0, float camY = 0, float camZ = 0, 
							float coiX = 0, float coiY = 0, float coiZ = -1,
							float upX = 0, float upY = 1, float upZ = 0);

	//! call this if you want to finish a frame
	virtual void finishFrame();

	virtual void drawLine(
		const Vec3& from, 
		const Vec3& to, 
		const Material material) {};
		
	virtual void drawBox(
		const Vec3& position,
		const Vec3& scale,
		const Material material) {};

	virtual void drawBox(
		const Vec3& position,
		const Vec3& scale,
		const Quaternion& orientation,
		const Material material) {};

	virtual void drawPlane(
		const Vec3& position,
		const Quaternion& orientation, 
		const Material material,
		float size = 10000);  

	virtual void drawPlane(
		const Vec3& position,
		const Material material) {};

	virtual void drawSphere(
		const Vec3& position, 
		float radius, 
		const Quaternion& orientation,
		const Material material);

	virtual void drawSphere(
		const Vec3& position, 
	    float radius, 
		const Material material) {};

	virtual void drawCapsule(
		const Vec3& position, 
		float radius, 
		float height,
		const Quaternion& orientation,
		const Material material) {};

	virtual void drawCapsule(
		const Vec3& position, 
		float radius, 
		float height,
		const Material material) {};
							
	virtual void drawContactPoint(
		ContactPoint& contactPoint,
		const Material material) {};
  private:
    const std::string getColor   (Material material);
    const std::string getSurface (Material material);
    std::ostringstream mFile;
    std::ostringstream mPass1;
    std::ostringstream mPass2;
    std::ostringstream mPass3;
    std::string mFileName;
    std::string mFileNamePass1;
    std::string mFileNamePass2;
    std::string mFileNamePass3;
    std::string mPath;
    int mFrameNumber;
    int mSphereNumber;
    int mPlaneNumber;
};

#endif
