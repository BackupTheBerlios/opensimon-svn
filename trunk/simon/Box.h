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


//------------------------------------------------------------------------------
/**
*  \file Box.h
*  \class Box
*/
//------------------------------------------------------------------------------
#ifndef BOX_H
#define BOX_H

#include <simon/RigidBody.h>
#include <simon/Geometry.h>

class Plane;
class Sphere;

class Box : public Geometry {
public:

	Box(SmartPointer<RigidBody>& rigidBody, Vec3 scale);
	~Box();
	void  setScale (Vec3 scale);    
	void  setHeight(float height);
	void  setWidth (float width);
	void  setDepth (float depth);

	Vec3 getScale(){ return mScale; };  
	float getHeight(){ return mScale[Y]*2; };
	float getWidth(){ return mScale[X]*2; };
	float getDepth(){ return mScale[Z]*2; };

	virtual float getArea();

	// kontakt-punkte und normale fuer box-box berechnen
	void getIntersectionWithBox(Interference*& interference, Box* box, 
								Vec3* normalsA, Vec3* normalsB, bool parallel);

	// abgespeckte sphere-box kollision nur fuer den caps-box fall
	bool collideSphereBox (Vec3 shperePosition, float radius, Vec3& contactPoint, Vec3& normalB, float& distanceFromBox);

	// testet kollision mit OBB
	bool intersectionWithOBB (Vec3 positionOBB, Quaternion oriOBB, Vec3 scaleOBB);
public:

	Vec3 mScale;
};

#endif // !BOX_H
