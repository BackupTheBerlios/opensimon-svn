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
*  \file Sphere.h
*  \class Sphere
*/
//------------------------------------------------------------------------------

#ifndef SPHERE_H
#define SPHERE_H

#include <simon/Geometry.h>
#include <simon/Vector3.h>

class Plane;

class Sphere : public Geometry
{
public:
	Sphere(SmartPointer<RigidBody>& rigidBody, float radius);
	~Sphere(){};
	    
	void  setRadius(float radius) { mRadius = radius; };
	float getRadius()             { return mRadius; };
        
	virtual float getArea();

/* Wer auch immer das war, das ist ABSOLUT verboten !!!
 * Der SmartPointer denkt dann, er wäre der einzige, der dieses Objekt kennt
 * und löscht es im Destruktor. NEVER EVER DO THIS !!
	SpherePtr getReference();*/
protected:
	float mRadius;
};

#endif // !SPHERE_H


