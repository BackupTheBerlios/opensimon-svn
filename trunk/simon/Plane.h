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
*  \file Plane.h
*  \class Plane
*/
//------------------------------------------------------------------------------

#ifndef PLANE_H
#define PLANE_H

#include <simon/Geometry.h>

class Sphere;

class Plane : public Geometry
{
public:
	Plane(RigidBodyPtr& rigidBody);
	
	//! Nothing to destruct.
	~Plane();
        
	//! Gibt die normale der ebene zurück
	Vec3 getNormal();
	
	virtual float getArea();
	
protected:
	//! Defines the standard Normal of a Plane for the Hesse Notation. 
	//! \see getNormal()
	static Vec3 mStandardNormal; 
};

#endif // !PLANE_H
