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


#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <simon/WorldObject.h>
#include <simon/RigidBody.h>
#include <simon/SmartPointer.h>

#include <assert.h>
#include <vector>
#include <stdio.h>

#include <simon/Interference.h>

       
//! forward declarations        
class Plane;
class Sphere;
class Capsule;
class Box;

class Geometry
{


public:
    
	//! Sinvolle initialisierung der Eigenschaften.
	Geometry(RigidBodyPtr& rigidBody);

	virtual ~Geometry(){};

	//! Set the owning rigidBody object
	void  setRigidBody(RigidBodyPtr& rigidBody) { mRigidBody=rigidBody; };        

	//! Get the owning rigidBody Object
	RigidBodyPtr& getRigidBody() { return mRigidBody; };
        
	virtual float getArea()=0;

	void  setBounciness(float bounciness);
	float getBounciness();
        
	void  setFriction(float friction);
	float getFriction() { return mFriction; }; 

	//! tells weather the geometry has changed or not
	bool hasChanged() const {return mHasChanged; };
	//! sets the geometry changed flag
	void hasChanged(bool flag) {mHasChanged = flag; };

protected:
	RigidBodyPtr mRigidBody;
	/*
	// vorrübergehend solange die rotierung von richtungs-vektoren nicht funktioniert
	Vec3 mNormal;        
	*/
	float mBounciness;
	float mFriction;

private: 

	//! indicates if the geometry has changed
	bool mHasChanged;
};



#endif // !GEOMETRY_H


