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
*  \file RigidBodySystem.h
*  \class RigidBodySystem
*/
//------------------------------------------------------------------------------
#ifndef RIGIDBODYSYSTEM_H
#define RIGIDBODYSYSTEM_H

#include <map>
#include <simon/RigidBody.h>
#include <simon/Id.h>
#include <simon/SimonState.h>
#include <simon/SmartPointer.h>


class RigidBodySystem {
public:
	RigidBodySystem();
	~RigidBodySystem();

	RigidBodyPtr create(Id id);
	RigidBodyPtr getRigidBody(Id id);
	int getNumberOfRigidBodies();

	//! call every RigidBody with the given function
	void forEveryRigidBodyCall(void (*givenFunction)(RigidBodyPtr body));

	const std::map<Id, RigidBodyPtr>* getMap();
	
	void deleteRigidBody(Id id);

	void addGravity();

	void integrate(float interval);
	void integrateVelocities(float interval);
	void integratePositions(float interval);

	void enableViscosity();
	void disableViscosity();

	typedef std::map<Id, RigidBodyPtr> RigidBodyList;
	RigidBodyList mBodyList;

	void integrateEuler(float interval);

	void integrateEulerVelocities(float interval);
	void integrateEulerPositions(float interval);

private:


};

#endif
