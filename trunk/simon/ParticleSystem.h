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


// $Id: ParticleSystem.h,v 1.28 2004/12/14 18:29:47 alangs Exp $
#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include <map>
#include <vector>


#include <simon/SimonState.h>
#include <simon/Particle.h>
#include <simon/Id.h>
#include <simon/SmartPointer.h>

#include <simon/Capsule.h>
#include <simon/Sphere.h>
#include <simon/Plane.h>
#include <simon/Box.h>


class ParticleSystem {
public:
	ParticleSystem();
	void drawGeometries();
	
	ParticlePtr create(Id id);
	ParticlePtr getParticle(int index);
	ParticlePtr getParticle(Id id);
	int getNumberOfParticles();

	void deleteParticle(Id id);


	RigidBodyPtr getCollidingBody();
	void setCollidingBody(RigidBodyPtr);
	bool isColliding(ParticlePtr);
	int isCollidingWhere(ParticlePtr);

	void addGravity();

	void integrateEuler(float interval);
	void integrateRungeKutta(float interval);
	void integrateVerletBaltman(double);
	void forEveryParticleCall(void (*givenFunction)(ParticlePtr));
	std::vector<Vec3> getPositionsOfParticles();
	std::vector<Id> getIdsOfParticles();
	std::vector<std::pair<Id, Vec3> > getIdsAndPosOfParticles();
	void checkCollisions();
    
	void lookForCollisionsWith(BoxPtr);

	void lookForCollisionsWith(CapsulePtr);

	void lookForCollisionsWith(SpherePtr);

	void lookForCollisionsWith(PlanePtr);

	void collide(ParticlePtr, CapsulePtr);

	void collide(ParticlePtr, SpherePtr);

	void collide(ParticlePtr, PlanePtr);

	void collide(ParticlePtr, BoxPtr);

private:
	typedef std::map<Id, ParticlePtr> ParticleList;
	ParticleList mParticleList;

	typedef std::vector<CapsulePtr> capsuleVector;
	capsuleVector mCaps;
	typedef std::vector<SpherePtr> sphereVector;
	sphereVector mSphere;
	typedef std::vector<PlanePtr> planeVector;
	planeVector mPlane;
	typedef std::vector<BoxPtr> boxVector;
	boxVector mBox;


	//values for viscosit
	RigidBodyPtr mCollidingBody;
};

#endif
