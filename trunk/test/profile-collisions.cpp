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



#include "test-environment.h"
#include "TestEnvironment.h"

#include <simon/config.h>

#include <simon/GeometrySystem.h>
#include <simon/Sphere.h>
#include <simon/Plane.h>
#include <simon/SmartPointer.h>

#include <iostream>
#include <fstream>

#include <simon/Quaternion.h>
#include <simon/SimonState.h>


#define NUM_COLLISION_CHECKS 5.0
#define SPEED_FACTOR 1
#define TIME_STEP 10


static SmartPointer<RigidBody> movingObject;

static RigidBodySystem rigidBodySystem;
static GeometrySystem geometrySystem;

                                                  
SmartPointer<RigidBody> rigidBodyPlane1(rigidBodySystem.create(Id(Id::typePlane,0)));
SmartPointer<RigidBody> rigidBodyPlane2(rigidBodySystem.create(Id(Id::typePlane,1)));
SmartPointer<RigidBody> rigidBodyPlane3(rigidBodySystem.create(Id(Id::typePlane,2)));
SmartPointer<RigidBody> rigidBodyPlane4(rigidBodySystem.create(Id(Id::typePlane,3)));
SmartPointer<RigidBody> rigidBodyPlane5(rigidBodySystem.create(Id(Id::typePlane,4)));

SmartPointer<RigidBody> rigidBodySphereA(rigidBodySystem.create(Id(Id::typeSphere,5)));
SmartPointer<RigidBody> rigidBodySphereB(rigidBodySystem.create(Id(Id::typeSphere,6)));
SmartPointer<RigidBody> rigidBodySphereC(rigidBodySystem.create(Id(Id::typeSphere,7)));
SmartPointer<RigidBody> rigidBodySphereD(rigidBodySystem.create(Id(Id::typeSphere,8)));
SmartPointer<RigidBody> rigidBodySphereE(rigidBodySystem.create(Id(Id::typeSphere,9)));
SmartPointer<RigidBody> rigidBodySphereF(rigidBodySystem.create(Id(Id::typeSphere,10)));
SmartPointer<RigidBody> rigidBodySphereG(rigidBodySystem.create(Id(Id::typeSphere,11)));
SmartPointer<RigidBody> rigidBodySphereH(rigidBodySystem.create(Id(Id::typeSphere,12)));

SmartPointer<RigidBody> rigidBodyBox1(rigidBodySystem.create(Id(Id::typeBox,13)));
SmartPointer<RigidBody> rigidBodyBox2(rigidBodySystem.create(Id(Id::typeBox,14)));
SmartPointer<RigidBody> rigidBodyCaps1(rigidBodySystem.create(Id(Id::typeSphere,15)));
SmartPointer<RigidBody> rigidBodyCaps2(rigidBodySystem.create(Id(Id::typeSphere,16)));
SmartPointer<RigidBody> rigidBodyCaps3(rigidBodySystem.create(Id(Id::typeSphere,17)));
SmartPointer<RigidBody> rigidBodyCaps4(rigidBodySystem.create(Id(Id::typeSphere,18)));

using namespace std;

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung
 * aufgerufen. Hier sollte alles reingeschrieben werden,
 * was für die initialisierung der einzelnen Tests nötig ist.
 */
int main(int /*num*/, char** /*args*/)
{
	rigidBodySystem.disableViscosity();
//SimonState::exemplar()->setGravityVector(Vector3<float>(0,-0.004,0));
	ofstream out("log.txt");
	clog.rdbuf(out.rdbuf());

//planes für testbox

	GeometryPtr leftPlane = geometrySystem.createPlane(
		rigidBodyPlane1);
	leftPlane->setVisibilityState(true);
	leftPlane->setPosition( Vector3<float>(-300,0,0));
	leftPlane->setOrientation(Quaternion (M_PI/2,Vector3<float>(0,1,0)));
	leftPlane->getRigidBody()->setIsDynamicFlag(false);
	leftPlane->getRigidBody()->setTorque(Vector3<float>(0,0,0));
	leftPlane->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
	leftPlane->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
	leftPlane->setInvMass(0);
	leftPlane->setBounciness(0.9);


	SmartPointer<Geometry> rightPlane = geometrySystem.createPlane(
		rigidBodyPlane2);
	rightPlane->setVisibilityState(false);
	rightPlane->setPosition( Vector3<float>(300,0,0));
	rightPlane->setOrientation(Quaternion (-M_PI/2,Vector3<float>(0,1,0)));
	rightPlane->getRigidBody()->setIsDynamicFlag(false);
	rightPlane->getRigidBody()->setTorque(Vector3<float>(0,0,0));
	rightPlane->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
	rightPlane->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
	rightPlane->setInvMass(0);

	SmartPointer<Geometry> frontPlane = geometrySystem.createPlane(
		rigidBodyPlane4);
	frontPlane->setVisibilityState(false);
	frontPlane->setPosition( Vector3<float>(0,0,300));
	frontPlane->setOrientation(Quaternion(M_PI,Vector3<float>(0,1,0)));
	frontPlane->getRigidBody()->setIsDynamicFlag(false);
	frontPlane->setInvMass(0);

	SmartPointer<Geometry> backPlane = geometrySystem.createPlane(
		rigidBodyPlane5);
	backPlane->setVisibilityState(false);
	backPlane->setPosition( Vector3<float>(0,0,-300));
//                    frontPlane->setOrientation(Quaternion(,Vector3<float>(0,1,0)));
	backPlane->getRigidBody()->setIsDynamicFlag(false);
	backPlane->setInvMass(0);

	SmartPointer<Geometry> bottomPlane = geometrySystem.createPlane(rigidBodyPlane3);
	bottomPlane->setOrientation(Quaternion(-M_PI/2,Vector3<float>(1,0,0)));
	bottomPlane->setPosition( Vector3<float>(0,-300,0));
	bottomPlane->setVisibilityState(true);
	bottomPlane->getRigidBody()->setIsDynamicFlag(false);
	bottomPlane->setInvMass(0);

	for (int i =0; i<50; i++)
	{
		SmartPointer<RigidBody> rigidBodySphere(rigidBodySystem.create(Id(Id::typeSphere,20+i)));

		SmartPointer<Geometry> mySphere = geometrySystem.createSphere(
			rigidBodySphere,
			150);
		mySphere->setMass(3);
		mySphere->setBounciness(0.95);
		mySphere->setVelocity(Vector3<float>(0.1, 0.0 ,0.3));
//  mySphere->setPosition(Vector3<float>( (rand()/(float)RAND_MAX)*200, (rand()/(float)RAND_MAX)*300, (rand()/(float)RAND_MAX)*200));
		mySphere->setPosition(Vector3<float>(i*40.0, i*400+500, 0.0));

	}

	float duration = 60000;
	Clock time;
	time.init();
	time.start();
	cout << "Running " << duration << " ms form now!" << endl;
	int loopCount = 0;
	while (time.getTicks() < duration)
	{
		++loopCount;
		SimonState::exemplar()->setGravityVector( Vector3<float>(0, -0.00098, 0));
    	rigidBodySystem.addGravity();
        GeometrySystem::vIterator end = geometrySystem.end();

		for (int i=0; i<NUM_COLLISION_CHECKS;i++)
		{
			geometrySystem.resolveCollisions(TIME_STEP*SPEED_FACTOR);
		}

		rigidBodySystem.integrateVelocities(TIME_STEP*SPEED_FACTOR);
		geometrySystem.resolveContacts(TIME_STEP*SPEED_FACTOR);
		rigidBodySystem.integratePositions(TIME_STEP*SPEED_FACTOR);
	}
	cout << "I could compute " << loopCount << " steps in this time!" << endl;
}

