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


/**
 * \brief Dieses Test-Environment testet die Belastbarkeit unsere Engine.
 */

#include "test-environment.h"
#include "TestEnvironment.h"



static GeometrySystem myGeometrySystem;
static RigidBodySystem myRigidBodySystem;
static RigidBodySystem myRigidBodySystemRunge;

//! Anzahl der Objekte die erzeugt werden sollen
static int numOfObjects = 55;

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {

	//! gravitation aufrechnen
	//myRigidBodySystem.addGravity();
	
	myGeometrySystem.drawGeometries();

	myGeometrySystem.resolveCollisions();
	

	myRigidBodySystem.integrateEuler(getLastTime());
//	myRigidBodySystemRunge.integrateRungeKutta(getLastTime());
}

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung
 * aufgerufen. Hier sollte alles reingeschrieben werden,
 * was für die initialisierung der einzelnen Tests nötig ist.
 */
TEFUNC void initialize(int /*argc*/, char** /*argv*/) {

 	Vector3<float> gravity(0,-0.0001,0);
 	SimonState::exemplar()->setGravityVector(gravity);

	
	for (int i = 0; i < numOfObjects; i++) {

//! komischwerweise brauchen wir rand() bzw. unter linux random().
//! \todo Das muss sich Ã¤ndern.
#ifdef OS_WINDOWS
		int randX = rand();
		int randY = rand();
		int randZ = rand();
#endif
#ifdef OS_LINUX 
		int randX = random();
		int randY = random();
		int randZ = random();		
#endif
		//Euler
		Id boxId(Id::typeBox, i);
		
		SmartPointer<RigidBody> bodyBox = SmartPointer<RigidBody>(myRigidBodySystem.create(boxId));
	
		bodyBox->setPosition(Vector3<float>((randX % 200) - 100, (randY % 100), (randZ % 200) - 100));
		bodyBox->setMass(55.0);
		bodyBox->setVelocity(Vector3<float>((randX % 10) * 0.01, (randY % 10) * 0.01, (randZ % 10) * 0.01));
		bodyBox->setAngularVelocity(Vector3<float>((randX % 10) * 0.001, (randY % 10) * 0.001, (randZ % 10) *0.001));
		
		SmartPointer<Geometry> box = myGeometrySystem.createBox(bodyBox, Vector3<float>(50.0, 30.0, 50.0));
		box->setBounciness(0.6);
		box->setColor(Graphics::red);
		
		

		//Runge-Kutta
		Id boxId2(Id::typeBox, i+ numOfObjects);
		
		bodyBox = SmartPointer<RigidBody>(myRigidBodySystemRunge.create(boxId2));
	
		bodyBox->setPosition(Vector3<float>((randX % 200) - 100, (randY % 100), (randZ % 200) - 100));
		bodyBox->setMass(55.0);
		bodyBox->setVelocity(Vector3<float>((randX % 10) * 0.01, (randY % 10) * 0.01, (randZ % 10) * 0.01));
		bodyBox->setAngularVelocity(Vector3<float>((randX % 10) * 0.001, (randY % 10) * 0.001, (randZ % 10) *0.001));
		
		SmartPointer<Geometry> box2 = myGeometrySystem.createBox(bodyBox, Vector3<float>(50.0, 30.0, 50.0));
		box2->setBounciness(0.6);
		box2->setColor(Graphics::blue);

				
	}
}

TEFUNC void keyHandler(unsigned char /*key*/) {
	;
}

#ifdef OS_WINDOWS
	const TestEnvironment testEnvironmentBodies(initialize,displayLoop, keyHandler);
#endif
