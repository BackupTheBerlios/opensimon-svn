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
 * \brief Dieses Test-Environment testet die belastbarkeit unsere Engine.
 */

#include "test-environment.h"
#include "TestEnvironment.h"

static GeometrySystem geometrySystem;
static RigidBodySystem bodySystem;
static ConstraintSystem constraintSystem;

//! Anzahl der Objekte die erzeugt werden sollen
static int numOfObjects = 100;

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {

	//! gravitation aufrechnen
	bodySystem.addGravity();

	geometrySystem.drawGeometries();

	geometrySystem.resolveCollisions();

	constraintSystem.step();
		
	bodySystem.integrateEulerVelocities(10);
	geometrySystem.resolveContacts(10);
	bodySystem.integrateEulerPositions(10);
	//bodySystem.integrateRungeKutta(getLastTime());

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

	Id planeId(Id::typePlane, 0);

	SmartPointer<RigidBody> body = SmartPointer<RigidBody>(bodySystem.create(planeId));

	body->setIsDynamicFlag(false);

	//  etwas drehen, damit die Ebene richtig ausgerichtet ist
	body->setOrientation(Quaternion(-M_PI/2, Vector3<float>(1,0,0)));

	SmartPointer<Geometry> plane = geometrySystem.createPlane(body);

	// eine Fette Kugel

	Id sphereId(Id::typeSphere, 0);

	SmartPointer<RigidBody> bodySphere = SmartPointer<RigidBody>(bodySystem.create(sphereId));
	
	bodySphere->setPosition(Vec3(0, 400, 0));
	bodySphere->setMass(1500);

	SmartPointer<Geometry> sphere = geometrySystem.createSphere(bodySphere, 60);
	sphere->setBounciness(0.6);		

	for (int i = 1; i <= numOfObjects; i++) {

//! komischwerweise brauchen wir rand() bzw. unter linux random().
//! \todo Das muss sich ändern.
#ifdef OS_WINDOWS
		int randX = rand();
		int randY = rand();
		int randZ = rand();
		int randId = rand();
#endif
#ifdef OS_LINUX 
		int randX = random();
		int randY = random();
		int randZ = random();
		int randId = random();		
#endif
		Id sphereId(Id::typeSphere, i);

		SmartPointer<RigidBody> bodySphere = SmartPointer<RigidBody>(bodySystem.create(sphereId));
	
		bodySphere->setPosition(Vector3<float>((randX % 200) - 100, (randY % 100), (randZ % 200) - 100));
		bodySphere->setMass(5.0);

		SmartPointer<Geometry> sphere = geometrySystem.createSphere(bodySphere, 5);
		sphere->setBounciness(0.9);		
		
	    // --- Constraint mit einem der Vorgänger

		// der rest der Schleife funktioniert noch nicht (Speicherzugriffsfehler)
		continue;
		assert(false);
		
		if (i < 2)
			continue;

		cout << randId % (i-1) << endl;
		assert (bodySystem.getRigidBody(Id(Id::typeSphere, randId % (i-1))));
		assert (body);
		Id jointId(Id::typeBallJoint, i);
		constraintSystem.createBallAndSocketConstraint(jointId,
													   bodySystem.getRigidBody(Id(Id::typeSphere, randId % (i-1))),
													   body,
													   Vec3(10,0,0),
													   Vec3(10,0,0));

	}

	// funktioniert noch nicht, deswegen ein vorzeitiger abbruch
	return;
	assert(false);
	constraintSystem.buildGraphs();
	
	constraintSystem.setTau(60);
	
 	SimonState::exemplar()->setViscositySlowdownAngular(0.4);
 	SimonState::exemplar()->setViscositySlowdownLinear(0.98);

}

TEFUNC void keyHandler(unsigned char /*key*/) {
	;
}

#ifdef OS_WINDOWS
	const TestEnvironment testEnvironmentStress(initialize,displayLoop, keyHandler);
#endif
