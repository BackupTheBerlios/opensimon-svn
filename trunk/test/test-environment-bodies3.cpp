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
 * Rotations-Test:
 * Basierend auf der Gleichung angle = oldangle + angularVelocity * time
 * wird hier der Spezialfall von einer ganzen Umdrehung getestet.
 * \todo andere Testfaelle ausprobieren
 */
//------------------------------------------------------------------------------

#include "test-environment.h"
#include "TestEnvironment.h"

#include "math.h"

static string testCase = "one";

static GeometrySystem geometrySystem;
static RigidBodySystem rigidBodySystemEuler;
static RigidBodySystem rigidBodySystemRunge;
static RigidBodySystem rigidBodySystemVerlet;

static RigidBodyPtr referenceBody(new RigidBody);

static Vec3 positionInit(-100,400,340);
static Vec3 velocityInit(0,0,0);
static Quaternion orientationInit(0,M_PI/2,M_PI);

static Vec3 angularVelocityInit(0,0,0);
static float massInit = 50;
static Vec3 boxScaleInit(50,50,50);

static Vec3 correctPosition(positionInit);
static Vec3 correctVelocity(velocityInit);
static Quaternion correctOrientation(orientationInit);
static Vec3 correctAngularVelocity(angularVelocityInit);
static Vec3 correctRotationAxis;
static float correctRotationAngle;

static float lambda = -0.1;

static Clock timer;

static float interval = 0.1;

static double simulationTime = 0.0;
	

TEFUNC void computeCorrectValuesForOne() {
	simulationTime += interval;

	double scalar = exp(lambda * simulationTime);
	
	correctPosition = scalar * positionInit;
	
	Vec3 axis;
	float angle;
	
	orientationInit.getAxisAngle(axis, angle);
	correctOrientation = Quaternion(scalar * angle, axis);

 	correctAngularVelocity = scalar * angularVelocityInit;
	
	referenceBody->setPosition(correctPosition);
	referenceBody->setVelocity(correctVelocity);
	referenceBody->setOrientation(correctOrientation);
	referenceBody->setAngularVelocity(correctAngularVelocity);

	referenceBody->getOrientation().getAxisAngle(axis, angle);
	correctRotationAxis = axis;
	correctRotationAngle = angle;

	cout << "Correct Values:" << endl
		 << "  Position: " << correctPosition << endl
		 << "  Velocity: " << correctVelocity << endl
		 << "  Orientation: " << correctOrientation << endl
		 << "  AngularVelocity: " << correctAngularVelocity << endl
		 << "  RotationAngle: " << correctRotationAngle << endl
		 << "  RotationAxis: " << correctRotationAxis 
		 << endl << endl;
}

TEFUNC void computeCorrectValuesForTwo() {
	simulationTime += interval;

	double scalar = exp(lambda * simulationTime);
	
	Vec3 correctRotation(scalar * orientationInit.getEulerRotation());
	
	correctPosition = scalar * positionInit;
	correctOrientation = Quaternion(correctRotation[0],
 									correctRotation[1], 
 									correctRotation[2]);
	
	Vec3 axis;
	float angle;
	referenceBody->getOrientation().getAxisAngle(axis, angle);
	correctRotationAxis = axis;
	correctRotationAngle = angle;

	referenceBody->setPosition(correctPosition);
	referenceBody->setVelocity(correctVelocity);
	referenceBody->setOrientation(correctOrientation);
	referenceBody->setAngularVelocity(correctAngularVelocity);

	cout << "Correct Values:" << endl
		 << "  Position: " << correctPosition << endl
		 << "  Orientation: " << correctOrientation << endl
		 << "  RotationAngle: " << correctRotationAngle << endl
		 << "  RotationAxis: " << correctRotationAxis 
		 << endl << endl;
}


TEFUNC void computeCorrectValuesForThree() {
	simulationTime += interval;

	double scalar = exp(lambda * simulationTime);
	
	Vec3 correctRotation(orientationInit.getEulerRotation() + 
						 lambda * simulationTime * 
						 scalar * angularVelocityInit);
	
	correctPosition = positionInit + 
		lambda * simulationTime * 
		scalar * velocityInit;

	correctOrientation = Quaternion(correctRotation[0],
 									correctRotation[1], 
 									correctRotation[2]);
	
	Vec3 axis;
	float angle;
	referenceBody->getOrientation().getAxisAngle(axis, angle);
	correctRotationAxis = axis;
	correctRotationAngle = angle;

	referenceBody->setPosition(correctPosition);
	referenceBody->setVelocity(correctVelocity);
	referenceBody->setOrientation(correctOrientation);
	referenceBody->setAngularVelocity(correctAngularVelocity);

	cout << "Correct Values:" << endl
		 << "  Position: " << correctPosition << endl
		 << "  Orientation: " << correctOrientation << endl
		 << "  RotationAngle: " << correctRotationAngle << endl
		 << "  RotationAxis: " << correctRotationAxis 
		 << endl << endl;
}

//! Do update for the RB by useing the first test case
TEFUNC void updateRigidBodyForOne(RigidBodyPtr body) {

	body->setVelocity(lambda * body->getPosition());	
	body->setAcceleration(lambda * body->getVelocity());

	Vec3 axis;
	float angle;
	body->getOrientation().getAxisAngle(axis, angle);
	Quaternion tmp(lambda * angle, axis);
	body->setAngularVelocity(tmp.getEulerRotation());
	body->setAngularAcceleration(lambda * body->getAngularVelocity());
}

//! Do update for the RB by useing the second test case
TEFUNC void updateRigidBodyForTwo(RigidBodyPtr body) {

	body->setVelocity(lambda * body->getPosition());	

	body->setAngularVelocity(lambda * body->getOrientation().getEulerRotation());

	body->setAngularAcceleration(Vec3(0,0,0));
}


//! Do update for the RB by useing the second test case
TEFUNC void updateRigidBodyForThree(RigidBodyPtr body) {

	body->setAcceleration(lambda * body->getVelocity());

	body->setAngularAcceleration(lambda * body->getAngularVelocity());

	cout << "Accel: " <<  body->getAcceleration() << endl;
}

TEFUNC void printRigidBody(RigidBodyPtr body) {
	Vec3 axis;
	float angle;
	body->getOrientation().getAxisAngle(axis, angle);

	cout << "Differences from " << body->getId() << " to reference:" <<  endl
		 << "  Position: " << body->getPosition() - correctPosition << endl
		 << "  Velocity: " << body->getVelocity() - correctVelocity << endl
		//! \todo komponentenweise subtraktion für quaternionen
		 << "  Orientation: " << body->getOrientation() + (-1 * correctOrientation) << endl
		 << "  AngularVelocity: " << body->getAngularVelocity() - correctAngularVelocity << endl
		 << "  RotationAngle: " << angle - correctRotationAngle << endl
		 << "  RotationAxis: " << axis - correctRotationAxis 
		 << endl << endl;
}


/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {
	//timer.sleep(100);

	geometrySystem.drawGeometries();

	if (testCase == "one") {
		computeCorrectValuesForOne();
		
		rigidBodySystemEuler.forEveryRigidBodyCall(&updateRigidBodyForOne);
		rigidBodySystemRunge.forEveryRigidBodyCall(&updateRigidBodyForOne);
		rigidBodySystemVerlet.forEveryRigidBodyCall(&updateRigidBodyForOne);
	}
	else if (testCase == "two") {
 		computeCorrectValuesForTwo();

		rigidBodySystemEuler.forEveryRigidBodyCall(&updateRigidBodyForTwo);
		rigidBodySystemRunge.forEveryRigidBodyCall(&updateRigidBodyForTwo);
		rigidBodySystemVerlet.forEveryRigidBodyCall(&updateRigidBodyForTwo);		
	}
	else if (testCase == "three") {
 		computeCorrectValuesForThree();

		rigidBodySystemEuler.forEveryRigidBodyCall(&updateRigidBodyForThree);
		rigidBodySystemRunge.forEveryRigidBodyCall(&updateRigidBodyForThree);
		rigidBodySystemVerlet.forEveryRigidBodyCall(&updateRigidBodyForThree);		
	}

	rigidBodySystemEuler.integrateEuler(interval);
 	rigidBodySystemRunge.integrateRungeKutta(interval);
	rigidBodySystemVerlet.integrateVerletBaltman (interval);

	rigidBodySystemEuler.forEveryRigidBodyCall(&printRigidBody);
   	rigidBodySystemRunge.forEveryRigidBodyCall(&printRigidBody);
  	rigidBodySystemVerlet.forEveryRigidBodyCall(&printRigidBody);
		
}

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung
 * aufgerufen.
 */
TEFUNC
void initialize(int /*argc*/, char** /*argv*/) {

	// --------- Ein wenig Testkram, ist unbedeutetnd -------------

	orientationInit.normalize();
	Quaternion test(0,M_PI/2,M_PI/2);
	cout << test << endl
		 << M_PI/2 << endl
		 << test.getEulerRotation() << endl;

//		exit (0);


	//----------------- uhr initialisieren --------------------
	timer.init();
	
	// --- Anlegen der Zeiger und Objekte mit denen gearbeitet werden soll ----

	RigidBodyPtr body;
	GeometryPtr boxGeo;

	// Erste id ist null. Wird dann hochgezählt.
	Id id(Id::typeBox,0);


	// -------------- Einfache Rotation -----------
	//Euler
	id.setNumber(0);
	
	body = rigidBodySystemEuler.create(id);
	
	body->setPosition(positionInit);
	body->setMass(massInit);
	body->setOrientation(orientationInit);
	body->setVelocity(velocityInit);
	body->setAngularVelocity(angularVelocityInit);
		
 	boxGeo = geometrySystem.createBox(body, boxScaleInit);
 	boxGeo->setBounciness(0.6);
 	boxGeo->setColor(Graphics::red);
  

	//Runge-Kutta
	id.setNumber(1);
	
	body = rigidBodySystemRunge.create(id);
	body->setPosition(Vec3(-300.0, 0.0, 0.0));
	
	body->setPosition(positionInit);
	body->setMass(massInit);
	body->setOrientation(orientationInit);
	body->setVelocity(velocityInit);
	body->setAngularVelocity(angularVelocityInit);
		
	boxGeo = geometrySystem.createBox(body, boxScaleInit);
   	boxGeo->setBounciness(0.6);
	boxGeo->setColor(Graphics::blue);

	// Verlet-Baltman
	 id.setNumber(2);

 	body = rigidBodySystemVerlet.create(id);
 	body->setPosition(positionInit);

 	body->setMass(massInit);
 	body->setOrientation(orientationInit);
 	body->setVelocity(velocityInit);
 	body->setAngularVelocity(angularVelocityInit);

		
   	boxGeo = geometrySystem.createBox(body, boxScaleInit);
   	boxGeo->setBounciness(0.6);
   	boxGeo->setColor(Graphics::green);


	referenceBody->setPosition(positionInit);
	referenceBody->setMass(massInit);
	referenceBody->setOrientation(orientationInit);
	referenceBody->setVelocity(velocityInit);
	referenceBody->setAngularVelocity(angularVelocityInit);
		
	boxGeo = geometrySystem.createBox(referenceBody, boxScaleInit);
	boxGeo->setBounciness(0.6);
	boxGeo->setColor(Graphics::yellow);

	// ------------- Keine Visosity -------------
	rigidBodySystemEuler.disableViscosity();
	rigidBodySystemRunge.disableViscosity();
	rigidBodySystemVerlet.disableViscosity();
	
			
}

TEFUNC void keyHandler(unsigned char key) {
	switch (key) {
	case '1': 
		testCase = "one";
		break;
	case '2':
		testCase = "two";
		break;
	case '3':
		testCase = "three";
		break;
	default: 
		break;
	}
	
	cout << "Testcase: " << key << endl;
}

#ifdef OS_WINDOWS
	const TestEnvironment testEnvironmentBodies3(initialize, displayLoop, keyHandler);
#endif
