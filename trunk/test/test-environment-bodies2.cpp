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


// $Id: test-environment-bodies2.cpp,v 1.23 2004/10/05 17:09:16 trappe Exp $

//------------------------------------------------------------------------------
/**
 * Euler-Integrator-Test:
 * Ein eigener Euler soll mit den bestehenden Integratoren verglichen werden
 */
//------------------------------------------------------------------------------

#include "test-environment.h"
#include "TestEnvironment.h"

#include "math.h"


static GeometrySystem geometrySystem;
static RigidBodySystem rigidBodySystemEuler;
static RigidBodySystem rigidBodySystemRunge;

struct OwnRigidBody {
	//! hier einen kleinen Rigid Body basteln.
} referenceBody;

static Vec3 positionInit(-100,400,340);
static Vec3 velocityInit(0.00, -0.00, 0.0);
static Quaternion orientationInit(0,M_PI/2,M_PI/2);

static Vec3 angularVelocityInit(0.000, 0.0, -0.00);
static float massInit = 50;
static Vec3 boxScaleInit(50,50,50);

static float lambda = -2;

static Clock timer;

static float interval = 0.001;

//static double simulationTime = 0.0;

//! update the self implemented rigid body with the self implementet Euler
TEFUNC void updateReferenceBody() {
	// do Eulerstuff with the refereceBody
}

TEFUNC void updateRigidBody(RigidBodyPtr body) {
	body->setVelocity(lambda * body->getPosition());
	body->setAcceleration(lambda * body->getVelocity());

	body->setAngularVelocity(lambda * body->getOrientation().getEulerRotation());
	body->setAngularAcceleration(lambda * body->getAngularVelocity());
}

TEFUNC void printRigidBody(RigidBodyPtr /*body*/) {
	// print differeces between body und refereceBody
}


TEFUNC void printReferenceBody() {
	// show some values of the refBody
}

TEFUNC void drawReferenceBody() {
	// paint ReferenceBody with Graphics::drawCube(...)
}

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {
//	timer.sleep(1000);

	geometrySystem.drawGeometries();
	drawReferenceBody();

	rigidBodySystemEuler.forEveryRigidBodyCall(&updateRigidBody);
	rigidBodySystemRunge.forEveryRigidBodyCall(&updateRigidBody);

 	rigidBodySystemEuler.integrateEuler(interval);
 	rigidBodySystemRunge.integrateRungeKutta(interval);

	updateReferenceBody();

	rigidBodySystemEuler.forEveryRigidBodyCall(&printRigidBody);
	rigidBodySystemRunge.forEveryRigidBodyCall(&printRigidBody);
	printReferenceBody();
}

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung
 * aufgerufen.
 */
TEFUNC
void initialize(int /*argc*/, char** /*argv*/) {

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

	// ------------- Keine Visosity -------------
	rigidBodySystemEuler.disableViscosity();
	rigidBodySystemRunge.disableViscosity();	
}

TEFUNC void keyHandler(unsigned char /*key*/) {
	;
}

#ifdef OS_WINDOWS
	const TestEnvironment testEnvironmentBodies2(initialize, displayLoop, keyHandler);
#endif
