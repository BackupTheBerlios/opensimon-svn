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


// $Id: test-environment-network.cpp,v 1.68 2004/12/15 12:33:06 alangs Exp $
/**
 * \brief Hier testen wir unsere netzwerk verbindung
 */

#include "../Base/test-environment.h"
#include "../Base/SimonState.h"
#include "../Base/Graphics.h"
#include "../../config.h"
#include "../Simon/TestEnvironment.h"

//Fürs Netzwerk
#include"../../Common/Tools/NetworkManagerSimon.h"

// createing all collecting Systems
RigidBodySystemPtr bodySystem;
CollisionSystemPtr colSystem;
ConstraintSystemPtr constraintSystem;
ParticleSystemPtr particleSystem;
ClothSystemPtr clothSystem;

void sendTestRigidBody(RigidBodyPtr body) {
	// Nur zu testzwecken einkommentieren!!! Nicht ins CVS (trappe)
//	if (body->getId().getNumber()==34) Clock::sleep(10000);
	if (body->getId().getNumber()==0 || body->getId().getNumber()==22) {
//		cout << body->getId() << endl;
//		body->addForce(-1.14 * SimonState::exemplar()->getGravityVector()* body->getMass());
	}

 	if (body->getIsDynamicFlag()) {
 		NetworkManagerSimon::exemplar()->sendTransformation(body);
 	}
}

void sendParticle(ParticlePtr particle) {
 	if (particle->getIsDynamicFlag()) {
 		//NetworkManagerSimon::exemplar()->sendTransformation(particle);
 	}
//	cout << particle->getPosition() << endl;
}

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {

	//Clock::sleep(1000);
	
	//! gravitation aufrechnen
	bodySystem->addGravity();
	particleSystem->addGravity();

	// get new Events from the network and consume them
	NetworkManagerSimon::exemplar()->doReceiveFromCentralLoop();

	particleSystem->checkCollisions();

	// Zwangskräfte ausrechen und wirken lassen
	constraintSystem->step();
	constraintSystem->computePostStabilization();
	//clothSystem->computeCloth();

	// neue Positionen und Orientierenungen berechnen
	bodySystem->integrateRungeKuttaFirstStep(10);
	colSystem->doContacts(10);
	bodySystem->integrateRungeKuttaSecondStep(10);

	particleSystem->integrateRungeKutta(10);
	//particleSystem->integrateEuler(10);
	//bodySystem->integrateEuler(1);

    // Hier neue Bones schicken
	bodySystem->forEveryRigidBodyCall(&(sendTestRigidBody));

	NetworkManagerSimon::exemplar()->sendContactInformationContainer();
	NetworkManagerSimon::exemplar()->sendCloth();

	// benachrichtige den client vom Ende des Datenstroms

	colSystem->drawGeometries();
	particleSystem->drawGeometries();
	clothSystem->drawSpringConnection();


	//	std::cout << "time: " << getLastTime() << std::endl;
}

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung
 * aufgerufen. Hier sollte alles reingeschrieben werden,
 * was für die initialisierung der einzelnen Tests nötig ist.
 */
TEFUNC void initialize(int /*argc*/, char** /*argv*/) {

#ifdef PINGTIMING
	std::cout.precision(10);
#endif

	// gravitation auf slomo setzen (sonst sieht man noch nix)
//	SimonState::exemplar()->setGravityVector(Vec3(0 , -0.0001, 0));
	//SimonState::exemplar()->setGravityVector(Vec3(0, 0, 0));


	//! merken der SimulationsSysteme
	bodySystem =SimonState::exemplar()->getBodySystem();
	colSystem = SimonState::exemplar() ->getCollisionSystem();
	constraintSystem = SimonState::exemplar() ->getConstraintSystem();
	particleSystem = SimonState::exemplar() ->getParticleSystem();
	clothSystem = SimonState::exemplar() ->getClothSystem();

	// NetworkManager initialisieren
	if (!NetworkManagerSimon::exemplar()->init())
		exit(0);
   
	// ------- Eine Ebene als Fussboden definieren 
	//! \todo dies sollte auch übers netzwerk geschehen
	// Die fängt mit 0 an, wir (ritschel) auch, toll
	// habs auf 10000 geändert. Sorry (trappe)
	Id planeId(Id::typePlane, 10000);

 	// erzeuge einen Festkörper für die Ebene
	RigidBodyPtr body = bodySystem->create(planeId);
	
	// die Ebene sollte nicht vom Integrator behandelt werden.
	body->setIsDynamicFlag(false);

	// position anpassen
	body->setPosition(Vec3(0,-10,0));

	//  etwas drehen, damit die Ebene richtig ausgerichtet ist
	body->setOrientation(Quaternion(-M_PI/2, Vector3<float>(1,0,0)));

	// erzeuge eine Ebene, mit dem oben defineriten Festkörper
	PlanePtr plane = static_pointer_cast<Plane>(colSystem->createPlane(body));

	plane->setBounciness(0.2);


	// ----------- Allles in schweres Öl tauchen ----------------

 	SimonState::exemplar()->setViscositySlowdownAngular(0.4);
 	SimonState::exemplar()->setViscositySlowdownLinear(0.98);
	
	//! use the sneaky O(n) Algorithm ;)
	constraintSystem->setComputationAlgorithm(4);

	// Eine kleine Hilfe ausgeben
	cout << "Press 'm' to move all Objets a bit!" << endl;

	particleSystem->lookForCollisionsWith(plane);


// 	Id sphereId(Id::typeSphere, 100000);

// 	SmartPointer<RigidBody> bodySphere = bodySystem->create(sphereId);
	
// 	bodySphere->setPosition(Vec3(10, 20, 15));
// 	bodySphere->setMass(1500);
// 	bodySphere->setIsDynamicFlag(false);

// 	SpherePtr sphere = static_pointer_cast<Sphere>(colSystem->createSphere(bodySphere, 30));
// 	sphere->setBounciness(0.6);		

// 	particleSystem->lookForCollisionsWith(sphere);
}

void setVelocity(RigidBodyPtr body) {
	body->setVelocity(Vec3(0,0.2,0));
}


TEFUNC void keyHandler(unsigned char key) {

	if (key == 'm') {
		bodySystem->forEveryRigidBodyCall(&setVelocity);
	}
}

#ifdef OS_WINDOWS
	const TestEnvironment testEnvironmentNetwork(initialize,displayLoop, keyHandler);
#endif
