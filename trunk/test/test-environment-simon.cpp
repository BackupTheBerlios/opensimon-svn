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


// $Id: test-environment-simon.cpp,v 1.75 2004/10/30 18:22:09 trappe Exp $
//! Standard und Basis Tests für Simon. Hier wird der neue Kram immer erst mal ausgetestet.


#include "test-environment.h"
#include "TestEnvironment.h"

// createing all Systems
static RigidBodySystem bodySystem;
static GeometrySystem geometrySystem;
static ConstraintSystem constraintSystem;
bool loop = false;
bool move = false;
static bool postStabilization = false;

RigidBodyPtr sph1acc;
RigidBodyPtr sph6acc;
RigidBodyPtr sph7acc;
RigidBodyPtr sph8acc;
RigidBodyPtr sph9acc;

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {
	if (loop) {
		//! gravitation aufrechnen
		bodySystem.addGravity();
	
		geometrySystem.resolveCollisions(10);
	
		constraintSystem.step();
		if (postStabilization)
			constraintSystem.computePostStabilization ();
		
		//! Integriere so weit, wie der letzte simulations Schritt brauchte
		//bodySystem.integrateRungeKutta(getLastTime());
		//bodySystem.integrateEuler(getLastTime());
		
		bodySystem.integrateEulerVelocities(10);
		geometrySystem.resolveContacts(10);
		bodySystem.integrateEulerPositions(10);
					
		// für schweben : -1.170
		sph1acc->addForce(-1.10 * SimonState::exemplar()->getGravityVector()* sph1acc->getMass());
		
		if (move) {
			sph6acc->addForce(Vec3(5.0,5.0,0.0));
			sph7acc->addForce(Vec3(-5.0,5.0,0.0));
			sph8acc->addForce(Vec3(0.0,5.0,5.0));
			sph9acc->addForce(Vec3(0.0,5.0,-5.0));
		}
		//sph->addForce(50 * Vec3(1.0,0.0,0.0));
	
		
	}
	geometrySystem.drawGeometries();
	//cout << getLastTime() << endl;
}

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung
 * aufgerufen. Hier sollte alles reingeschrieben werden,
 * was für die initialisierung der einzelnen Tests nötig ist.
 */
TEFUNC void initialize(int /*argc*/, char** /*argv*/) {

	// ------- Ein Paar Ausgaben, um die Funktionsweise vom Logging zu zeigen. -----//
	SimonState::exemplar()->errors <<"Simon: Dies ist eine Fehler-Meldung!!!!" << SimonState::endm;
	SimonState::exemplar()->errors <<"Simon: Dies ist eine weitere Fehler-Meldung!!!!" << SimonState::endm;
	SimonState::exemplar()->errors <<"Simon: Dies ist eine weitere Fehler-Meldung2!!!!" << SimonState::endm;
	SimonState::exemplar()->errors <<"Simon: Dies ist eine weitere Fehler-Meldung3!!!!" << SimonState::endm;
	SimonState::exemplar()->errors <<"Simon: Dies ist eine weitere Fehler-Meldung4!!!!" << SimonState::endm;

	SimonState::exemplar()->messages <<"Simon: Dies ist eine normale Message!!!!" << SimonState::endm;
	SimonState::exemplar()->messages <<"Simon: Dies ist eine witer ueberfluessige Meldung!!!!" << SimonState::endm;
	SimonState::exemplar()->messages <<"Collision: Hier spricht das Kollisionssystem!!!" << SimonState::endm;

	// ---------------- Ende der Log-Tests -----------------//


	// use low gravity for testing
 	//Vector3<float> gravity(0.0, -0.00089, 0.0);
 	//SimonState::exemplar()->setGravityVector(gravity);

	Id planeId(Id::typePlane, 0);

 	// erzeuge einen Festkörper für die Ebene
	SmartPointer<RigidBody> body = bodySystem.create(planeId);
	
	// die Ebene sollte nicht vom Integrator behandelt werden.
	body->setIsDynamicFlag(false);

	//  etwas drehen, damit die Ebene richtig ausgerichtet ist
	body->setOrientation(Quaternion(0, Vector3<float>(1,0,0)));

	body->setPosition(Vector3<float>(0.0,-200.0,0.0));

	// erzeuge eine Ebene, mit dem oben defineriten Festkörper
	SmartPointer<Geometry> plane = geometrySystem.createPlane(body);


	

	Id sphereId1(Id::typeSphere, 0);
	Id sphereId2(Id::typeSphere, 1);
	Id sphereId3(Id::typeSphere, 2);
	Id sphereId4(Id::typeSphere, 3);
	Id sphereId5(Id::typeSphere, 4);
	
	Id sphereId6(Id::typeSphere, 5);
	Id sphereId7(Id::typeSphere, 6);
	
	Id sphereId8(Id::typeSphere, 7);
	Id sphereId9(Id::typeSphere, 8);
	

	
	RigidBodyPtr sphb1 = bodySystem.create(sphereId1);
	RigidBodyPtr sphb2 = bodySystem.create(sphereId2);
	RigidBodyPtr sphb3 = bodySystem.create(sphereId3);
	RigidBodyPtr sphb4 = bodySystem.create(sphereId4);
	RigidBodyPtr sphb5 = bodySystem.create(sphereId5);
	
	
	//Kontrollkugeln
	RigidBodyPtr sphb6 = bodySystem.create(sphereId6);
	RigidBodyPtr sphb7 = bodySystem.create(sphereId7);
	
	//Fallende Kugeln
	RigidBodyPtr sphb8 = bodySystem.create(sphereId8);
	RigidBodyPtr sphb9 = bodySystem.create(sphereId9);
	
	
	sphb1->setMass(10000.0);
	sphb2->setMass(100.0);
	sphb3->setMass(100.0);
	sphb4->setMass(100.0);
	sphb5->setMass(100.0);
	
	sphb6->setMass(100.0);
	sphb7->setMass(100.0);
	sphb8->setMass(100.0);
	sphb9->setMass(100.0);
	
	
	
	
	SmartPointer<Geometry> sph1 = geometrySystem.createSphere(sphb1, 50);
	SmartPointer<Geometry> sph2 = geometrySystem.createSphere(sphb2, 30);
	SmartPointer<Geometry> sph3 = geometrySystem.createSphere(sphb3, 30);
	SmartPointer<Geometry> sph4 = geometrySystem.createSphere(sphb4, 30);
	SmartPointer<Geometry> sph5 = geometrySystem.createSphere(sphb5, 30);
	
	SmartPointer<Geometry> sph6 = geometrySystem.createSphere(sphb6, 10);
	SmartPointer<Geometry> sph7 = geometrySystem.createSphere(sphb7, 10);
	SmartPointer<Geometry> sph8 = geometrySystem.createSphere(sphb8, 10);
	SmartPointer<Geometry> sph9 = geometrySystem.createSphere(sphb9, 10);
	
	
	
	sph1->setBounciness(0.0);
	sph2->setBounciness(0.0);
	sph3->setBounciness(0.0);
	sph4->setBounciness(0.0);
	sph5->setBounciness(0.0);
	sph6->setBounciness(0.0);
	sph7->setBounciness(0.0);
	sph8->setBounciness(0.0);
	sph9->setBounciness(0.0);

	
	Id capsuleId1(Id::typeCapsule, 0);
	Id capsuleId2(Id::typeCapsule, 1);
	Id capsuleId3(Id::typeCapsule, 2);
	Id capsuleId4(Id::typeCapsule, 3);
	Id capsuleId5(Id::typeCapsule, 4);
	
	Id capsuleId6(Id::typeCapsule, 5);
	Id capsuleId7(Id::typeCapsule, 6);
	Id capsuleId8(Id::typeCapsule, 7);
	Id capsuleId9(Id::typeCapsule, 8);

	// hohle neuen RigidBody 
    	RigidBodyPtr capsb1 = bodySystem.create(capsuleId1);
	RigidBodyPtr capsb2 = bodySystem.create(capsuleId2);
	RigidBodyPtr capsb3 = bodySystem.create(capsuleId3);
	RigidBodyPtr capsb4 = bodySystem.create(capsuleId4);
	RigidBodyPtr capsb5 = bodySystem.create(capsuleId5);
	RigidBodyPtr capsb6 = bodySystem.create(capsuleId6);
	RigidBodyPtr capsb7 = bodySystem.create(capsuleId7);
	RigidBodyPtr capsb8 = bodySystem.create(capsuleId8);
	RigidBodyPtr capsb9 = bodySystem.create(capsuleId9);
	
	// setzen einiger attribute
	
	sphb1->setPosition(Vector3<float>(0.0,700.0,0.0));//Kopf
	capsb1->setPosition(Vector3<float>(0.0,520.0,0.0));//Körper
	
	capsb2->setPosition(Vector3<float>(120.0,600.0,0.0));//linker Oberarm
	capsb3->setPosition(Vector3<float>(120.0,460.0,0.0));//linker Unterarm
	sphb2->setPosition(Vector3<float>(120.0,380.0,0.0));//linke Hand
	sphb6->setPosition(Vector3<float>(120.0,340.0,0.0));//linke Hand Steuerung
	
	capsb4->setPosition(Vector3<float>(-120.0,600.0,0.0));//rechter Oberarm
	capsb5->setPosition(Vector3<float>(-120.0,460.0,0.0));//rechter Unterarm
	sphb3->setPosition(Vector3<float>(-120.0,380.0,0.0));//rechte Hand
	sphb7->setPosition(Vector3<float>(-120.0,340.0,0.0));//rechte Hand Steuerung
	
	capsb6->setPosition(Vector3<float>(40.0,300.0,0.0));//linker Oberschenkel
	capsb7->setPosition(Vector3<float>(40.0,160.0,0.0));//linker Unterschenkel
	sphb4->setPosition(Vector3<float>(40.0,80.0,0.0));//linker Fuss
	sphb8->setPosition(Vector3<float>(40.0,40.0,0.0));//linker Fuss Steuerung
	
	
	capsb8->setPosition(Vector3<float>(-40.0,300.0,0.0));//rechter Oberschenkel
	capsb9->setPosition(Vector3<float>(-40.0,160.0,0.0));//rechter Unterschenkel
	sphb5->setPosition(Vector3<float>(-40.0,80.0,0.0));//rechter Fuss
	sphb9->setPosition(Vector3<float>(-40.0,40.0,0.0));//rechter Fuss Steuerung
	
	capsb1->setOrientation(Quaternion(0,Vec3(0.0,0.0,1.0)));
	//capsb2->setOrientation(Quaternion(M_PI/2,Vec3(0.0,0.0,1.0)));
	//capsb3->setOrientation(Quaternion(M_PI/2,Vec3(0.0,0.0,1.0)));
	//capsb4->setOrientation(Quaternion(M_PI/2,Vec3(0.0,0.0,1.0)));
	//capsb5->setOrientation(Quaternion(M_PI/2,Vec3(0.0,0.0,1.0)));
	//capsb6->setOrientation(Quaternion(M_PI/2,Vec3(0.0,0.0,1.0)));
	//capsb7->setOrientation(Quaternion(M_PI/2,Vec3(0.0,0.0,1.0)));
	//capsb8->setOrientation(Quaternion(M_PI/2,Vec3(0.0,0.0,1.0)));
	//capsb9->setOrientation(Quaternion(M_PI/2,Vec3(0.0,0.0,1.0)));
	
	capsb1->setMass(100.0);
	capsb2->setMass(100.0);
	capsb3->setMass(100.0);
	capsb4->setMass(100.0);
	capsb5->setMass(100.0);
	capsb6->setMass(100.0);
	capsb7->setMass(100.0);
	capsb8->setMass(100.0);
	capsb9->setMass(100.0);
	
	// setzen der geometrie
	GeometryPtr caps1 = geometrySystem.createCapsule(capsb1, 40, 160);
	GeometryPtr caps2 = geometrySystem.createCapsule(capsb2, 30, 60);
	GeometryPtr caps3 = geometrySystem.createCapsule(capsb3, 30, 60);
	GeometryPtr caps4 = geometrySystem.createCapsule(capsb4, 30, 60);
	GeometryPtr caps5 = geometrySystem.createCapsule(capsb5, 30, 60);
	GeometryPtr caps6 = geometrySystem.createCapsule(capsb6, 30, 60);
	GeometryPtr caps7 = geometrySystem.createCapsule(capsb7, 30, 60);
	GeometryPtr caps8 = geometrySystem.createCapsule(capsb8, 30, 60);
	GeometryPtr caps9 = geometrySystem.createCapsule(capsb9, 30, 60);
	
	sph1->setBounciness(0.3);


	// ---------------- Ende: Große Kapsel in der Mitte ----------------//

	// ---------------- Constraint zwischen Kugel und Kapsel ---- //

	Id jointId1(Id::typeBallJoint, 0);
	Id jointId2(Id::typeBallJoint, 1);
	Id jointId3(Id::typeBallJoint, 2);
	Id jointId4(Id::typeBallJoint, 3);
	Id jointId5(Id::typeBallJoint, 4);
	Id jointId6(Id::typeBallJoint, 5);
	Id jointId7(Id::typeBallJoint, 6);
	Id jointId8(Id::typeBallJoint, 7);
	Id jointId9(Id::typeBallJoint, 8);
	Id jointId10(Id::typeBallJoint, 9);
	Id jointId11(Id::typeBallJoint, 10);
	Id jointId12(Id::typeBallJoint, 11);
	Id jointId13(Id::typeBallJoint, 12);
	Id jointId14(Id::typeBallJoint, 13);
	Id jointId15(Id::typeBallJoint, 14);
	Id jointId16(Id::typeBallJoint, 15);
	Id jointId17(Id::typeBallJoint, 16);

	constraintSystem.createBallAndSocketConstraint(jointId1, sphb1, capsb1, Vec3(0,-70,0), Vec3(0,110,0));
	
	//constraintSystem.createBallAndSocketConstraint(jointId2, capsb2, capsb3, Vec3(0,-65,0), Vec3(0,65,0));
	constraintSystem.createHingeConstraint(jointId2, capsb2, capsb3, Vec3(0,-65,0), Vec3(0,65,0), Vec3(1,0,0), Vec3(1,0,0));
	constraintSystem.createBallAndSocketConstraint(jointId3, capsb1, capsb2, Vec3(100,80,0), Vec3(0,30,0));
	constraintSystem.createBallAndSocketConstraint(jointId10, capsb3, sphb2, Vec3(0,-70,0), Vec3(0,40,0));
	
	constraintSystem.createBallAndSocketConstraint(jointId4, capsb1, capsb4, Vec3(-100,80,0), Vec3(0,30,0));
	//constraintSystem.createBallAndSocketConstraint(jointId5, capsb4, capsb5, Vec3(0,-65,0), Vec3(0,65,0));
	constraintSystem.createHingeConstraint(jointId5, capsb4, capsb5, Vec3(0,-65,0), Vec3(0,65,0), Vec3(1,0,0), Vec3(1,0,0));
	constraintSystem.createBallAndSocketConstraint(jointId11, capsb5, sphb3, Vec3(0,-70,0), Vec3(0,40,0));
	
	//constraintSystem.createBallAndSocketConstraint(jointId6, capsb6, capsb7, Vec3(0,-65,0), Vec3(0,65,0));
	constraintSystem.createHingeConstraint(jointId6, capsb6, capsb7, Vec3(0,-65,0), Vec3(0,65,0), Vec3(1,0,0), Vec3(1,0,0));
	constraintSystem.createBallAndSocketConstraint(jointId7, capsb1, capsb6, Vec3(50,-160,0), Vec3(0,30,0));
	constraintSystem.createBallAndSocketConstraint(jointId12, capsb7, sphb4, Vec3(0,-70,0), Vec3(0,40,0));
	
	
	//constraintSystem.createBallAndSocketConstraint(jointId9, capsb8, capsb9, Vec3(0,-65,0), Vec3(0,65,0));
	constraintSystem.createHingeConstraint(jointId9, capsb8, capsb9, Vec3(0,-65,0), Vec3(0,65,0), Vec3(1,0,0), Vec3(1,0,0));
	constraintSystem.createBallAndSocketConstraint(jointId8, capsb1, capsb8, Vec3(-50,-160,0), Vec3(0,30,0));
	constraintSystem.createBallAndSocketConstraint(jointId13, capsb9, sphb5, Vec3(0,-70,0), Vec3(0,40,0));
	
	//Steuerungen
	constraintSystem.createBallAndSocketConstraint(jointId14, sphb2, sphb6, Vec3(0,-50,0), Vec3(0,0,0));
	constraintSystem.createBallAndSocketConstraint(jointId15, sphb3, sphb7, Vec3(0,-50,0), Vec3(0,0,0));
	constraintSystem.createBallAndSocketConstraint(jointId16, sphb4, sphb8, Vec3(0,-50,0), Vec3(0,0,0));
	constraintSystem.createBallAndSocketConstraint(jointId17, sphb5, sphb9, Vec3(0,-50,0), Vec3(0,0,0));

	// ---------------- Ende: Constraint zwischen Kugel und Kapsel ---- //

	constraintSystem.buildGraphs();
	
	constraintSystem.setTau(60);

	SimonState::exemplar()->setViscositySlowdownAngular(0.8);
 	SimonState::exemplar()->setViscositySlowdownLinear(0.98);

	sph1acc = sphb1;
	sph6acc = sphb6;
	sph7acc = sphb7;
	sph8acc = sphb8;
	sph9acc = sphb9;

}

TEFUNC void keyHandler(unsigned char key) {
	switch (key) {
		case '1': constraintSystem.setComputationAlgorithm(1); break;
		case '2': constraintSystem.setComputationAlgorithm(2); break;
		case '3': constraintSystem.setComputationAlgorithm(3); break;
		case '4': constraintSystem.setComputationAlgorithm(4); break;
		case 'a': 
			if (loop) {
				loop = false;
			} else {
				loop = true;
			}
		break;
		case 'm': 
			if (move) {
				move = false;
			} else {
				move = true;
			}
		break;
		case 'p': postStabilization = true; break;
		case 'P': postStabilization = false; break;
	}

}

#ifdef OS_WINDOWS
	const TestEnvironment testEnvironmentSimon(initialize,displayLoop, keyHandler);
#endif
