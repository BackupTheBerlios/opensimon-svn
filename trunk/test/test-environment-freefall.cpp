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

static GeometrySystem geometrySystem;
static RigidBodySystem rigidBodySystem;
static ConstraintSystem constraintSystem;

// Globaler körper, der an einer Position gehalten werden soll
static RigidBodyPtr fixedBody;
static GeometryPtr capsule;

static int threadLength = 100;
static int chainElementLength = 50;

static Vec3 positionOfFixedBody(0, 300, 0);
static Vec3 positionOfFixedBodyOld (positionOfFixedBody);

static bool connectTwoBodies = false;

static int stepsize = 10;

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {

	// run a few times, beacause of small stepsize
	for(int i = 0; i <= 33/stepsize; ++i) {
		//! gravitation aufrechnen
		rigidBodySystem.addGravity();

		// fixieren
		if (fixedBody) {

			// fixedBody->addForce(-1.0 * SimonState::exemplar()->getGravityVector() * fixedBody->getMass());

			//Vec3 direction = (positionOfFixedBody - fixedBody->getPosition());
			//if (direction.length() > 5) {
			//	direction.normalize();
			//	direction *= 0.01;
			//	fixedBody->addForce(direction * fixedBody->getMass());
			//}
			//fixedBody->setForce (Vec3 (0.0, 0.0, 0.0));
			// fixedBody->setTorque (Vec3 (0.0, 0.0, 0.0));
			//Vec3 velocity = 0.1 * (positionOfFixedBody - fixedBody->getPosition());
			//Vec3 force = 0.1 * fixedBody->getMass() * (velocity - fixedBody->getVelocity());
			//fixedBody->setVelocity (velocity);
			//fixedBody->setForce (0.1 * force);

			// Federberechnung, funktioniert
			Vec3 positionRel (positionOfFixedBody - fixedBody->getPosition ());
			Vec3 velocityOfFixedBody (positionOfFixedBody - positionOfFixedBodyOld);

			// Original: Vec3 velocityRel (velocityOfFixedBody - fixedBody->getVelocity ());
			// Geht auch:
			Vec3 velocityRel (fixedBody->getVelocity ());
			float constantSpring (10);
			float constantDamping (0.01);
			float distance (0.0f);
			float lengthRel (positionRel.length ());
			Vec3 directionRel (0.0, 0.0, 0.0);
			if (lengthRel)
				directionRel = (1.0/lengthRel) * positionRel;
			float coefficientHooke (+1.0 * constantSpring * (lengthRel - distance));
			float coefficientDamping (+1.0 * constantDamping * (dot (velocityRel, directionRel)));
			Vec3 force ((coefficientHooke + coefficientDamping) * directionRel);
			fixedBody->setForce (force);
		}
			
		//! zeichne alle geometrie
		geometrySystem.drawGeometries();

		//! berechne kollisionen
		for (int i = 0; i < 5; ++i){
			;
		}
		geometrySystem.resolveCollisions(stepsize);


		if (connectTwoBodies) {
			constraintSystem.step();
			//constraintSystem.computePostStabilization();
		}


		rigidBodySystem.integrateEulerVelocities(stepsize);
		geometrySystem.resolveContacts(stepsize);
		rigidBodySystem.integrateEulerPositions(stepsize);

	}
}

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung
 * aufgerufen. Hier sollte alles reingeschrieben werden,
 * was für die initialisierung der einzelnen Tests nötig ist.
 */
TEFUNC void initialize(int /*argc*/, char** /*argv*/) {

//   	Vector3<float> gravity(0.0, -0.002, 0.0);
//   	SimonState::exemplar()->setGravityVector(gravity);

	Id planeId(Id::typePlane, 0);

 	// erzeuge einen Festkörper für die Ebene
	RigidBodyPtr body = rigidBodySystem.create(planeId);
	
	// die Ebene sollte nicht vom Integrator behandelt werden.
	body->setIsDynamicFlag(false);

	//  etwas drehen, damit die Ebene richtig ausgerichtet ist
	body->setOrientation(Quaternion(0, Vec3(0,1,0)));

	// erzeuge eine Ebene, mit dem oben defineriten Festkörper
 	GeometryPtr plane = geometrySystem.createPlane(body);
 	plane->setBounciness(0.7);

	Id capsuleId(Id::typeCapsule, 0);

	// hohle neuen RigidBody 
    body = rigidBodySystem.create(capsuleId);
	
	// setzen einiger attribute
	body->setPosition(Vector3<float>(0.0,300.0,0.0));
	body->setMass(200.0);
	
	// setzen der geometrie
	capsule = geometrySystem.createBox(body, Vec3(10,60,10));
	capsule->setBounciness(0.7);

	body->setOrientation(Quaternion(-M_PI/5, Vec3(1,0,0)));
	
	constraintSystem.setComputationAlgorithm(4);

 	SimonState::exemplar()->setViscositySlowdownAngular(0.9);
 	SimonState::exemplar()->setViscositySlowdownLinear(0.97);

	cout << "Control: " << endl
		 << "  't' to onnect the two bodies" << endl
		 << "  '8', '5' for Up and Down" << endl
		 << "  '4', '6' for Left and Right" << endl 
		 << endl;
}

void createThread() {
	// ------------ Now comes a Sphere with a thread (represented by some ballASocket-Constraints --- //

	Id sphereId(Id::typeSphere, 0);
	
	// hohle neuen RigidBody 
    RigidBodyPtr body = rigidBodySystem.create(sphereId);
	
	// setzen einiger attribute
	body->setPosition(positionOfFixedBody);
	body->setMass(5000000.0);
	
	// setzen der geometrie
	GeometryPtr sphere = geometrySystem.createSphere(body, 30);
	sphere->setBounciness(0.1);


	// save sphere for further access into a global variable
	fixedBody = body;

	RigidBodyPtr lastBody = body;

	Id id(Id::typeCapsule, 100);
	// every body is chainElementLength mm long
	int chainCount = threadLength/chainElementLength;
	for (int i = 1; i < chainCount; ++i) {
		body = rigidBodySystem.create(id);
		body->setMass(100);
 		Vec3 direction = (capsule->getPosition() - sphere->getPosition());
 		direction.normalize();
 		body->setPosition(sphere->getPosition() + direction * chainElementLength * i);

		float r = 1, h = chainElementLength;
		float mass = body->getMass();
		float tElement1 = (1.0/12.0) * mass * h * h + 0.25 * mass * r * r;
		float tElement2 = 0.5 * mass * r * r;
		body->setInertiaTensor(tElement1, tElement1, tElement2);
		
		//geometrySystem.createCapsule(body, 1, chainElementLength);

		constraintSystem.createBallAndSocketConstraint(Id(Id::typeBallJoint,id.getNumber()),
													   lastBody,
													   body,
													   Vec3(0,-chainElementLength,0),
													   Vec3(0,chainElementLength,0));

		lastBody = body;
		
		id.setNumber(id.getNumber() + 1);
	}

	constraintSystem.createBallAndSocketConstraint(Id(Id::typeBallJoint,id.getNumber()),
												   lastBody,
												   capsule->getRigidBody(),
												   Vec3(0,-chainElementLength,0),
												   Vec3(0,70,0));


	constraintSystem.buildGraphs();
}


TEFUNC void keyHandler(unsigned char key) {
	
	float stepsize = 20.0;
	positionOfFixedBodyOld = positionOfFixedBody;

	switch (key) {
	case 't' :
		connectTwoBodies = true;
		createThread();
		break;
	case '8' : 
		positionOfFixedBody += Vec3(0,stepsize,0);
		break;
	case '5' : 
		positionOfFixedBody += Vec3(0,-stepsize,0);
		break;
	case '4' : 
		positionOfFixedBody += Vec3(-stepsize,0,0);
		break;
	case '6' :
		positionOfFixedBody += Vec3(stepsize,0,0);
		break;
	default:
		break;
	}

}

#ifdef OS_WINDOWS
	const TestEnvironment testEnvironmentFreefall(initialize,displayLoop,keyHandler);
#endif
