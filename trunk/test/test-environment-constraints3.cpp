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

#include "assert.h"

// $Id: test-environment-constraints3.cpp,v 1.45 2004/12/15 11:52:38 alangs Exp $
// TeamGelenke
#include <iostream>

using namespace std;

static RigidBodySystem rbs;
static ConstraintSystem cs;

static RigidBodyPtr mrb;
static RigidBodyPtr mrb2;
static RigidBodyPtr movedBody;

static PrimaryConstraintPtr mc1;
static PrimaryConstraintPtr mc2;

static bool doConstraints = true;
static bool integrate = true;
static bool postStabilization = true;

Clock performance;
Clock performanceCS;

/**
 * \brief Loop-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {

	const std::map<Id, RigidBodyPtr >* list = rbs.getMap();	
	


	performance.start();

 	if (doConstraints) {
		performanceCS.start();
		cs.step();
		performanceCS.stop();
		cout << "whole step: " << performanceCS << endl;
	}

	if (integrate) {
		//rbs.integrateEuler(interval);
		//rbs.integrateRungeKutta(interval);
		//rbs.integrateRungeKutta(10);
		//rbs.integrateImplicitEuler (10);
		rbs.integrateEuler(10);
		//rbs.integrateVerletBaltman (10);
		//performance.stop ();
		//cout << "integrate: " << performance << endl;
		//performance.start ();
		if (postStabilization)
			cs.computePostStabilization (true);
		//performance.stop ();
		//cout << "stabilize: " << performance << endl;
		//cout << interval << endl;
		rbs.addGravity();
		//mrb->setIsDynamicFlag(false);
		mrb->addForce(-1.013 * SimonState::exemplar()->getGravityVector()* mrb->getMass());
	}

	performance.stop();
	cout << "whole simulation: " << performance << endl;
	
	bool color = true;
	//bool posi = true;
	for (std::map<Id, SmartPointer<RigidBody> >::const_iterator p = list->begin(); p != list->end(); ++p) {
        
		Quaternion q = p->second->getOrientation();
		Vector3<float> axis;
		float angle;
		q.getAxisAngle(axis, angle);
		angle = RADTODEG(angle);
		Vector3<float> pos = p->second->getPosition();

		glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2]);
		glRotatef(angle, axis[0], axis[1], axis[2]);
			
			
		if (color) {
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Graphics::red);
			color=false;
		} else {
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Graphics::green);
				
		}
		glutSolidCube(60.0);
				
		glPushMatrix();
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Graphics::yellow);
		glTranslatef(-30.0,0.0,0.0);
		glutSolidSphere(10.0,5,5);
		glPopMatrix();
		glPushMatrix();
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Graphics::blue);
		glTranslatef(30.0,0.0,0.0);
		glutSolidSphere(10.0,5,5);
		glPopMatrix();
			
			
		glPopMatrix();
		
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
	
	cout << "hello everyone " << endl;
	
	Id id1;
	id1.setType(Id::typeBox);
	id1.setNumber(1);
	SmartPointer<RigidBody> rb = rbs.create(id1);
	
	rb->setMass(200000);
	rb->setPosition(Vector3<float>(30.0, 0.0, 0.0));
	rb->setPrevPosition(Vector3<float>(30.0, 0.0, 0.0));
	
	rb->setInertiaTensor((rb->getMass()/12) * 7200, (rb->getMass()/12) * 7200, (rb->getMass()/12) * 7200);
	
	Id id2;
	id2.setType(Id::typeBox);
	id2.setNumber(2);
	SmartPointer<RigidBody> rb2 = rbs.create(id2);
	
	rb2->setMass(200);
	rb2->setPosition(Vector3<float>(-30.0, 0.0, 0.0));
	rb2->setPrevPosition(Vector3<float>(-30.0, 0.0, 0.0));
    
	rb2->setInertiaTensor((rb2->getMass()/12) * 7200, (rb2->getMass()/12) * 7200, (rb2->getMass()/12) * 7200);
	
	Id id3;
	id3.setType(Id::typeBox);
	id3.setNumber(3);
	SmartPointer<RigidBody> rb3 = rbs.create(id3);
	
	rb3->setMass(200);
	rb3->setPosition(Vector3<float>(-90.0, 0.0, 0.0));
	rb3->setPrevPosition(Vector3<float>(-90.0, 0.0, 0.0));

	rb3->setInertiaTensor((rb3->getMass()/12) * 7200, (rb3->getMass()/12) * 7200, (rb3->getMass()/12) * 7200);
	
	Id id4;
	id4.setType(Id::typeBox);
	id4.setNumber(4);
	SmartPointer<RigidBody> rb4 = rbs.create(id4);
	
	rb4->setMass(200);
	rb4->setPosition(Vector3<float>(-150.0, 0.0, 0.0));
	rb4->setPrevPosition(Vector3<float>(-150.0, 0.0, 0.0));
	//rb4->setVelocity(Vector3<float>(1.0, 0.00, 0.00));
	
    
	rb4->setInertiaTensor((rb4->getMass()/12) * 7200, (rb4->getMass()/12) * 7200, (rb4->getMass()/12) * 7200);
	
	Id id5;
	id5.setType(Id::typeBox);
	id5.setNumber(5);
	SmartPointer<RigidBody> rb5 = rbs.create(id5);
	
	rb5->setMass(2000);
	rb5->setPosition(Vector3<float>(-210.0, 0.0, 0.0));
	rb5->setPrevPosition(Vector3<float>(-210.0, 0.0, 0.0));

	rb5->setInertiaTensor((rb5->getMass()/12) * 7200, (rb5->getMass()/12) * 7200, (rb5->getMass()/12) * 7200);
	
	

	Id cid1;
	cid1.setType(Id::typeBallJoint);
	cid1.setNumber(1);
	Id cid2;
	cid2.setType(Id::typeBallJoint);
	cid2.setNumber(2);
	Id cid3;
	cid3.setType(Id::typeBallJoint);
	cid3.setNumber(3);
	
	Id cid4;
	cid4.setType(Id::typeBallJoint);
	cid4.setNumber(4);
	
	Id cid5;
	cid5.setType(Id::typeBallJoint);
	cid5.setNumber(5);
	
	
	Vector3 <float>  eq(-30, 0, 0);
	Vector3 <float> eq2(30, 0, 0);
	
	Vector3 <float>  eu(0, -5, 0);
	Vector3 <float> eu2(0, 5, 0);
	
	/*cs.createBallAndSocketConstraint(cid1, &(*rb5), &(*rb6), eq, eq2);
	cs.createBallAndSocketConstraint(cid2, &(*rb4), &(*rb5), eq, eq2);*/
	/*cs.createBallAndSocketConstraint(cid3, &(*rb3), &(*rb4), eq, eq2);
	cs.createBallAndSocketConstraint(cid4, &(*rb2), &(*rb3), eq, eq2);
	cs.createBallAndSocketConstraint(cid5, &(*rb2), &(*rb), eq, eq2);*/
	
	/*cs.createBallAndSocketConstraint(cid1, &(*rb2), &(*rb), eq, eq2);
	cs.createBallAndSocketConstraint(cid2, &(*rb3), &(*rb2), eq, eq2);
	cs.createBallAndSocketConstraint(cid3, &(*rb4), &(*rb3), eq, eq2);
	cs.createBallAndSocketConstraint(cid4, &(*rb5), &(*rb4), eq, eq2);
	cs.createBallAndSocketConstraint(cid5, &(*rb6), &(*rb5), eq, eq2);*/
	
	
	
	cs.createBallAndSocketConstraint(cid1, rb, rb2, eq, eq2);
	cs.buildGraphs();
	cs.createBallAndSocketConstraint(cid2, rb2, rb3, eq, eq2);
	cs.buildGraphs();
	cs.createBallAndSocketConstraint(cid3, rb3, rb4, eq, eq2);
	cs.buildGraphs();
	cs.createBallAndSocketConstraint(cid4, rb4, rb5, eq, eq2);
	/*cs.createBallAndSocketConstraint(cid5, rb3, rb6, eu, eu2);*/
	cs.buildGraphs();
	
	
	
	mrb = rb;
	mrb2 = rb2;
	movedBody = rb5;

	mc1 = cs.getBallAndSocketConstraint(cid1);
	mc2 = cs.getBallAndSocketConstraint(cid2);
	
	cs.printGraphs();
	cs.setComputationAlgorithm(4);
	cs.setTau(60);
	
	//rbs.setViscositySlowdownAngular(0.05);
	//rbs.setViscositySlowdownLinear(0.5);
 	SimonState::exemplar()->setViscositySlowdownAngular(0.5);
 	SimonState::exemplar()->setViscositySlowdownLinear(0.999);
	//rbs.setViscositySlowdownLinear(0.992);
	//cs.disableMaxForceLengthCheck();
	
	
	/*vector<Matrix<float> > matrixH;
	Matrix<float> klein(3,3);
	Matrix<float> gross(6,6);
	
	klein[0][0] = 4;
	gross[3][3] = 6;
	
	matrixH.push_back(klein);
	matrixH.push_back(klein);
	
	matrixH[0].show();
	matrixH[1].show();
	
	matrixH[0] = gross;
	matrixH[0].show();
	matrixH[1].show();*/
	
	//SimonState::exemplar()->setGravityVector(Vector3<float> (0,-0.0000000981,0));

}

TEFUNC void keyHandler(unsigned char key) {
	switch (key) {
		case '1': cs.setComputationAlgorithm(1); break;
		case '2': cs.setComputationAlgorithm(2); break;
		case '3': cs.setComputationAlgorithm(3); break;
		case '4': cs.setComputationAlgorithm(4); break;
		case 'a': integrate = true; doConstraints = true; break;
		case 'A': integrate = false; doConstraints = false; break;
		case 'p': postStabilization = true; break;
		case 'P': postStabilization = false; break;
		default:break;
	}
}

#ifdef OS_WINDOWS
const TestEnvironment testEnvironmentConstraints3(initialize,displayLoop,keyHandler);
#endif
