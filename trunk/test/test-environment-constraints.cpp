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


// $Id: test-environment-constraints.cpp,v 1.34 2004/12/15 11:52:38 alangs Exp $

#include "test-environment.h"
#include "TestEnvironment.h"
#include <iostream>

using namespace std;

static RigidBodySystem rbs;
static ConstraintSystem cs;

static RigidBodyPtr mrb;
static RigidBodyPtr mrb2;

static PrimaryConstraintPtr mc1;
static PrimaryConstraintPtr mc2;

static float zeit = 0.0;

static bool doConstraints = false;
static bool doTorque = false;
static bool integrate = false;
static bool postStabilization = false;

/**
 * \brief Loop-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {
	const std::map<Id, RigidBodyPtr >* list = rbs.getMap();

	//Clock *zeit = new Clock();
	float interval = getLastTime();

	zeit += interval;

	if (doConstraints) {
		try {
			cs.step();
		} catch(std::exception& e) {
			std::cout << e.what() << std::endl;
		}
	}
	
	if (integrate) {
		//rbs.integrateEuler(interval);
		//rbs.integrateEuler(16);
                //rbs.integrateRungeKutta (20);
		rbs.integrateRungeKutta (10);
                //rbs.integrateVerletBaltman (10);
                //rbs.integrateEuler (10);
		if (postStabilization)
			cs.computePostStabilization ();
		//cout << interval << endl;
		rbs.addGravity();
		mrb->addForce(-1.0 * SimonState::exemplar()->getGravityVector()* mrb->getMass());
		
		if (doTorque) {
			mrb->addTorque(Vec3(20000.0,0.0,0.0));
		}
	}

	bool color = true;
	//bool posi = true;
	for (std::map<Id, SmartPointer<RigidBody> >::const_iterator p = list->begin(); p != list->end(); ++p) {
        
		Quaternion q = p->second->getOrientation();
		Vector3<float> axis;
		float angle;
		q.getAxisAngle(axis, angle);
		angle = RADTODEG(angle);
		Vector3<float> pos = p->second->getPosition();
		
		glLineWidth(1);
		
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
					glBegin(GL_LINES);
						glVertex3f(-30,0,0);
						glVertex3f(-30,0,90);
					glEnd();	
				glPopMatrix();
				
				glPushMatrix();
				glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, Graphics::blue);
				glTranslatef(30.0,0.0,0.0);
				glutSolidSphere(10.0,5,5);
				glPopMatrix();
				glPushMatrix();
					glBegin(GL_LINES);
						glVertex3f(30,0,0);
						glVertex3f(30,0,90);
					glEnd();	
				glPopMatrix();
				
			
		glPopMatrix();		
		
	}

	//sleep (1.0);
  
}


/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung 
 * aufgerufen. Hier sollte alles reingeschrieben werden, 
 * was für die initialisierung der einzelnen Tests nötig ist.
 */
TEFUNC void initialize(int /*argc*/, char** /*argv*/) {
	Id id1;
	id1.setType(Id::typeBox);
	id1.setNumber(1);
	SmartPointer<RigidBody> rb = rbs.create(id1);
	
	rb->setMass(200000);
	rb->setPosition(Vector3<float>(30.0, 0.0, 0.0));
	rb->setPrevPosition(Vector3<float>(30.0, 0.0, 0.0));
	
	
	rb->setVelocity(Vector3<float>(0.00, 0.00, 0.00));
	
	//rb->setForce(Vector3<float>(0.000, 0.00, 0.00));
	rb->setForce(Vector3<float>(0.0, 0.00,0.00));
	
    
	
	Quaternion q = Quaternion();
	//q.normalize();
	rb->setOrientation(q);
	rb->setAngularVelocity(Vector3<float>(0.0, 0.0, 0.0));
	rb->setInertiaTensor((rb->getMass()/12) * 7200, (rb->getMass()/12) * 7200, (rb->getMass()/12) * 7200);
	
	//rb->setTorque(Vector3<float> (0.0, 0.00, -0.001));
 	rb->setTorque(Vector3<float> (0.0, 0.0, 0.0));	
	

	Id id2;
	id2.setType(Id::typeBox);
	id2.setNumber(2);
	SmartPointer<RigidBody> rb2 = rbs.create(id2);
	
	rb2->setMass(200);
	rb2->setPosition(Vector3<float>(-30.0, 0.0, 0.0));
	rb2->setPrevPosition(Vector3<float>(-30.0, 0.0, 0.0));
	rb2->setVelocity(Vector3<float>(0.0, 0.00, 0.00));
	
	//rb2->setForce(Vector3<float>(0.0,0.0, 0.0));
	rb2->setForce(Vector3<float>(0.0,0.0,0.0));
    
	Quaternion q2 = Quaternion();
	//q2.normalize();
	rb2->setOrientation(q2);
	rb2->setAngularVelocity(Vector3<float>(0.0, 0.0, 0.0));
	rb2->setInertiaTensor((rb2->getMass()/12) * 7200, (rb2->getMass()/12) * 7200, (rb2->getMass()/12) * 7200);
	
	//rb2->setTorque(Vector3<float> (0.0, 0.0, 0.001));
	rb2->setTorque(Vector3<float> (0.0, 0.0,0.0));
	
	Id id3;
	id3.setType(Id::typeBox);
	id3.setNumber(3);
	SmartPointer<RigidBody> rb3 = rbs.create(id3);
	
	rb3->setMass(200);
	rb3->setPosition(Vector3<float>(-90.0, 0.0, 0.0));
	rb3->setPrevPosition(Vector3<float>(-90.0, 0.0, 0.0));
	rb3->setVelocity(Vector3<float>(0.0, 0.00, 0.00));
	
	//rb3->setForce(Vector3<float>(0.0,0.0, 0.0));
	rb3->setForce(Vector3<float>(0.0,0.0,0.0));
    
	Quaternion q3 = Quaternion();
	//q2.normalize();
	rb3->setOrientation(q3);
	rb3->setAngularVelocity(Vector3<float>(0.0, 0.0, 0.0));
	rb3->setInertiaTensor((rb3->getMass()/12) * 7200, (rb3->getMass()/12) * 7200, (rb3->getMass()/12) * 7200);
	
	//rb3->setTorque(Vector3<float> (0.0, 0.0, 0.001));
	rb3->setTorque(Vector3<float> (0.0, 0.0,0.0));
	
	Id cid1;
	cid1.setType(Id::typeHingeJoint);
	cid1.setNumber(1);
	Id cid2;
	cid2.setType(Id::typeBallJoint);
	cid2.setNumber(2);
	
	Vector3 <float>  eq(-30, 0, 0);
	Vector3 <float> eq2(30, 0, 0);
	//Vector3 <float>  eu(-30, 90, 0);
	Vector3 <float>  eu(0, 0, 30);
        //Vector3 <float>  eu(10, 0, 0.0000000000000000000285);
        //Vector3 <float>  eu(10, 0, 0);
	//Vector3 <float>  eu2(30, 90, 0);
	Vector3 <float>  eu2(0, 0, 30);
	
	//cs.createBallAndSocketConstraint(cid1, rb, rb2, eq, eq2);
	cs.createHingeConstraint(cid1, rb, rb2, eq, eq2, eu, eu2);
	//cs.createBallAndSocketConstraint(cid2, rb2, rb3, eq, eq2);
	cs.createHingeConstraint(cid2, rb2, rb3, eq, eq2, eu, eu2);
	//mc1 = cs.getHingeConstraint(cid1);
	
	
	
	//cs.createBallAndSocketConstraint(cid1, rb, rb2, eq, eq2);
	cs.buildGraphs();
	
	mrb = rb;
	mrb2 = rb2;
	
	//mc1 = cs.getBallAndSocketConstraint(cid1);
	//mc2 = cs.getBallAndSocketConstraint(cid2);
	
	cs.printGraphs();
	cs.setComputationAlgorithm(1);
	cs.setTau(60);
	
 	SimonState::exemplar()->setViscositySlowdownAngular(0.7);
 	SimonState::exemplar()->setViscositySlowdownLinear(0.97);
	
	/*mc1->computeConstraint();
	
	cout << "Constraint:" << endl;
	mc1->getConstraint().show();
	cout << endl;
	cout << "ConstraintDot:" << endl;
	mc1->getConstraintDot().show();*/
	
	//SimonState::exemplar()->setGravityVector(Vec3(0,-0.00181,0));
	
}

TEFUNC void keyHandler(unsigned char key) {
	switch (key) {
		case '1': cs.setComputationAlgorithm(1); break;
		case '2': cs.setComputationAlgorithm(2); break;
		case '3': cs.setComputationAlgorithm(3); break;
		case '4': cs.setComputationAlgorithm(4); break;
		case 'a': integrate = true; doConstraints = true; break;
		case 'A': integrate = false; doConstraints = false; break;
		case 't': if (doTorque) {doTorque=false;} else {doTorque=true;} break;
		case 'b': 
			if (PrimaryConstraint::doesBaumgarteStabilisation()) {
				PrimaryConstraint::doBaumgarteStabilisation(false);
			} else {
				PrimaryConstraint::doBaumgarteStabilisation(true);
			}
			break;
		default:break;
		case 'p': postStabilization = true; break;
		case 'P': postStabilization = false; break;
	}
}

#ifdef OS_WINDOWS
const TestEnvironment testEnvironmentConstraints(initialize,displayLoop,keyHandler);
#endif
