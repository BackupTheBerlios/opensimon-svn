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
 * Der hier verwendete Ablauf verdeutlicht wie die Kleidung von Simon verwendet werden soll.
 */

#include "test-environment.h"
#include "TestEnvironment.h"
#include <map>
using namespace std;

static RigidBodySystem bodySystem;
static GeometrySystem geometrySystem;
static ClothSystem clothSystem;
static ParticleSystem particleSystem;

// wie breit soll der stoff sein?

int patchWidth = 20;	// Anzahl der Partikel
int patchLength = patchWidth;
//bei Faktor auf normalSpringLength im ClothSystem achten!!!
float distanceL = 15; // Distanz der Partikel
float distanceB = distanceL;

bool animate = false;
Id capsuleId(Id::typeCapsule,0);
Id sphereId(Id::typeSphere,0);

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {
	
	

	//! Gravitation aufrechnen

	particleSystem.addGravity();
	if (animate)
		bodySystem.addGravity();

	//! Kräfte auf Partikel berechnen und aufaddieren
	clothSystem.computeCloth();
	
	particleSystem.checkCollisions();
	// integriere etwas
	//particleSystem.integrateEuler(10);
	particleSystem.integrateRungeKutta(5);	// getLastTime() wg. Instabilität rausgenommen!!! Werte: 0.01 od. 0.02
	//particleSystem.integrateVerletBaltman(10);
	bodySystem.integrateRungeKutta(10);

	//! zeichne alle geometrie
	geometrySystem.drawGeometries();
	clothSystem.drawSpringConnection();
	particleSystem.drawGeometries(); 
}

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung
 * aufgerufen. Hier sollte alles reingeschrieben werden,
 * was für die initialisierung der einzelnen Tests nötig ist.
 */

TEFUNC void initialize(int /*argc*/, char** /*argv*/) {
	//! hier kann die gravitation geändert werden
  	//Vector3<float> gravity(0.0f, -9.81f, 0.0f);
  	//SimonState::exemplar()->setGravityVector(gravity);

/*-------------------------------------------------------------------------------------------*/
	
	// erzeuge eine Reihe von Kugeln und verbinde diese mit SpringConnections
	// Patch wird über eine Doppelschleife erzeugt
	for (int i = 0; i < patchWidth; i++) 
	     for (int j = 0; j < patchLength; j++){
	    	
  
        // fortlaufende Id
		Id particleId(Id::typeParticle, j+i*patchLength);
		
	   
		ParticlePtr particle = particleSystem.create(particleId);
		//Welche Partikel werden festgehalten. Also nicht integriert...
		/*if (particle->getId().getNumber() % 20  == 0)
		  particle->setIsDynamicFlag(false);*/
        /*if (particle->getId().getNumber() == (patchWidth-1))
            particle->setIsDynamicFlag(false); 
		if (particle->getId().getNumber() == ((patchWidth * patchLength)-1))
		particle->setIsDynamicFlag(false);*/
		
		// vier Eckpartikel werden aufgehangen
		/*if (particle->getId().getNumber() == patchLength - 1)
		  particle->setIsDynamicFlag(false);
		if (particle->getId().getNumber() == patchWidth * patchLength - patchLength)
		  particle->setIsDynamicFlag(false);
		if (particle->getId().getNumber() == 0)
		  particle->setIsDynamicFlag(false);
		if (particle->getId().getNumber() == (patchLength * patchWidth - 1))
		particle->setIsDynamicFlag(false);*/
             	
		// setzen der Position (in x-richtung variable)
		//particle->setPosition(Vector3<float>(distanceL * i, distanceB * j * (-1), 0));	
		particle->setPosition(Vector3<float>(distanceL * i, 50, distanceB * j));	
		particle->setVelocity(Vector3<float>(0.0,0.0,0.0));
		particle->setForce(Vector3<float>(0.0f, 0.0f, 0.0f));
		particle->setIsMoveableFlag(true);
		// fünf gramm sollten reichen ;)
		particle->setMass(2.0f);	// Standard-Wert 5.0f	

		// definere den "Hüpffaktor" - niedrige Priorität
		// particle->setBounciness(0.6);	

		// spingId
		Id springIdX(Id::typeCloth, (j+i*patchLength)+5000);
		Id springIdZ(Id::typeCloth, (j+i*patchWidth));
		//Id springIdXZ1(Id::typeCloth, (j+i*patchLength)+10000);
		
		//Id springIdXZ2(Id::typeCloth, (j+i*patchLength)+15000);
		// eine SpringConnection zwischen Vorgänger und aktuellem Particle erzeugen
	
	if (i != 0)
        
 			clothSystem.createSpringConnection(particleSystem.getParticle(Id(Id::typeParticle,(j+i*patchLength)-patchLength)),
 							   particle, 
 							   springIdX);
	
	
	
	if (j != 0) 
 			clothSystem.createSpringConnection(particleSystem.getParticle(Id(Id::typeParticle,(j+i*patchLength)-1)),
 							   particle, 
 							   springIdZ);

	/*if (i != 0 && j != 0) 
 			clothSystem.createSpringConnection(particleSystem.getParticle(Id(Id::typeParticle,(j+i*patchLength)-patchWidth-1)),
 							   particle, 
 							   springIdXZ1);*/
	
	/*if (i != 0 && j != patchLength)
			clothSystem.createSpringConnection(particleSystem.getParticle(Id(Id::typeParticle,(j+i*patchLength)-patchWidth+1)),
 							   particle, 
 							   springIdXZ2);*/

          	

	}
// das ParticleSystem speichert sich entsprechende SmartPointer auf alle interessante geometrie
// normale Kollisionen werden mit dem CollisionSystem getestet


// ----------- Hier noch eine Kapsel, auf die der Stoff fallen soll -------------

	//Id capsuleId(Id::typeCapsule,0);
	RigidBodyPtr bodyCapsule = RigidBodyPtr(bodySystem.create(capsuleId));	

	bodyCapsule->setIsDynamicFlag(true);
	bodyCapsule->setPosition(Vector3<float>(150,-175,150));

	
	
	bodyCapsule->setOrientation(Quaternion((M_PI / 2.0), Vector3<float>(0.0, 0.0, 1.0)));
	
	bodyCapsule->setMass(20.0);	
	//bodyCapsule->setForce(Vec3(0.0, 0.0, 0.0));
	GeometryPtr capsuleGeo = geometrySystem.createCapsule(bodyCapsule, 50.0, 150.0);
	capsuleGeo->setBounciness(0.6);	
	
	CapsulePtr capsule = boost::static_pointer_cast<Capsule>(capsuleGeo);	
    
	particleSystem.lookForCollisionsWith(capsule);

	/*Id hartSpring(Id::typeCloth, 15000);
	clothSystem.createSpringConnection(particleSystem.getParticle(0), bodyCapsule, Vec3(0.0,50.0,0.0), hartSpring);*/
	

// ----------- Hier eine zweite Kapsel, auf die der Stoff fallen soll --------------------------------------------
/*
	Id capsuleId2(Id::typeCapsule,1);
	RigidBodyPtr bodyCapsule2 = RigidBodyPtr(bodySystem.create(capsuleId2));	

	bodyCapsule2->setIsDynamicFlag(false);
	bodyCapsule2->setPosition(Vector3<float>(100,-175,75));
	bodyCapsule2->setOrientation(Quaternion((M_PI / 2.0), Vector3<float>(1.0, 0.0, 0.0)));
	bodyCapsule2->setMass(20.0);	
	
	GeometryPtr capsuleGeo2 = geometrySystem.createCapsule(bodyCapsule2, 25.0, 75.0);
	capsuleGeo2->setBounciness(0.6);	
	
	CapsulePtr capsule2 = boost::static_pointer_cast<Capsule>(capsuleGeo2);	
    
	particleSystem.lookForCollisionsWith(capsule2);*/

// ----------- Hier noch eine Kugel, auf die der Stoff fallen soll -------------
/*
	Id sphereId(Id::typeSphere,0);
	RigidBodyPtr bodySphere = RigidBodyPtr(bodySystem.create(sphereId));
	
	bodySphere->setIsDynamicFlag(true);	
	bodySphere->setPosition(Vector3<float>(125,150,275));
	bodySphere->setMass(80.0);

	GeometryPtr sphereGeo = geometrySystem.createSphere(bodySphere, 75.0);

	sphereGeo->setBounciness(0.6);

	SpherePtr sphere = boost::static_pointer_cast<Sphere>(sphereGeo);
	particleSystem.lookForCollisionsWith(sphere);
*/


// ----------- Hier noch eine Ebene, auf die der Stoff fallen soll -------------

	/*Id planeId(Id::typePlane, 0);
	RigidBodyPtr bodyPlane = RigidBodyPtr(bodySystem.create(planeId));

	bodyPlane->setIsDynamicFlag(false);
	bodyPlane->setPosition(Vector3<float>(25, -275, 25));
	bodyPlane->setMass(200.0);
	bodyPlane->setOrientation(Quaternion(((3.0*M_PI) / 2.0), Vector3<float>(1.0, 0.0, 0.0)));
	
	GeometryPtr planeGeo = geometrySystem.createPlane(bodyPlane);
	PlanePtr plane = boost::static_pointer_cast<Plane>(planeGeo);

	particleSystem.lookForCollisionsWith(plane);*/


// ----------- Hier noch eine Box, auf die der Stoff fallen soll -------------
	
	/*Id boxId(Id::typeBox, 0);
	RigidBodyPtr bodyBox = RigidBodyPtr(bodySystem.create(boxId));
	
	bodyBox->setIsDynamicFlag(false);	
	bodyBox->setPosition(Vector3<float>(75, -250, 75));
	bodyBox->setMass(200.0);
	bodyBox->setOrientation(Quaternion((M_PI / 2.0), Vector3<float>(0.0, 1.0, 1.0)));
	GeometryPtr boxGeo = geometrySystem.createBox(bodyBox, Vector3<float>(75.0f,75.0f,75.0f));
	BoxPtr box = boost::static_pointer_cast<Box>(boxGeo);
			
    particleSystem.lookForCollisionsWith(box);*/

// ----------- Hier noch eine 2. Box, auf die der Stoff fallen soll -------------
/*	
	Id boxId2(Id::typeBox, 0);
	RigidBodyPtr bodyBox2 = RigidBodyPtr(bodySystem.create(boxId2));
	
	bodyBox2->setIsDynamicFlag(false);	
	bodyBox2->setPosition(Vector3<float>(-25, -200, 75));
	bodyBox2->setMass(200.0);
	bodyBox2->setOrientation(Quaternion(2 * (M_PI / 2.0), Vector3<float>(1.0, 0.0, 0.0)));
	GeometryPtr boxGeo2 = geometrySystem.createBox(bodyBox2, Vector3<float>(100.0f,110.0f,75.0f));
	BoxPtr box2 = boost::static_pointer_cast<Box>(boxGeo2);
			
    particleSystem.lookForCollisionsWith(box2);*/

// ----------- Hier noch eine 3. Box, auf die der Stoff fallen soll -------------
	
	/*Id boxId3(Id::typeBox, 2);
	RigidBodyPtr bodyBox3 = RigidBodyPtr(bodySystem.create(boxId3));
	
	bodyBox3->setIsDynamicFlag(false);	
	bodyBox3->setPosition(Vector3<float>(75, -150, 175));
	bodyBox3->setMass(200.0);
	bodyBox3->setOrientation(Quaternion(3 * (M_PI / 2.0), Vector3<float>(1.0, 1.0, 0.0)));
	GeometryPtr boxGeo3 = geometrySystem.createBox(bodyBox3, Vector3<float>(50.0f,75.0f,55.0f));
	BoxPtr box3 = boost::static_pointer_cast<Box>(boxGeo3);
			
    particleSystem.lookForCollisionsWith(box3);*/


}
TEFUNC void keyHandler(unsigned char key) {
	if ( key == 'x')
		//animate = true;
		bodySystem.getRigidBody(capsuleId)->setForce(Vec3(0.5, 0.0, 0.0));
	if ( key == 'y')
		//animate = true;
		bodySystem.getRigidBody(capsuleId)->setForce(Vec3(0.0, 0.5, 0.0));
	if ( key == 'z')
		//animate = true;
		bodySystem.getRigidBody(capsuleId)->setForce(Vec3(0.0, 0.0, 0.5));
	if ( key == 'X')
		//animate = true;
		bodySystem.getRigidBody(capsuleId)->setForce(Vec3(-0.5, 0.0, 0.0));
	if ( key == 'Y')
		//animate = true;
		bodySystem.getRigidBody(capsuleId)->setForce(Vec3(0.0, -0.5, 0.0));
	if ( key == 'Z')
		//animate = true;
		bodySystem.getRigidBody(capsuleId)->setForce(Vec3(0.0, 0.0, -0.5));
	if ( key == 't')
		//animate = true;
		bodySystem.getRigidBody(capsuleId)->setTorque(Vec3(0.0, 0.0, -5.5));
   
	if ( key == 'a'){
		animate = true;
		bodySystem.getRigidBody(capsuleId)->setForce(Vec3(0.0, 0.0, -8.5));
		}
		
		
		
}


#ifdef OS_WINDOWS
	const TestEnvironment testEnvironmentClothSimonWish(initialize,displayLoop,keyHandler);
#endif
