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

#include <simon/config.h>

#include <iostream>
#include <fstream>

// CONFIG OPTIONS

#define NUM_COLLISION_CHECKS 5.0
#define STEPS_PER_LOOP 3

// computed from the config options
#define SPEED_FACTOR STEPS_PER_LOOP/10



// Gravity. kann mit --gravity und --no-gravity explizit gesetzt werden.
bool enableGravity=true;

// Einzelschritt. kann mit --stepping und --no-stepping explizit gesetzt werden.
bool enableStepping=false;

float gravityMagnitude = 0.00981;

#define LEFT_PLANE 1
#define RIGHT_PLANE 2
#define BACK_PLANE 4
#define FRONT_PLANE 8
#define BOTTOM_PLANE 16
#define TOP_PLANE 32

// Gravity. kann mit --open und --closed explizit gesetzt werden.
int whichPlanes = LEFT_PLANE | RIGHT_PLANE | BACK_PLANE | FRONT_PLANE | BOTTOM_PLANE;

// Mögliche Werte für whichCollisionTest. 
enum{
	SPHERE_BOX_1,
	SPHERE_BOX_2,
	CAPS_CAPS_1,
	CAPS_CAPS_2,
	SPHERE_SPHERE_1,
	MANY_SPHERES,
	SPHERE_CAPS_1,
	SPHERE_CAPS_2,
	CAPS_PLANE_1,
	CAPS_SPHERE_1,
	MANY_CAPS,
	BOX_BOX_1,
	BOX_BOX_2,
	BOX_PLANE,
	CAPS_BOX,
	MANY_BOXES,
	SPHERE_PLANE};
	
// entscheidet, welcher Kollisionstest durchgeführt werden soll.
// Wird mit Programmargumenten gesetzt.
int whichCollisionTest=MANY_SPHERES;

int whichIntegrator = SimonState::INTEGRATE_RUNGEKUTTA;

// auf welchen wert soll die velocity gesetzt werden, bei tastatursteuerung ?
#define MANUAL_VELOCITY_UPDATE_STEP 0.5

#define MANUAL_FORCE_UPDATE_STEP 0.02

static SmartPointer<Geometry> movingObject;
static RigidBodySystem rigidBodySystem;
static GeometrySystem geometrySystem;

int numStepsForWatch=0;
float displayWatchTotal =0;
float collisionWatchTotal =0;
float contactWatchTotal =0;


SmartPointer<RigidBody> rigidBodyPlane1(rigidBodySystem.create(Id(Id::typePlane,0)));
SmartPointer<RigidBody> rigidBodyPlane2(rigidBodySystem.create(Id(Id::typePlane,1)));
SmartPointer<RigidBody> rigidBodyPlane3(rigidBodySystem.create(Id(Id::typePlane,2)));
SmartPointer<RigidBody> rigidBodyPlane4(rigidBodySystem.create(Id(Id::typePlane,3)));
SmartPointer<RigidBody> rigidBodyPlane5(rigidBodySystem.create(Id(Id::typePlane,4)));
SmartPointer<RigidBody> rigidBodyPlane6(rigidBodySystem.create(Id(Id::typePlane,5)));

SmartPointer<RigidBody> rigidBodySphereA(rigidBodySystem.create(Id(Id::typeSphere,5)));
SmartPointer<RigidBody> rigidBodySphereB(rigidBodySystem.create(Id(Id::typeSphere,6)));
SmartPointer<RigidBody> rigidBodySphereC(rigidBodySystem.create(Id(Id::typeSphere,7)));
SmartPointer<RigidBody> rigidBodySphereD(rigidBodySystem.create(Id(Id::typeSphere,8)));
SmartPointer<RigidBody> rigidBodySphereE(rigidBodySystem.create(Id(Id::typeSphere,9)));
SmartPointer<RigidBody> rigidBodySphereF(rigidBodySystem.create(Id(Id::typeSphere,10)));
SmartPointer<RigidBody> rigidBodySphereG(rigidBodySystem.create(Id(Id::typeSphere,11)));
SmartPointer<RigidBody> rigidBodySphereH(rigidBodySystem.create(Id(Id::typeSphere,12)));

SmartPointer<RigidBody> rigidBodyBox1(rigidBodySystem.create(Id(Id::typeBox,13)));
SmartPointer<RigidBody> rigidBodyBox2(rigidBodySystem.create(Id(Id::typeBox,14)));
SmartPointer<RigidBody> rigidBodyCaps1(rigidBodySystem.create(Id(Id::typeSphere,15)));
SmartPointer<RigidBody> rigidBodyCaps2(rigidBodySystem.create(Id(Id::typeSphere,16)));
SmartPointer<RigidBody> rigidBodyCaps3(rigidBodySystem.create(Id(Id::typeSphere,17)));
SmartPointer<RigidBody> rigidBodyCaps4(rigidBodySystem.create(Id(Id::typeSphere,18)));

using namespace std;

TEFUNC void specialKeyboard(int key, int /*x*/, int /*y*/)
{
 if (movingObject)
 {
  switch(key)
  {//Tastaturabfrage
  case GLUT_KEY_UP:
//      puts("Left\n");
    //  movingObject->setAcceleration( movingObject->getAcceleration()+Vector3<float>( -10,0,0) / movingObject->getMass());
      movingObject->addForce( Vector3<float>( 0,0,-MANUAL_FORCE_UPDATE_STEP));
      movingObject->setVelocity( Vector3<float>( 0,0,-MANUAL_VELOCITY_UPDATE_STEP));
    break;
  case GLUT_KEY_DOWN:
//      puts("Left\n");
    //  movingObject->setAcceleration( movingObject->getAcceleration()+Vector3<float>( -10,0,0) / movingObject->getMass());
      movingObject->addForce( Vector3<float>( 0,0,MANUAL_FORCE_UPDATE_STEP));
      movingObject->setVelocity( Vector3<float>( 0,0, MANUAL_VELOCITY_UPDATE_STEP));
    break;      
  case GLUT_KEY_LEFT:
//      puts("Left\n");
    //  movingObject->setAcceleration( movingObject->getAcceleration()+Vector3<float>( -10,0,0) / movingObject->getMass());
      movingObject->addForce( Vector3<float>( -MANUAL_FORCE_UPDATE_STEP,0,0));
      movingObject->setVelocity( Vector3<float>( -MANUAL_VELOCITY_UPDATE_STEP,0,0));      
    break;
  case GLUT_KEY_RIGHT:
  //    puts("Right\n");  
    //  movingObject->setAcceleration( movingObject->getAcceleration()+Vector3<float>( 10,0,0) / movingObject->getMass());
      movingObject->addForce( Vector3<float>( MANUAL_FORCE_UPDATE_STEP,0,0));
      movingObject->setVelocity( Vector3<float>( MANUAL_VELOCITY_UPDATE_STEP,0,0));            
    break;  
  case GLUT_KEY_HOME:
 //     puts("up\n");  
  //    movingObject->setAcceleration( movingObject->getAcceleration()+Vector3<float>( 0,10,0) / movingObject->getMass());
      movingObject->addForce( Vector3<float>( 0,MANUAL_FORCE_UPDATE_STEP,0));
      movingObject->setVelocity( Vector3<float>( 0,MANUAL_VELOCITY_UPDATE_STEP,0));            
    break;
  case GLUT_KEY_END:
  //    puts("down\n");  
//      movingObject->setAcceleration( movingObject->getAcceleration()+Vector3<float>( 0,-10,0) / movingObject->getMass());
      movingObject->addForce( Vector3<float>( 0,-MANUAL_FORCE_UPDATE_STEP,0));
      movingObject->setVelocity( Vector3<float>( 0,-MANUAL_VELOCITY_UPDATE_STEP,0));       
    break;  
  case GLUT_KEY_PAGE_DOWN:
  //    puts("down\n");  
//      movingObject->setAcceleration( movingObject->getAcceleration()+Vector3<float>( 0,-10,0) / movingObject->getMass());
	for (GeometrySystem::vIterator it = geometrySystem.begin(); it != geometrySystem.end(); it++)
	{					
		if ((*it)==movingObject)
		{
			if (++it==geometrySystem.end()) it=geometrySystem.begin();
			movingObject = (*it);
			return;
		}
	}
	movingObject = *(geometrySystem.begin());
    break;      
  case GLUT_KEY_PAGE_UP:
  //    puts("down\n");  
//      movingObject->setAcceleration( movingObject->getAcceleration()+Vector3<float>( 0,-10,0) / movingObject->getMass());
	for (GeometrySystem::vIterator it = geometrySystem.end()-1; it != geometrySystem.begin(); it--)
	{					
		if ((*it)==movingObject)
		{
			if (it==geometrySystem.begin())
			{
				it=geometrySystem.end()-1;
			}
			else
			{
				--it;
			}
			movingObject = (*it);
			return;
		}
	}
	movingObject = *(geometrySystem.end()-1);	
    break;      
    
  }
 }
}



/**
 * \brief Loop-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was fÃ¼r die einzelnen
 * Tests nÃ¶tig ist.
 */
TEFUNC void displayLoop() {

	for (int i = 0; i < STEPS_PER_LOOP; i++){

		//! gravitation aufrechnen
		if (enableGravity) rigidBodySystem.addGravity();
		
		for (GeometrySystem::vIterator it = geometrySystem.begin(); it != geometrySystem.end(); it++)
		{					
			assert((*it));
			(*it)->setColor(Graphics::grey);
			if ((*it)==movingObject) movingObject->setColor(Graphics::green);
			(*it)->draw();
		}

		numStepsForWatch++;  
		Clock displayWatch;
		displayWatch.start();  

		Clock collisionWatch;
		collisionWatch.start();        
		for (int i=0; i<NUM_COLLISION_CHECKS;i++)
		{
			geometrySystem.resolveCollisions((30)*SPEED_FACTOR);
		}		

		collisionWatch.stop();
		collisionWatchTotal+=collisionWatch.getDuration();

		if (enableStepping)
		{
			for (GeometrySystem::vIterator it = geometrySystem.begin(); it != geometrySystem.end(); it++)
			{
				if ((*it)->getInvMass()!=0)
				{				
					(*it)->getRigidBody()->undoSetVelocity();
					(*it)->getRigidBody()->undoSetAngularVelocity();
				}
			}
			
			rigidBodySystem.integrateVelocities((30)*SPEED_FACTOR);
			rigidBodySystem.integratePositions((30)*SPEED_FACTOR);
			for (GeometrySystem::vIterator it = geometrySystem.begin(); it != geometrySystem.end(); it++)
			{
				if ((*it)->getInvMass()!=0)
				{
					(*it)->setColor(Graphics::red);       	
					(*it)->draw();
		       	
					(*it)->getRigidBody()->undoSetPosition();
					(*it)->getRigidBody()->undoSetOrientation();
					(*it)->getRigidBody()->undoSetVelocity();
					(*it)->getRigidBody()->undoSetAngularVelocity();
				}
				//		(*it)->undoSetVelocity();    
			}    			
		}
	
		rigidBodySystem.integrateVelocities((30)*SPEED_FACTOR);

		Clock resolveContactsWatch;
		resolveContactsWatch.start();
		geometrySystem.resolveContactsFast((30)*SPEED_FACTOR, 2);

		resolveContactsWatch.stop();
		contactWatchTotal +=resolveContactsWatch.getDuration();
        
		rigidBodySystem.integratePositions((30)*SPEED_FACTOR);

		if (enableStepping)
		{
			for (GeometrySystem::vIterator it = geometrySystem.begin(); it != geometrySystem.end(); it++)
			{
				if ((*it)->getInvMass()!=0)
				{
					(*it)->setColor(Graphics::yellow);       	
					(*it)->draw();
	
	       	
					(*it)->getRigidBody()->undoSetPosition();
					(*it)->getRigidBody()->undoSetOrientation();
					(*it)->getRigidBody()->undoSetVelocity();
					(*it)->getRigidBody()->undoSetAngularVelocity();
				}
				//		(*it)->undoSetVelocity();    
			}    
		}

		displayWatch.stop();
		displayWatchTotal +=displayWatch.getDuration();
     
		glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, Graphics::magenta );
		Graphics::drawLine(Vector3<float>(0,0,0), Vector3<float>(30,0,0));
		Graphics::drawLine(Vector3<float>(0,-20.25020,0), Vector3<float>(0,30,0));
		Graphics::drawLine(Vector3<float>(0,0,0), Vector3<float>(0,0,30));

	}
}

/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung 
 * aufgerufen. Hier sollte alles reingeschrieben werden, 
 * was fÃ¼r die initialisierung der einzelnen Tests nÃ¶tig ist.
 */
TEFUNC void initialize(int argc, char** argv)
{
  	cout << "Argumente :\n";
  	cout << " --help : Nur diese Hilfe zeigen\n";
  	cout << " --euler : Integrator auf Euler setzen\n";
  	cout << " --rungekutta : Integrator auf RungeKutta setzen\n";  	      
  	cout << " --gravity<n>: Gravity setzen\n";
  	cout << " 				n ist die absolute Stärke der Gravity. zB. 0.00981\n";
  	cout << " --no-gravity<n>: alias für --gravity0\n";
  	cout << " --stepping --no-stepping : Einzelschritt an/ausschalten\n";
  	cout << " --planes<n> : Welche Planes sollen aktiv sein ?\n";	  	  	
  	cout << " 				Bitweise oder aus folgenden Werten :\n";
  	cout << "  				  1 = LEFT_PLANE\n";
  	cout << " 				  2 = RIGHT_PLANE\n";
  	cout << " 				  4 = BACK_PLANE\n";
  	cout << " 				  8 = FRONT_PLANE\n";
  	cout << " 				  16= BOTTOM_PLANE\n";
  	cout << " 				  32= TOP_PLANE\n";
  	cout << "				z.B. 31 = alle außer top-plane";
  	cout << " dazu einen der folgenden Testfälle:\n";	
  	cout << " --sphere-box1\n";  	  	
  	cout << " --sphere-box2\n";  	  	  	
  	cout << " --sphere-caps1\n";  	  	  	
  	cout << " --sphere-caps2\n";
  	cout << " --sphere-sphere1\n";   	
  	cout << " --sphere-plane\n";
  	cout << " --many-spheres\n";
  	cout << " --sphere-sphere1\n";
  	cout << " --caps-caps1\n";
  	cout << " --caps-caps2\n";
  	cout << " --caps-plane\n";   	  	  	
  	cout << " --caps-sphere\n";   	  	  	  	
  	cout << " --many-caps\n";   	  	  	  	  	
  	cout << " --box-box1\n";   	  	  	  	  	  	
  	cout << " --box-box2\n";
  	cout << " --box-plane\n";   	  	  	  	  	  	  	
  	cout << " --caps-box\n";  
	cout << " --many-boxes\n";  
  	cout << "Zusätzliche Tasten im Programm :\n";
  	cout << " Bild_auf / Bild_ab : über alle Objekte selektieren\n";
  	cout << " Pos1 / Ende : Selektiertes(grünes) Objekt hoch/runter bewegen\n";   
  	cout << " Pfeiltasten : Selektiertes(grünes) Objekt in x-z Richtung bewegen\n";
  	
	
	for (int i=1; i<argc; i++)
	{
		if (strcmp(argv[i], "--euler")==0)
		{
			whichIntegrator = SimonState::INTEGRATE_EULER;
            continue;
		}
		if (strcmp(argv[i], "--rungekutta")==0)
		{
			whichIntegrator = SimonState::INTEGRATE_RUNGEKUTTA;
            continue;      
		}
    
		if (strcmp(argv[i], "--help")==0)
		{
			exit(0);
		}
		if (strncmp(argv[i], "--planes", 8)==0)
		{
			char planeString[80];
			strcpy(planeString , argv[i]+8);
			whichPlanes = atoi(planeString);
			continue;
		}
		
		if (strncmp(argv[i], "--gravity", 8)==0)
		{
			enableGravity = true;
            //wenn kein parameter std. gravity
            if (strlen(argv[i])<10) continue;
			char gravityString[80];
			strcpy(gravityString, argv[i]+9);
			gravityMagnitude = atof(gravityString);
			continue;
		}
		if (strcmp(argv[i], "--no-gravity")==0)
		{
			enableGravity = false;
			continue;
		}
		if (strcmp(argv[i], "--stepping")==0)
		{
			enableStepping = true;
			continue;
		}
		if (strcmp(argv[i], "--no-stepping")==0)
		{
			enableStepping = false;
			continue;
		}
		if (strcmp(argv[i], "--sphere-box1")==0)
		{
			whichCollisionTest = SPHERE_BOX_1;
			continue;
		}
		if (strcmp(argv[i], "--sphere-box2")==0)
		{
			whichCollisionTest = SPHERE_BOX_2;
			continue;
		}    
		if (strcmp(argv[i], "--sphere-caps1")==0)
		{
			whichCollisionTest = SPHERE_CAPS_1;
			continue;
		}    
		if (strcmp(argv[i], "--sphere-caps2")==0)
		{
			whichCollisionTest = SPHERE_CAPS_2;
			continue;
		}    
		if (strcmp(argv[i], "--sphere-sphere1")==0)
		{
			whichCollisionTest = SPHERE_SPHERE_1;
			continue;
		}       
		if (strcmp(argv[i], "--many-spheres")==0)
		{
			whichCollisionTest = MANY_SPHERES;
			continue;
		}
		
		if (strcmp(argv[i], "--caps-box")==0)
		{
			whichCollisionTest = CAPS_BOX;
			continue;
		}		
		if (strcmp(argv[i], "--caps-caps1")==0)
		{
			whichCollisionTest = CAPS_CAPS_1;
			continue;
		}
		if (strcmp(argv[i], "--caps-caps2")==0)
		{
			whichCollisionTest = CAPS_CAPS_2;
			continue;
		}        
		if (strcmp(argv[i], "--caps-plane")==0)
		{
			whichCollisionTest = CAPS_PLANE_1;
			continue;
		}        
		if (strcmp(argv[i], "--caps-sphere")==0)
		{
			whichCollisionTest = CAPS_SPHERE_1;
			continue;
		}        
		if (strcmp(argv[i], "--many-caps")==0)
		{
			whichCollisionTest = MANY_CAPS;
			continue;
		}        
		if (strcmp(argv[i], "--box-box1")==0)
		{
			whichCollisionTest = BOX_BOX_1;
			continue;
		}        
		if (strcmp(argv[i], "--box-box2")==0)
		{
			whichCollisionTest = BOX_BOX_2;
			continue;
		}        
		if (strcmp(argv[i], "--box-plane")==0)
		{
			whichCollisionTest = BOX_PLANE;
			continue;
		}        
		if (strcmp(argv[i], "--many-boxes")==0)
		{
			whichCollisionTest = MANY_BOXES;
			continue;
		}        
		if (strcmp(argv[i], "--sphere-plane")==0)
		{
			whichCollisionTest = SPHERE_PLANE;
			continue;
		}       
		cout << "Ignoriere unbekannten Parameter : " << argv[i] << "\n";
	}
  
//gravity vector
	SimonState::exemplar()->setGravityVector(Vector3<float>(0,-gravityMagnitude,0));
	SimonState::exemplar()->setIntegrator(whichIntegrator);  

//geometrySystem.testTransitiveClosure();

//	rigidBodySystem.disableViscosity();
//SimonState::exemplar()->setGravityVector(Vector3<float>(0,-0.004,0));
	ofstream out("log.txt");
	clog.rdbuf(out.rdbuf());

	glutSpecialFunc(specialKeyboard);//Keyboard Funktion


//planes fÃ¼r testbox

	if (whichPlanes & LEFT_PLANE)
	{
		SmartPointer<Geometry> leftPlane = geometrySystem.createPlane(rigidBodyPlane1);
		leftPlane->setVisibilityState(true);
		leftPlane->setPosition( Vector3<float>(-300,0,0));
		leftPlane->setOrientation(Quaternion (-M_PI/2,Vector3<float>(0,0,1))); 
		leftPlane->getRigidBody()->setIsDynamicFlag(false);
		leftPlane->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		leftPlane->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		leftPlane->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
		leftPlane->setInvMass(0);
	}

	if (whichPlanes & RIGHT_PLANE)
	{
		SmartPointer<Geometry> rightPlane = geometrySystem.createPlane(rigidBodyPlane2);
		rightPlane->setVisibilityState(true);
		rightPlane->setPosition(Vector3<float>(300,0,0));
		rightPlane->setOrientation(Quaternion (M_PI/2,Vector3<float>(0,0,1)));
		rightPlane->getRigidBody()->setIsDynamicFlag(false);
		rightPlane->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		rightPlane->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		rightPlane->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
		rightPlane->setInvMass(0);
	}

	if (whichPlanes & FRONT_PLANE)
	{
		SmartPointer<Geometry> frontPlane = geometrySystem.createPlane(rigidBodyPlane4);
		frontPlane->setVisibilityState(true);
		frontPlane->setPosition( Vector3<float>(0,0,300));
		frontPlane->setOrientation(Quaternion(-M_PI/2,Vector3<float>(1,0,0)));                    
		frontPlane->getRigidBody()->setIsDynamicFlag(false);
		frontPlane->setInvMass(0);
	}	

	if (whichPlanes & BACK_PLANE)
	{
		SmartPointer<Geometry> backPlane = geometrySystem.createPlane(rigidBodyPlane5);
		backPlane->setVisibilityState(true);
		backPlane->setPosition( Vector3<float>(0,0,-300));
		backPlane->setOrientation(Quaternion(M_PI/2,Vector3<float>(1,0,0)));
		backPlane->getRigidBody()->setIsDynamicFlag(false);
		backPlane->setInvMass(0);
	}

	if (whichPlanes & BOTTOM_PLANE)
	{
		SmartPointer<Geometry> bottomPlane = geometrySystem.createPlane(rigidBodyPlane3);
		bottomPlane->setOrientation(Quaternion(0,Vector3<float>(0,1,0))); 
		bottomPlane->setPosition( Vector3<float>(0,-300,0));
		bottomPlane->setBounciness(0.9);    
		bottomPlane->setVisibilityState(true);                  
		bottomPlane->getRigidBody()->setIsDynamicFlag(false);
		bottomPlane->setInvMass(0);
	}

	if (whichPlanes & TOP_PLANE)
	{
		SmartPointer<Geometry> topPlane = geometrySystem.createPlane(rigidBodyPlane5);
		topPlane->setOrientation(Quaternion(0,Vector3<float>(0,1,0))); 
		topPlane->setPosition( Vector3<float>(0,300,0));
		topPlane->setVisibilityState(true);                  
		topPlane->getRigidBody()->setIsDynamicFlag(false);
		topPlane->setInvMass(0);
	}

	switch (whichCollisionTest)
	{
	case SPHERE_SPHERE_1:
	{
		SmartPointer<Geometry> mySphereC = geometrySystem.createSphere(rigidBodySphereC, 30);           
		mySphereC->setMass(10);
		mySphereC->setBounciness(0.5);  
		mySphereC->setVelocity(Vector3<float>(0.0,0.0,0.0));
		mySphereC->setPosition(Vector3<float>( -160.0,0.0,0.0));   
		//mySphereC->getRigidBody()->setForce( Vector3<float>(0.0,-5.0,0.0));

		SmartPointer<Geometry> mySphereD = geometrySystem.createSphere(rigidBodySphereD, 30);           
		mySphereD->setMass(10);
		mySphereD->setBounciness(0.5);  
		mySphereD->setVelocity(Vector3<float>(0.0,0.0,0.0));
		mySphereD->setPosition(Vector3<float>( -100,0.0,0.0));   
		//mySphereD->getRigidBody()->setForce( Vector3<float>(0.0,-5.0,0.0));

		SmartPointer<Geometry> mySphereE = geometrySystem.createSphere(rigidBodySphereE, 30);           
		mySphereE->setMass(10);
		mySphereE->setBounciness(0.5);  
		mySphereE->setVelocity(Vector3<float>(0.0,0.0,0.0));
		mySphereE->setPosition(Vector3<float>( -40.0,0.0,0.0));   
		//mySphereE->getRigidBody()->setForce( Vector3<float>(0.0,-5.0,0.0));

		SmartPointer<Geometry> mySphereF = geometrySystem.createSphere(rigidBodySphereF, 30);           
		mySphereF->setMass(10);
		mySphereF->setBounciness(0.5);  
		mySphereF->setVelocity(Vector3<float>(0.0,0.0,0.0));
		mySphereF->setPosition(Vector3<float>( 20.0,0.0,0.0));   
		//mySphereF->getRigidBody()->setForce( Vector3<float>(0.0,-5.0,0.0));


		SmartPointer<Geometry> mySphereG = geometrySystem.createSphere(rigidBodySphereG, 30);           
		mySphereG->setMass(10);
		mySphereG->setBounciness(0.5);  
		mySphereG->setVelocity(Vector3<float>(0.0,0.0,0.0));
		mySphereG->setPosition(Vector3<float>( 80.0,0.0,0.0));   
		//mySphereG->getRigidBody()->setForce( Vector3<float>(0.0,-5.0,0.0));


		SmartPointer<Geometry> mySphereH = geometrySystem.createSphere(rigidBodySphereH, 30);           
		mySphereH->setMass(10);
		mySphereH->setBounciness(0.95);  
		mySphereH->setVelocity(Vector3<float>(-0.08,0,0.0));
		mySphereH->setPosition(Vector3<float>( 200.0,0.0,0.0));   
		//mySphereH->getRigidBody()->setForce( Vector3<float>(0.0,-5.0,0.0));
		break;
	}
	case MANY_SPHERES:
	{
		for (int i =0; i<6; i++)
		{
			SmartPointer<RigidBody> rigidBodySphere(rigidBodySystem.create(Id(Id::typeSphere,20+i)));

			SmartPointer<Geometry> mySphere = geometrySystem.createSphere(
				rigidBodySphere,
				120);
			mySphere->setMass(3);                    
			mySphere->setBounciness(0.95);  
			mySphere->setVelocity(Vector3<float>(0.1, 0.0 ,0.3));
//  mySphere->setPosition(Vector3<float>( (rand()/(float)RAND_MAX)*200, (rand()/(float)RAND_MAX)*300, (rand()/(float)RAND_MAX)*200));
			mySphere->setPosition(Vector3<float>(i*20.0, i*300, 0.0));
                    
		}
		break;
	}
	case CAPS_CAPS_1:
	{
		SmartPointer<Geometry>	myCaps1 = geometrySystem.createCapsule(rigidBodyCaps1,30,150);           
		myCaps1->setMass(10);
		myCaps1->setBounciness(0.90);  
		myCaps1->setVelocity(Vector3<float>( 0.05,0.0,0.0));
		myCaps1->setPosition(Vector3<float>( -50.0,500.0,0.0));   
		myCaps1->setOrientation(Quaternion(0, Vector3<float>(0,0,1)));
		myCaps1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myCaps1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		myCaps1->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
					
                    
		SmartPointer<Geometry>	myCaps2 = geometrySystem.createCapsule(rigidBodyCaps2,30,50);           
		myCaps2->setMass(10);
		myCaps2->setBounciness(0.90);  
		myCaps2->setPosition(Vector3<float>( 200.0,500.0,0.0));   
		myCaps2->setOrientation(Quaternion(M_PI/3, Vector3<float>(0,0,1)));
		myCaps2->setVelocity(Vector3<float> (-0.05,0.00,0.0));
		myCaps2->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myCaps2->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		myCaps2->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
			
		break;
	}
	case CAPS_CAPS_2:
	{
		SmartPointer<Geometry>  myCaps3 = geometrySystem.createCapsule(rigidBodyCaps3,3,15);           
		myCaps3->setMass(0.1);
		myCaps3->setBounciness(0.90);  
		myCaps3->setVelocity(Vector3<float>(0.0,0.0,0.0));
		myCaps3->setPosition(Vector3<float>(0.0,0.0,0.0));   
		myCaps3->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0.001));

		SmartPointer<Geometry> myCaps4 = geometrySystem.createCapsule(rigidBodyCaps4,30,150);           
		myCaps4->setMass(0.0275);
		myCaps4->setBounciness(0.90);
		myCaps4->setPosition(Vector3<float>( 0.0,0.0,200.0));   
		//myCaps4->setOrientation(Quaternion(M_PI_2, Vector3<float>(0,0,1)));
		myCaps4->setVelocity(Vector3<float> (0.0,0.0,0.0));
		//myCaps4->getRigidBody()->setTorque(Vector3<float>(0,0,0.001));
		myCaps4->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0.0001));
		break;

	}
	case SPHERE_BOX_1:
	{
		SmartPointer<Geometry> myBox1 = geometrySystem.createBox(rigidBodyBox1, Vector3<float>(50.0, 180.0, 80.0));
		myBox1->setMass(40);
		myBox1->setBounciness(0.8);
		myBox1->setVelocity(Vector3<float>(0,0,0));
		myBox1->setPosition(Vector3<float>(0,0,-50));
		myBox1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myBox1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		myBox1->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
		//myBox1->setOriesetBouncinessntation(Quaternion(M_PI_4, Vector3<float>(1,0,0)));

		SmartPointer<Geometry> mySphereA = geometrySystem.createSphere(rigidBodySphereA, 20);
		mySphereA->setMass(10);
		mySphereA->setBounciness(0.8);  
		mySphereA->setVelocity(Vector3<float>(-0.01, 0.0, 0.0));
		mySphereA->setPosition(Vector3<float>( 100.0,100.0,0.0));   
		mySphereA->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		//mySphereA->getRigidBody()->setForce( Vector3<float>(0.0,-5.0,0.0));

/*SmartPointer<Geometry> mySphereB = geometrySystem.createSphere(rigidBodySphereB, 30);           
  mySphereB->setMass(0.001);
  mySphereB->setBounciness(1.0);  
  mySphereB->setVelocity(Vector3<float>(0.0008,0.0,0.0));
  mySphereB->setPosition(Vector3<float>( -250.0,50.0,0.0));   
  //mySphereB->getRigidBody()->setForce( Vector3<float>(0.0,-5.0,0.0)); 
  */
		break;
	}
	case SPHERE_BOX_2:
	{
		SmartPointer<Geometry> myBox2 = geometrySystem.createBox(rigidBodyBox2, Vector3<float>(100.0, 150.0, 80.0));
		myBox2->setMass(10);
		myBox2->setBounciness(1.0);
		myBox2->setVelocity(Vector3<float>(0,0,0));
		myBox2->setPosition(Vector3<float>(0,0,0));
		myBox2->setOrientation(Quaternion(M_PI/4, Vector3<float>(1,0,0)));


		SmartPointer<Geometry> mySphereB = geometrySystem.createSphere(rigidBodySphereB, 30);           
		mySphereB->setMass(10);
		mySphereB->setBounciness(1.0);  
		mySphereB->setVelocity(Vector3<float>(0.0,-0.02,0.0));
		mySphereB->setPosition(Vector3<float>( 0.0,300.0,0.0));   
		//mySphereB->getRigidBody()->setForce( Vect-or3<float>(0.0,-5.0,0.0)); 
		break;
	}
	case CAPS_PLANE_1:
	{
		SmartPointer<Geometry> myCaps1 = geometrySystem.createCapsule(rigidBodyCaps1,30,150);           
		myCaps1->setMass(10);
		myCaps1->setBounciness(0.90);  
		myCaps1->setVelocity(Vector3<float>( -0.20,0.0,0.0));
		myCaps1->setPosition(Vector3<float>( 0,0.0,0.0));   
		myCaps1->setOrientation(Quaternion(M_PI/4.0, Vector3<float>(0,0,1)));
		myCaps1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myCaps1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		myCaps1->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
		break;
	}
	case CAPS_SPHERE_1:
	{
		SmartPointer<Geometry> mySphereA = geometrySystem.createSphere(rigidBodySphereA, 50);
		mySphereA->setMass(10);
		mySphereA->setBounciness(0.8);  
		mySphereA->setVelocity(Vector3<float>(-0.09, 0.0, 0.0));
		mySphereA->setPosition(Vector3<float>( 100.0,50.0,0.0));   
		mySphereA->getRigidBody()->setTorque(Vector3<float>(0,0,0));

		SmartPointer<Geometry>myCaps1 = geometrySystem.createCapsule(rigidBodyCaps1,30,150);
		myCaps1->setMass(10);
		myCaps1->setBounciness(0.90);  
		myCaps1->setVelocity(Vector3<float>( 0.09,0.0,0.0));
		myCaps1->setPosition(Vector3<float>( -50,0.0,0.0));   
		myCaps1->setOrientation(Quaternion(0, Vector3<float>(0,0,1)));
		myCaps1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myCaps1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		myCaps1->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));

		/*SmartPointer<Geometry>myCaps2 = geometrySystem.createCapsule(rigidBodyCaps2,30,150);
		  myCaps2->setMass(10);
		  myCaps2->setBounciness(0.90);  
		  myCaps2->setVelocity(Vector3<float>( 0.09,0.0,0.0));
		  myCaps2->setPosition(Vector3<float>( -200,0.0,0.0));   
		  myCaps2->setOrientation(Quaternion(M_PI/-3.0, Vector3<float>(0,0,1)));
		  myCaps2->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		  myCaps2->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		  myCaps2->getRigidBody()->setAngularVelocity(Vector3<float>(0.001,0,0));
		*/

		break;
	}
	case BOX_BOX_1:
	{
		SmartPointer<Geometry> myBox1 = geometrySystem.createBox(rigidBodyBox1, Vector3<float>(100.0, 100.0, 100.0));
		myBox1->setMass(40);
		myBox1->setBounciness(0.8);
		myBox1->setVelocity(Vector3<float>(0,0,0));
		myBox1->setPosition(Vector3<float>(100,86,30));
		myBox1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myBox1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		//myBox1->getRigidBody()->setAngularVelocity(Vector3<float>(1,1,0));
		myBox1->getRigidBody()->setOrientation(Quaternion(M_PI/4, Vector3<float>(1.0, 0.0, 0.0)));					

		SmartPointer<Geometry> myBox2 = geometrySystem.createBox(rigidBodyBox2, Vector3<float>(50.0, 50.0, 50.0));
		myBox2->setMass(100);
		myBox2->setBounciness(0.8);
		myBox2->setVelocity(Vector3<float>(0.3,0,0));
		myBox2->setPosition(Vector3<float>(-300,60,20));
		myBox2->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myBox2->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		myBox2->getRigidBody()->setOrientation(Quaternion(M_PI/4, Vector3<float>(0.0, 0.0, 1.0)));
		//myBox2->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
		break;		
	}
	case BOX_BOX_2: {
		SmartPointer<Geometry> myBox1 = geometrySystem.createBox(rigidBodyBox1, Vector3<float>(100, 200.0, 130.0));
		myBox1->setMass(40);
		myBox1->setBounciness(0.8);
		myBox1->setVelocity(Vector3<float>(0,0,0));
		myBox1->setPosition(Vector3<float>(200,10,30));
		myBox1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myBox1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		//myBox1->getRigidBody()->setAngularVelocity(Vector3<float>(1,1,0));
		//myBox1->getRigidBody()->setOrientation(Quaternion(M_PI/4, Vector3<float>(1.0, 0.0, 0.0)));					

		SmartPointer<Geometry> myBox2 = geometrySystem.createBox(rigidBodyBox2, Vector3<float>(70.0, 110.0, 80.0));
		myBox2->setMass(100);
		myBox2->setBounciness(0.8);
		myBox2->setVelocity(Vector3<float>(0.05,0,0));
		//myBox2->setPosition(Vector3<float>(-300,250,20));
		myBox2->setPosition(Vector3<float>(-115,260,20));
		myBox2->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myBox2->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		myBox2->getRigidBody()->setOrientation(Quaternion(M_PI/4, Vector3<float>(1.0, 0.0, 0.0)));
		//myBox2->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
				
		break;
	}
	
	
	case CAPS_BOX:
	{
		SmartPointer<Geometry>	myCaps1 = geometrySystem.createCapsule(rigidBodyCaps1,30,150);           
		myCaps1->setMass(10);
		myCaps1->setBounciness(0.90);  
		myCaps1->setVelocity(Vector3<float>( 0.05,0.0,0.0));
		myCaps1->setPosition(Vector3<float>( -50.0, 100.0, 0.0));   
		myCaps1->setOrientation(Quaternion(0, Vector3<float>(0,0,1)));
		myCaps1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myCaps1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		myCaps1->getRigidBody()->setAngularVelocity(Vector3<float>(0,0,0));
		
		SmartPointer<Geometry> myBox1 = geometrySystem.createBox(rigidBodyBox1, Vector3<float>(100, 100.0, 130.0));
		myBox1->setMass(40);
		myBox1->setBounciness(0.8);
		myBox1->setVelocity(Vector3<float>(0,0,0));
		myBox1->setPosition(Vector3<float>(200,10,30));
		myBox1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myBox1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		//myBox1->getRigidBody()->setAngularVelocity(Vector3<float>(1,1,0));
		//myBox1->getRigidBody()->setOrientation(Quaternion(M_PI/4, Vector3<float>(1.0, 0.0, 0.0)));
					
                    
		
			
		break;
	}
	case BOX_PLANE:
	{
		SmartPointer<Geometry> myBox1 = geometrySystem.createBox(rigidBodyBox1, Vector3<float>(100.0, 100.0, 100.0));
		myBox1->setMass(40);
		myBox1->setBounciness(0.8);
		myBox1->setVelocity(Vector3<float>(0,0,0));
		myBox1->setPosition(Vector3<float>(0,0,0));
		myBox1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myBox1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		//myBox1->getRigidBody()->setAngularVelocity(Vector3<float>(1,0,0));				
		myBox1->getRigidBody()->setOrientation(Quaternion(M_PI/5, Vector3<float>(1.0, 1.0, 1.0)));	
		myBox1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		myBox1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));
		//myBox1->getRigidBody()->setAngularVelocity(Vector3<float>(1,0,0));				
		myBox1->getRigidBody()->setOrientation(Quaternion(M_PI/4, Vector3<float>(1.0, 1.0, 1.0)));	
	
		break;
	}
	case MANY_BOXES:
	{
		int numOfCaps =5;
		for (int i = 0; i < numOfCaps; i++)
		{
			SmartPointer<RigidBody> body(rigidBodySystem.create(Id(Id::typeBox,20+i)));

			SmartPointer<Geometry> myBox1 = geometrySystem.createBox(body, Vector3<float>(80.0+(numOfCaps-i)*50, 30.0+(numOfCaps-i)*10, 30.0+(numOfCaps-i)*50));
			myBox1->setMass(40);
			myBox1->setBounciness(0.4);
			myBox1->setVelocity(Vector3<float>( 0.0,0.0,0.0));
			myBox1->setPosition(Vector3<float>( 0,i*162-60,0.0)); 
/*
  myBox1->setOrientation(Quaternion(
  ((M_PI) * (((float) i + 1.0) / numOfCaps )),
  Vec3(1,0,0) ));*/
		}

		break;       
	}
	case MANY_CAPS:
	{
		int numOfCaps = 30;
		for (int i = 0; i < numOfCaps; i++)
		{
			SmartPointer<RigidBody> body(rigidBodySystem.create(Id(Id::typeCapsule,20+i)));

			SmartPointer<Geometry> myCaps1 = geometrySystem.createCapsule(body,100,150);
			myCaps1->setMass(20);
			myCaps1->setBounciness(0.8);  
			myCaps1->setVelocity(Vector3<float>( -0.10,0.0,0.0));
			myCaps1->setPosition(Vector3<float>( 0,i*120,0.0)); 
		
			myCaps1->setOrientation(Quaternion(
										((M_PI) * (((float) i + 1.0) / numOfCaps )),
										Vec3(1,0,0) ));
	
		}
/*
  {
  SmartPointer<RigidBody> body(rigidBodySystem.create(Id(Id::typeCapsule,1001)));
  SmartPointer<Geometry> myCaps1 = geometrySystem.createCapsule(body,50,50);           
  myCaps1->setMass(3);
  myCaps1->setBounciness(0.90);  
  myCaps1->setVelocity(Vector3<float>( -0.10,0.0,0.0));
  myCaps1->setPosition(Vector3<float>( 0.0,75,0.0));   
  }
	
  SmartPointer<Geometry> myBox1 = geometrySystem.createBox(rigidBodyBox1, Vector3<float>(100.0, 10.0, 100.0));
  myBox1->setMass(40);
  myBox1->setBounciness(0.8);
  myBox1->setVelocity(Vector3<float>(0,0,0));
  myBox1->setPosition(Vector3<float>(0,100,0));
  myBox1->getRigidBody()->setTorque(Vector3<float>(0,0,0));
  myBox1->getRigidBody()->setAcceleration(Vector3<float>(0,0,0));	
  {	
  SmartPointer<RigidBody> body(rigidBodySystem.create(Id(Id::typeCapsule,1002)));
  SmartPointer<Geometry> myCaps1 = geometrySystem.createCapsule(body,50,50);           
  myCaps1->setMass(3);
  myCaps1->setBounciness(0.90);  
  myCaps1->setVelocity(Vector3<float>( -0.10,0.0,0.0));
  myCaps1->setPosition(Vector3<float>( 0.0,165,0.0));   
  }*/
				
		/*
		  myCaps1->setOrientation(Quaternion(
		  ((M_PI) * (((float) numOfCaps + 1.0) / numOfCaps )),
		  Vec3(0,0,1) ));		
		*/

		break;
	}   
	case SPHERE_PLANE:
	{
		SmartPointer<Geometry> mySphereA = geometrySystem.createSphere(rigidBodySphereA, 50);
		mySphereA->setMass(10);
		mySphereA->setBounciness(0.9);
		mySphereA->setVelocity(Vector3<float>(-0.09, 0.0, 0.0));
		mySphereA->setPosition(Vector3<float>( 100.0,50.0,-50.0));   
		mySphereA->getRigidBody()->setTorque(Vector3<float>(0,0,0));
	
		SmartPointer<Geometry> mySphereB = geometrySystem.createSphere(rigidBodySphereB, 50);
		mySphereB->setMass(10);
		mySphereB->setBounciness(0.9);
		mySphereB->setVelocity(Vector3<float>(-0.09, 0.0, 0.0));
		mySphereB->setPosition(Vector3<float>( 170.0,150.0,-50.0));   
		mySphereB->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		/*
		  SmartPointer<Geometry> mySphereC = geometrySystem.createSphere(rigidBodySphereC, 50);
		  mySphereC->setMass(10);
		  mySphereC->setBounciness(0.8);
		  mySphereC->setVelocity(Vector3<float>(-0.09, 0.0, 0.0));
		  mySphereC->setPosition(Vector3<float>( 100.0,50.0,50.0));   
		  mySphereC->getRigidBody()->setTorque(Vector3<float>(0,0,0));
	
		  SmartPointer<Geometry> mySphereD = geometrySystem.createSphere(rigidBodySphereD, 50);
		  mySphereD->setMass(10);
		  mySphereD->setBounciness(0.8);
		  mySphereD->setVelocity(Vector3<float>(-0.09, 0.0, 0.0));
		  mySphereD->setPosition(Vector3<float>( 170.0,150.0,50.0));   
		  mySphereD->getRigidBody()->setTorque(Vector3<float>(0,0,0));
		*/
		/* 
		   float friction = 0.1;
	
		   mySphereA->setFriction(friction);
		   mySphereB->setFriction(friction);
		   mySphereC->setFriction(friction);
		   mySphereD->setFriction(friction);
		   bottomPlane->setFriction(friction);
		   frontPlane->setFriction(friction);
		   backPlane->setFriction(friction);
		   rightPlane->setFriction(friction);
		   leftPlane->setFriction(friction);
		*/
		break;
	}
	} // switch (whichCollisionTest)

	movingObject = *(geometrySystem.end()-1);

}

TEFUNC void keyHandler(unsigned char key)
{
	if (key=='D') SimonState::exemplar()->clearMessages();
	if (key=='t')
    {
        cout << "Durschnittswerte:\nDisplayloop : " << displayWatchTotal/numStepsForWatch<<"\n";
        cout << "Collision : " << collisionWatchTotal/numStepsForWatch<<"\n";
        cout << "Contacts : " << contactWatchTotal/numStepsForWatch<<"\n";
		
		// reset all values
		numStepsForWatch = displayWatchTotal = collisionWatchTotal = contactWatchTotal = 0;
    }
}


#ifdef OS_WINDOWS
const TestEnvironment testEnvironmentCollision(initialize,displayLoop,keyHandler);
#endif

