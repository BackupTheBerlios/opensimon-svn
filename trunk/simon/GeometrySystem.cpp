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
 * \file CollisionSystem.cpp
 * \class CollisionSystem
 *
 * \brief Steuerklasse zur Kollisionserkennung
 *
 */
//------------------------------------------------------------------------------
#include <simon/GeometrySystem.h>
#include <simon/SimonState.h>
#include <simon/Id.h>
#include <simon/Clock.h>
#include <simon/ContactInformationContainer.h>

#include <simon/AnalyticCollisionDetection.h>

#ifdef DHAVE_SWIFT
#include <simon/SwiftCollisionDetection.h>
#endif

#include <simon/config.h>

using namespace std;
using namespace boost;

// iterate over the contact graph a number of times. The last round is for shock propagation (freezing objekts layer-by layer), never <2 !!!
#define CONTACT_PROCESSING_PER_GRAPH 6
// iterate over every component(=layer) a number of times
#define CONTACT_PROCESSING_PER_LEVEL 10

#define DEBUG_LEVEL_PRINT_GRAPH 0
#define DEBUG_LEVEL_PRINT_COLLISIONS 0

// Graphik-Zugriff nur f�r Testzwecke. Im Multi-Thread-Betrieb nicht erlaubt.
//#define TEST_ENVIRONMENT


GeometrySystem::GeometrySystem(){
#ifdef DHAVE_SWIFT
	mCollisionDetection = new SwiftCollisionDetection;
#endif
#ifndef DHAVE_SWIFT
	mCollisionDetection = new AnalyticCollisionDetection;
#endif
}

GeometrySystem::~GeometrySystem(){
	delete mCollisionDetection;
}

    	
/**
* \brief Startet die Kollisionserkennung Ruft die findInterference()
* Funktionen f�r alle Objekte gegeneinander auf, speichert die
* Interferences im Member vInterferences und behandelt sie durch
* Aufruf von collisionResponse.  Es werden fedkiw erst alle objekte
* verschoben, im gegensatz zu resolveContacts
*/
void GeometrySystem::resolveCollisions(float interval){

	int numOfObjects = vCollisionObjects.size();

	// if there are no elements to check collisions for -> return
	if (numOfObjects == 0) 
		return;

	bool useViscosity 
		= SimonState::exemplar()->getViscosityFlag();
	float viscositySlowdownLinear 
		= SimonState::exemplar()->getViscositySlowdownLinear();
	float viscositySlowdownAngular 
		= SimonState::exemplar()->getViscositySlowdownAngular();

	// objektpositionen vorberechnen.
	for (int i = 0; i < numOfObjects; ++i){

		RigidBodyPtr rb = vCollisionObjects[i]->getRigidBody();

		//! Ueberspringe das Objekt obj1, wenn die inverse masse 0 ist. 
		if (rb->getInvMass() == 0) 
			continue;	
		
		rb->backupVelocities();
		rb->backupPositions();
			
        rb->integrate(interval, useViscosity, 
					  viscositySlowdownLinear, 
					  viscositySlowdownAngular, 
					  false);

		rb->restoreVelocities();
	}	
	
	// clear the Interferences
	mInterferences.clear();
	// find interferences between all geometries
	mCollisionDetection->findInterferences(mInterferences);
	
	// objektpositionen zurücksetzten. WICHTIG !
	for (int i = 0; i < numOfObjects; ++i){

		RigidBodyPtr rb = vCollisionObjects[i]->getRigidBody();
	
		//! Ueberspringe das Objekt obj1, wenn die inverse masse 0 ist. 
		if (rb->getInvMass() == 0) 
			continue;	
			
		rb->restorePositions();
	}

	int numOfInterferences = mInterferences.size();

    //! \todo as to Fedkiw, Bridson, Guendelman we should process deepest interpenentration first
    // use a map to sort???
	for (int i = 0; i < numOfInterferences; ++i){
 
        bool forCollision = true;
		collisionResponse(*mInterferences[i], forCollision);

   	    sendContactInformation(*mInterferences[i]);
	}
	
}


/** 
 * Implements section 6 and following sections from Guendelman,
 * Bridson and Fedkiw "Nonconvex Rigid Bodies with Stacking", SIGGRAPH
 * 2003.
 *
 * We account for collisions without friction and collisions with
 * static and kinetic (sliding) friction, but not for rolling and
 * spinning friction.
 *
 * \author Nils Hornung
 */
void GeometrySystem::collisionResponse(GeometryPtr objectA, 
									   GeometryPtr objectB, 
									   ContactPoint &contactPoint, 
									   bool forCollision, 
									   float contactRestitution) {

	float invMassA = objectA->getRigidBody()->getInvMass();
	float invMassB = objectB->getRigidBody()->getInvMass();

 	if (invMassA == 0 && invMassB == 0)
 		return;
	
	// initializations
	//
	// velocityA, angularVelocityA, velocityB, angularVelocityB,
	// invWorldInertiaTensorA, invWorldInertiaTensorB, restitutionA,
	// restitutionB, frictionA, frictionB

	RigidBodyPtr rbA = objectA->getRigidBody();
	RigidBodyPtr rbB = objectB->getRigidBody();

	Vec3 velocityA = rbA->getVelocity();
	Vec3 velocityB = rbB->getVelocity();
	Vec3 angularVelocityA = rbA->getAngularVelocity ();
	Vec3 angularVelocityB = rbB->getAngularVelocity ();
	Matrix3x3 invWorldInertiaTensorA = rbA->getInvWorldInertiaTensor ();
	Matrix3x3 invWorldInertiaTensorB = rbB->getInvWorldInertiaTensor ();
	float restitutionA = objectA->getBounciness ();
	float restitutionB = objectB->getBounciness ();
	float frictionA = objectA->getFriction ();
	float frictionB = objectB->getFriction ();

	// radius vectors of the collision points in world coordinates
	const Vec3 & radiusA = contactPoint.getRA ();
	const Vec3 & radiusB = contactPoint.getRB ();


	// fuer was ist dass???
    Quaternion invOriA = rbA->getOrientation();
    invOriA.invert();
    Vec3 localA = qRotate( radiusA, invOriA);
    contactPoint.setContactPointInA(localA);
    
    Quaternion invOriB = rbB->getOrientation();
    invOriB.invert();
    Vec3 localB = qRotate( radiusB, invOriB);
    contactPoint.setContactPointInB(localB);    
    
        

	// coefficient of restitution (use minimum of the two coefficients)
	float restitution = min (restitutionA, restitutionB);

	// original relative velocity at the point of collision
	Vec3 relativeVelocity = velocityA + cross (angularVelocityA, radiusA) -
		(velocityB + cross (angularVelocityB, radiusB));

	// normal direction of collision
	Vec3  normal = contactPoint.getNormalB ();

	//! \todo is normal already unit length?
	float magnitude3;
	if ((magnitude3 = normal.length ()) != 0.0f)
		normal = (1.0f / magnitude3) * normal;

	// scalar normal component of relative velocity
	float normalRelativeVelocity = dot (relativeVelocity, normal);


	// only process collisions, do not process contact
	if (forCollision)
		if (abs (normalRelativeVelocity) < 0.0000001)
			return;

	// only process contact. do not process collisions
	if (!forCollision) {
		if (normalRelativeVelocity < -0.0000001)
			return;
		else {
			//cout << "!forCollision\n";
			restitution = contactRestitution;
		}
	}

	// see the original implementiation
	//if (normalRelativeVelocity < 0.000003)
	//  return;

	// vector tangential component of relative velocity
	// assume that we are in the static friction case
	Vec3 tangentRelativeVelocity; // initialized with zeros
	//Vec3 tangentRelativeVelocity = relativeVelocity - normalRelativeVelocity *
	//  normal;

	// the 'K' values and their sum
	Matrix3x3 KA = Matrix3x3::createDiagonal (invMassA) + Matrix3x1
		(radiusA).star ().T () * invWorldInertiaTensorA * Matrix3x1 (radiusA).star
		();
	Matrix3x3 KB = Matrix3x3::createDiagonal (invMassB) + Matrix3x1
		(radiusB).star ().T () * invWorldInertiaTensorB * Matrix3x1 (radiusB).star
		();
	Matrix3x3 KT = KA + KB;

	// coefficient of friction (use maximum of the two coefficients)
	float friction = max (frictionA, frictionB);

	// impulse for the static friction case
	Vec3 impulse = choleskyInverse (KT) * (- restitution * normalRelativeVelocity
										   * normal - relativeVelocity);
  
	// the impulse magnitude is the negative relative velocity and describes the
	// "strength" of the collision
	contactPoint.setImpulseMagnitude (-normalRelativeVelocity);

	// the grind magnitude describes the "strength" of "sliding".
	//contactPoint.setGrindMagnitude (0.0f);

	// decide if it is the static friction case
	// i.e. the impulse is in the friction cone
	if ((impulse - (dot (impulse, normal)) * normal).length () <= 
		friction * (dot(impulse, normal))) {

		// set impulse for static friction    
		velocityA += invMassA * impulse;
		velocityB -= invMassB * impulse;
		angularVelocityA += invWorldInertiaTensorA * (cross (radiusA, impulse));
		angularVelocityB -= invWorldInertiaTensorB * (cross (radiusB, impulse));
		rbA->setVelocity (velocityA);
		rbB->setVelocity (velocityB);
		rbA->setAngularVelocity (angularVelocityA);
		rbB->setAngularVelocity (angularVelocityB);
		return;

	}

	// else we are in the sliding friction case
	//
	// tangent direction of collision
	tangentRelativeVelocity = relativeVelocity - normalRelativeVelocity * normal;
	Vec3 tangent; // zero init
	float magnitude;
	if ((magnitude = tangentRelativeVelocity.length ()) != 0.0f)
		tangent = tangentRelativeVelocity / magnitude;

	// scalar normal component of the impulse friction
	//! \todo check, which of the two denominator computations is prettier
	float denominator = (dot (Vec3 (KT * normal), (normal - friction *
												   tangent)));
//	float denominator = dot (normal, Vec3 (KT * (normal - friction *tangent)));

	float normalImpulse = 0.0f;
	if (denominator != 0.0f)
		normalImpulse = ((-1) * (restitution + 1) * normalRelativeVelocity) /
			denominator;

	// compute impulse for sliding friction
	impulse = normalImpulse * normal - friction * normalImpulse * tangent;

	// ??? Just save something strage...
	float grindMagnitude = friction * normalImpulse * tangent.length ();
	contactPoint.setGrindMagnitude (grindMagnitude);

	// set impulse for sliding friction
	velocityA += invMassA * impulse;
	velocityB -= invMassB * impulse;
	//! \todo Why is the angular velocity computation produceing ugly rotations?
// 	angularVelocityA += invWorldInertiaTensorA * (cross (radiusA, impulse));
// 	angularVelocityB -= invWorldInertiaTensorB * (cross (radiusB, impulse));
	rbA->setVelocity (velocityA);
	rbB->setVelocity (velocityB);
	rbA->setAngularVelocity (angularVelocityA);
	rbB->setAngularVelocity (angularVelocityB);
	return;

	// here we supply code for the frictionless case

//  //! velocityA, angularVelocityA, velocityB, angularVelocityB,
//  //! invWorldInertiaTensorA, invWorldInertiaTensorB, restitutionA,
//  //! restitutionB
//
//  //! coefficient of restitution (use minimum of the two coefficients)
//  float restitution = min (restitutionA, restitutionB);
//
//  //! original relative velocity at the point of collision
//  Vec3 relativeVelocity; // = ...
//
//  //! normal direction of collision
//  Vec3 normal;
//
//  //! scalar normal component of relative velocity
//  float normalRelativeVelocity = dot (relativeVelocity, normal);
//
//  //! vector tangential component of relative velocity
//  Vec3 tangentRelativeVelocity = relativeVelocity - normalRelativeVelocity *
//    normal;
//
//  //! radius vectors of the collision points in world coordinates
//  Vec3 radiusA;
//  Vec3 radiusB;
//
//  //! the 'K' values and their sum
//  Matrix3x3 KA = Matrix3x3::createDiagonal (invMassA) + radiusA.star ().T () *
//    invWorldInertiaTensorA * radiusA.star ();
//  Matrix3x3 KB = Matrix3x3::createDiagonal (invMassB) + radiusB.star ().T () *
//    invWorldInertiaTensorB * radiusB.star ();
//  Matrix3x3 KT = KA + KB;

//  //! scalar normal component of impulse
//  //! note that N^T K_T N equals (K_T N) dot N
//  float denominator2 = dot (Vec3 (KT * normal), normal);
//  float normalImpulse = 0.0f;
//  if (denominator2 != 0.0f) normalImpulse = (1.0f / dot (Vec3 (KT * normal),
//        normal)) * ((-1) * (restitution + 1) * normalRelativeVelocity);
//
//  //! impulse to be applied
//  Vec3 impulse = normalImpulse * normal;
//
//  // set impulse for frictionless case
//  velocityA += invMassA * impulse;
//  velocityB -= invMassB * impulse;
//  angularVelocityA += invWorldInertiaTensorA * (cross (radiusA, impulse));
//  angularVelocityB -= invWorldInertiaTensorB * (cross (radiusB, impulse));
//  objectA->setVelocity (velocityA);
//  objectB->setVelocity (velocityB);
//  objectA->setAngularVelocity (angularVelocityA);
//  objectB->setAngularVelocity (angularVelocityB);
//  return;

}


//! \param forCollision flag indicates "not for contact"
void GeometrySystem::collisionResponse(Interference &interference, 
									   bool forCollision, 
									   float restitution){

    GeometryPtr objA = interference.getObjectA();
    GeometryPtr objB = interference.getObjectB();

	int length = interference.getNumOfContacts();
	for (int i = 0; i < length; ++i){
		
		ContactPoint& point = interference.getContactPoint(i);

        //collisionResponseOld(objA,objB,point,restitution);
        collisionResponse(objA, objB, 
						  point, 
						  forCollision, 
						  restitution);
	}
}


void GeometrySystem::printContactGraph()
{
	/*
	  InterferencePropertyMapType interference = get(edge_interference_property, contactGraph);
	  ComponentIdPropertyMapType componentIdMap = get(vertex_component_id_property, contactGraph);  
	  IdPropertyMapType id = get(vertex_id_property, contactGraph);

	  VertexIterator ui, uiend;
	  boost::tie(ui, uiend) = vertices(contactGraph);

	  CollisionMessages << "Here comes the contact Graph with strong components.\n" << SimonState::endm;
	  for (; ui != uiend; ++ui)
	  {
	  OutEdgeIter out, out_end;
	  CollisionMessages << "Vertex mit ID: " << id[*ui] << " is in component " << componentIdMap[*ui] << ":\n" << SimonState::endm;
	   

	  boost::tie(out, out_end) = out_edges(*ui, contactGraph);
	  for(; out != out_end; ++out){

	  CollisionMessages << "--(Interference: " 
	  << interference[*out]->getObjectA()->getId() 
	  << " liegt auf "
	  << interference[*out]->getObjectB()->getId() 
	  << ")--> "
	  << id[target(*out,contactGraph)] 
	  << SimonState::endm;
	  
	  }
	  }
	*/
}

/*
 * \brief Behandelt schnell die Kontakte (vs. doCollision)
 * \param interval stepsize of the integrator
 * \param quality define the acurracy of the computation. 1 for low and 2 for high
*/
void GeometrySystem::resolveContactsFast(float interval, unsigned int quality) {

	// if there are no elements to check contacts for -> return
	if (vCollisionObjects.size() == 0) 
		return;

	int numOfObjects = vCollisionObjects.size();

    std::map< Id, VertexDescriptor> id2vertices;
    int indexCount = 0;
	
	// clear the Data
	mInterferences.clear();	
	contactGraph.clear();

	// often used temporal variables
	Vec3 
		oldPrevVelocity, 
		oldVelocity,
		oldPrevAngularVelocity,
		oldAngularVelocity,
		oldPrevPosition,
		oldPosition;
	Quaternion 
		oldPrevOrientation,

		oldOrientation;
	bool 
		useViscosity = SimonState::exemplar()->getViscosityFlag();				
	float 
		viscositySlowdownLinear = SimonState::exemplar()->getViscositySlowdownLinear(),
		viscositySlowdownAngular = SimonState::exemplar()->getViscositySlowdownAngular(),
		tempPerformance = 0;
	GeometryPtr 
		obj1,
		obj2;
	RigidBodyPtr 
		rb1,
		rb2;
	VertexDescriptor 
		s,
		s2;
	Clock performance,
		tempTimer;

	performance.start();
	// build up the contact graph of the strongly connected components
	for (int i = 0; i < numOfObjects; ++i){

		rb1 = vCollisionObjects[i]->getRigidBody();

		//! Don't work with object if they have ininite mass
		if (rb1->getInvMass()==0)
			continue;
		
		rb1->backupPositions();
		rb1->backupVelocities();

		rb1->integrate(interval, useViscosity, 
					   viscositySlowdownLinear, 
					   viscositySlowdownAngular,
					   false);

		rb1->restoreVelocities();

		obj1 = vCollisionObjects[i];

		mInterferences.clear();
		// find all interferences with this single object
		mCollisionDetection->findInterferences(obj1, mInterferences);

		int numOfInterferences = mInterferences.size();
		for (int j = 0; j < numOfInterferences; ++j) {

			// interference->getObjectA should be obj1, if it is not: swap
			if (mInterferences[j]->getObjectA() != obj1)
				mInterferences[j]->swap();
				
			// Add a vertex for obj1 
			if (id2vertices.find(rb1->getId()) == id2vertices.end()) { 
				s = boost::add_vertex(
					VertexProperty(

						obj1.get(),ComponentProperty(
							0,IdProperty(
								rb1->getId(),
								IndexProperty(indexCount)
								)
							)
						), contactGraph);
											   
				id2vertices.insert(make_pair(rb1->getId(), s));
				++indexCount;
			} else 
				s = id2vertices[rb1->getId()];


			GeometryPtr obj2 = mInterferences[j]->getObjectB();
			RigidBodyPtr rb2 = obj2->getRigidBody();
			
			assert (obj2);
			
			// add vertex for obj2 
			if (id2vertices.find(rb2->getId()) == id2vertices.end()) { 
				s2 = boost::add_vertex(
					VertexProperty(
						obj2.get(),ComponentProperty(
							0, IdProperty(
								rb2->getId(),
								IndexProperty(indexCount)
								)
							)
						), contactGraph);
												
				id2vertices.insert(make_pair(rb2->getId(), s2));
				++indexCount;
			} else 
				s2 = id2vertices[rb2->getId()];
 
			// create edge between obj2 and obj1
			// an edge a->b means a lies below b
			add_edge(s2, s, InterferenceProperty(mInterferences[j]), contactGraph);
		}

		rb1->restorePositions();
	}

	performance.stop();
	// 	cout << "building graph:" << performance.getDuration() << endl;
	// 	cout << "temp:" << tempPerformance << endl;

	// quit if there are no elements in the contact graph
	if (num_vertices(contactGraph) == 0)
		return;

	ComponentIdPropertyMapType pmap = 
		get( vertex_component_id_property, contactGraph);

	int numComponents = strong_components(contactGraph, pmap);
				
	InterferencePropertyMapType interference = 
		get(edge_interference_property, contactGraph);

	ComponentIdPropertyMapType componentIdMap = 
		get(vertex_component_id_property, contactGraph);

    IdPropertyMapType id = get(vertex_id_property, contactGraph);  
				
	for (int graphIterations = 1; 
		 graphIterations <= CONTACT_PROCESSING_PER_GRAPH; 
		 ++graphIterations) {
			
		for (int level = 0; level < numComponents; ++level) {

			// find Interferences with Objects in Component with ID i
			std::vector< InterferencePtr > interferencesOnLevel(10);
			interferencesOnLevel.clear();

			EdgeIterator out, out_end;
			boost::tie(out, out_end) = edges( contactGraph);
			for(; out != out_end; ++out)
			{
				// und target component ID gleich level
				if (componentIdMap[target(*out, contactGraph)]==level)
					interferencesOnLevel.push_back(interference[*out]);
			}
        
			
			// if there were no Interferences -> continue
			if (interferencesOnLevel.size() == 0) {
				continue;
			}				

			for (int i = 1; i <= CONTACT_PROCESSING_PER_LEVEL; ++i){

                //! \todo are all interferences with lower levels processed?
				//! \todo are they processed in order of increasing level?

				int length = interferencesOnLevel.size();

				for (int i = 0; i < length; ++i){

					// Shock Propagation
					if (graphIterations == CONTACT_PROCESSING_PER_GRAPH)
						interferencesOnLevel[i]->getObjectB()->getRigidBody()->freeze();

					// Restitution has shrinking values from -0.9,
					// -0.7, -0.6 ... to 0.
					float restitution = 
						1.f - (i * (10.f / (float)CONTACT_PROCESSING_PER_LEVEL)) * 0.1f;
					restitution = -restitution;

                    bool forCollision = false;
					collisionResponse(*interferencesOnLevel[i], forCollision, restitution);
				}
				
				// only one iteration for shock propagation	
				if (graphIterations == CONTACT_PROCESSING_PER_GRAPH) 
					continue;

			} // for CONTACT_PROCESSING_PER_LEVEL
		} // for numComponents
	} // for CONTACT_PROCESSING_PER_GRAPH

	GeometryPropertyMapType geometryMap = 
		get(vertex_geometry_property, contactGraph);  
	VertexIterator ui, uiend;
	boost::tie(ui, uiend) = vertices(contactGraph);
				
	// unfreeze all objects
	for (; ui != uiend; ++ui)
		geometryMap[*ui]->getRigidBody()->unfreeze();

	// IMORTANT: Delete all interferences
	InterferencePropertyMapType interferenceMap = get(edge_interference_property, contactGraph);
	EdgeIterator ei, eiend;
	boost::tie(ei, eiend) = edges(contactGraph);
				
	// deleteing ...
	for (; ei != eiend; ++ei){
		recheckInterference(interferenceMap[*ei]);
		//delete interferenceMap[*ei];
	}
}

/*
 * \brief Behandelt Kontakte (vs. doCollision)
 * Jedes Objekt kriegt individuell (!) die m�glichkeit sich fortzubewegen,
 * dann wird nach Interferenz gecheckt und mit denen der contactGraph aufgebaut
 * (als Kanten Eigenschaft).
 * 
*/
void GeometrySystem::resolveContacts(float interval) {
	cout << "don't use this anymore" << endl;
}


/**
 * \brief Erstellt eine neue Sphere und liefert Referenz auf SmartPointer zurück
 * \param rigidBody Ein SmartPointer auf den zugehörigen RigidBody.
 * \param radius Gibt den Radius der Kugel an.
 * \return SmartPointer, der dieses Objekt hält.
 */
GeometryPtr 
GeometrySystem::createSphere(RigidBodyPtr& rigidBody, 
							 float radius){

	GeometryPtr t(new Sphere(rigidBody, radius));

	mCollisionDetection->add(t);

	return push_back(t);
}


/**
 * \brief Erstellt eine neue Box und liefert Referenz auf SmartPointer zurück
 *
 * \param rigidBody Ein SmartPointer auf den zugehörigen RigidBody.
 * \param scale Gibt Breite, Höhe und Länge in einem Vector an.
 *
 * \return SmartPointer, der dieses Objekt hält.
 */
GeometryPtr GeometrySystem::createBox(SmartPointer<RigidBody>& rigidBody, Vec3 scale) {
    GeometryPtr t(new Box(rigidBody, scale));
	mCollisionDetection->add(t);
    return push_back( t );
}

/**
 * \brief Erstellt eine neue Ebene und liefert Referenz auf SmartPointer zurück
 *
 * \param _rigidBody Ein SmartPointer auf den zugehörigen RigidBody.
 *
 * \return SmartPointer, der dieses Objekt hält.
 */
GeometryPtr GeometrySystem::createPlane(SmartPointer<RigidBody>& rigidBody)
{
    GeometryPtr t(new Plane(rigidBody));
	mCollisionDetection->add(t);
    return push_back(t);
}

/**
 * \brief Erstellt eine neue Kapsel und liefert Referenz auf SmartPointer zurück
 * \param rigidBody Ein SmartPointer auf den zugehörigen RigidBody
 * \param radius 
 * \param height 
 * \return SmartPointer, der dieses Objekt hält
 */

GeometryPtr GeometrySystem::createCapsule(SmartPointer<RigidBody>& rigidBody, float radius, float height)
{
    GeometryPtr t(new Capsule(rigidBody, radius, height));
	mCollisionDetection->add(t);
    return push_back( t );
}




/**
* \brief Fügt ein Objekt in den Container ein und gibt Referenz zurück
* \param object SmartPointer
*/
GeometryPtr GeometrySystem::push_back( GeometryPtr& object){

    vCollisionObjects.push_back(object);
    return vCollisionObjects.back();
}


void GeometrySystem::sendContactInformation(Interference& interference)
{
	float impulseMagnitude = 0; 
	float grindMagnitude = 0;
	int numOfContacts = interference.getNumOfContacts();
	for (int i = 0; i < numOfContacts; ++i) {
		impulseMagnitude += interference.getContactPoint(i).getImpulseMagnitude();
		grindMagnitude += interference.getContactPoint(i).getGrindMagnitude();			
	}

	if (impulseMagnitude < SimonState::exemplar()->getContactInformationThreshold())
		return;
 
	impulseMagnitude /= (float)numOfContacts;
	grindMagnitude /= (float)numOfContacts;				

	SimonState::exemplar()->
		getContactInformationContainer()->
		addContactInformation(interference.getContactPoint(0).getPosition(),
							  interference.getObjectA()->getRigidBody()->getId(),
							  interference.getObjectB()->getRigidBody()->getId(), 
							  impulseMagnitude, 
							  grindMagnitude);

}

/**
 * \brief berechnet den Luftwiderstand
 */
	void GeometrySystem::airDrag()
	{
		// Luftdichte
		float airDensity = 0.000129f * SimonState::exemplar()->getAirDensityMultiplyer();

		vIterator collisionEnd = vCollisionObjects.end();
		for (vIterator it = vCollisionObjects.begin(); it != collisionEnd; ++it)
		{
			GeometryPtr obj = *it;
			RigidBodyPtr rb = obj->getRigidBody();
			
			int type = rb->getId().getType();
		
			if(type == 1 || type == 2 || type == 4)
			{
		
				Vec3 velocity = rb->getVelocity();
				Vec3 angVelocity = rb->getAngularVelocity();
			
				//Kappen von zu hohen angulaeren Geschwindigkeiten
				float max = 0.0;
				for (int i=0; i < 3; ++i)
					if (fabs(angVelocity[i]) > max)
						max = fabs(angVelocity[i]);
			
				float percentage = 0.0;
				if (max > 2.f)
				{
					percentage = 2.f / max;
			
					for (int i=0; i < 3; ++i)
						angVelocity[i] *= percentage;
				}
				//End Kappen
			
				float mass = rb->getMass();
				float area = obj->getArea();
			
				if (velocity[0] < 0)   
					velocity[0] +=  (area * airDensity * velocity[0] * velocity[0]) / (2 * mass);
				else
					velocity[0] -=  (area * airDensity * velocity[0] * velocity[0]) / (2 * mass);
				
				if (velocity[1] < 0)
					velocity[1] +=  (area * airDensity * velocity[1] * velocity[1]) / (2 * mass);
				else
					velocity[1] -=  (area * airDensity * velocity[1] * velocity[1]) / (2 * mass);
				
				if (velocity[2] < 0)
					velocity[2] +=  (area * airDensity * velocity[2] * velocity[2]) / (2 * mass);
				else
					velocity[2] -=  (area * airDensity * velocity[2] * velocity[2]) / (2 * mass);
				
				if (angVelocity[0] < 0)
					angVelocity[0] +=  (area * airDensity * angVelocity[0] * angVelocity[0]) / (2 * mass);
				else
					angVelocity[0] -=  (area * airDensity * angVelocity[0] * angVelocity[0]) / (2 * mass);
				
				if (angVelocity[1] < 0)
					angVelocity[1] +=  (area * airDensity * angVelocity[1] * angVelocity[1]) / (2 * mass);
				else
					angVelocity[1] -=  (area * airDensity * angVelocity[1] * angVelocity[1]) / (2 * mass);
			
				if (angVelocity[2] < 0)
					angVelocity[2] +=  (area * airDensity * angVelocity[2] * angVelocity[2]) / (2 * mass);
				else
					angVelocity[2] -=  (area * airDensity * angVelocity[2] * angVelocity[2]) / (2 * mass);
				
				rb->setVelocity(velocity);
				rb->setAngularVelocity(angVelocity);
			}		
		
		
		}
	}


/* \brief Brute Force Korrigieren von Rundungsfehlern durch setPosition
 * \param interference Interference, die gecheckt werden soll
 */
void GeometrySystem::recheckInterference(InterferencePtr interference)
{
	bool interferenceWasSwapped = interference->hasBeenSwapped();
	if (interferenceWasSwapped)
	{
		interference->swap();
		//	cout << "reswapped Interference\n";
		// now objA is again the one with the contactPoint on its surface
		// and objB is that, for which distanceFromB was set.
	}
	
	GeometryPtr objA = interference->getObjectA();
	GeometryPtr objB = interference->getObjectB();
                    
	//           cout << "recheck: A=" <<objA->getId() << " B=" <<objB->getId()<<"\n";
	float maxDifference=0;
	Vec3 normalB;
		
	int numOfContacts = interference->getNumOfContacts();
	for (int i = 0; i < numOfContacts; ++i){

		ContactPoint& cp = interference->getContactPoint(i);

		if (!cp.isDistanceFromBSet()) 
			continue;
                    		
		Vec3 localB = cp.getContactPointInB();
		Vec3 newContactPointWorldCoordinatesB 
			= qRotate( localB, objB->getRigidBody()->getOrientation()) + 
			objB->getRigidBody()->getPosition();
	                            
		Vec3 localA = cp.getContactPointInA();
		Vec3 newContactPointWorld 
			= qRotate( localA, objA->getRigidBody()->getOrientation()) + 
			objA->getRigidBody()->getPosition();
	                            
		float distance = dot(cp.getNormalB(),
							 newContactPointWorld - newContactPointWorldCoordinatesB);

		float difference = cp.getDistanceFromB() - distance;

		if (difference > maxDifference){

			maxDifference=difference;
			normalB = cp.getNormalB();
		}
	}

	if (maxDifference>0)
	{
		bool updateB=true;
		if (objA->getRigidBody()->getInvMass()!=0)
		{
			//Prefer to Move the Object on top.
			if (!interferenceWasSwapped || objB->getRigidBody()->getInvMass()==0)
			{
				updateB=false;
				objA->getRigidBody()->setPosition(objA->getRigidBody()->getPosition() + 
												  maxDifference * normalB);
//		                                cout << "recheckInterference added " << maxDifference * normalB << " to ObjectA\n";
			}
		}       
                            	
		if (updateB)
		{
			objB->getRigidBody()->setPosition(objB->getRigidBody()->getPosition() - 
							  maxDifference * normalB);
//	                                cout << "recheckInterference added " << maxDifference * normalB << " to ObjectB\n";
		}
	}
                            		
                            	
}
