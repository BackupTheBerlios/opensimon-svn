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


#include "RigidBodySystem.h"

//------------------------------------------------------------------------------
/**
 * \class RigidBodySystem
 * \author Achilles, Baierl, Koehler
 *
 * \brief Das RigidBodySystem ist die Klasse, mit der Festörper
 * (RigidBodies) verwaltet werden.
 *
 *
 * 
 */
//------------------------------------------------------------------------------

#include <iostream>
using namespace std;

/**
 * \brief Default Konstruktor für Default Werte
 *
 * - Viscosity (Klebrigikeit) ist eingeschaltet.
 * - Anguläre Viscosity ist 0,99
 * - Lineare Viscosity ist 0,99
 */
RigidBodySystem::RigidBodySystem() {
}

/**
 * \brief Default Dekonstruktor
 */
RigidBodySystem::~RigidBodySystem() {
}

/**
 * \brief Liefert ein Pointer auf einen neuen Festkörper zurück
 * \param Id wird zur identifikation des rigidbodies benutzt
 * \return RigidBody*
 */
RigidBodyPtr RigidBodySystem::create(Id id) {
	RigidBodyPtr rb(new RigidBody());
	rb->setId(id);
	mBodyList[id] = rb;
	return rb;
}

/**
 * \brief Schaltet die Viscosiät ein.
 * Weiterleitung an Simonstate
 */
void RigidBodySystem::enableViscosity()
{
	SimonState::exemplar()->enableViscosity();
}

/**
 * \brief Schaltet die Viscosity aus.
 * Weiterleitung an Simonstate 
 */
void RigidBodySystem::disableViscosity()
{
	SimonState::exemplar()->disableViscosity();
}

/**
 * \brief Liefert ein Pointer auf den FestKörper mit der Id des übergebenen Id-Objekts zurück, falls es vorhanden ist, sonst einen null-Pointer
 * \param Id wird zur identifikation des rigidbodies benutzt
 * \return RigidBodyPtr
 */
RigidBodyPtr RigidBodySystem::getRigidBody(Id id) {
  	if (mBodyList.count(id) != 0) 
  		return mBodyList[id];
  	else {
 		RigidBodyPtr rb;
 		return rb;
 	}
}

/**
 * \brief Liefert die Anzahl der gespeicherten FestKörper
 * \return int
 */
int RigidBodySystem::getNumberOfRigidBodies() {
	return static_cast<int>(mBodyList.size());
}

//! \todo diese Funktion sollte entfernt werden (trappe)
const std::map<Id, RigidBodyPtr >* RigidBodySystem::getMap() {
	return &mBodyList;
}

/**
 * Diese Funktion iteriert über alle RigidBodys und ruft die übegebene Funktion 
 * mit dem aktuellen Body als Parameter auf. Sorry, aber es ist so!
 *
 * \param givenFunction Ein Funktionspointer auf eine Funktion die aufgerufen werden soll.
 */
void RigidBodySystem::forEveryRigidBodyCall(void (*givenFunction)(RigidBodyPtr)) {
	
	RigidBodyList::iterator listEnd = mBodyList.end();
	for(
		RigidBodyList::iterator body = mBodyList.begin();
		body != listEnd;
		++body) {
		givenFunction((body->second));
	}		
}

/**
 * \brief Löscht den FestKörper mit der Id des übergebenen Id-Objekts, falls vorhanden
 * \param Id wird zur identifikation des rigidbodies benutzt
 */
void RigidBodySystem::deleteRigidBody(Id id) {
	std::cout << "Die Löschenfunktion vom RigidBodySystem wurde noch nie getestet. Viel Spass!" << std::endl;
	if (!mBodyList.count(id))
		mBodyList.erase(id);
	else
		//! \todo bitte SimonState's errors benuzten!
		printf("Zu loeschendes Objekt (%d) existiert nicht.", id.getNumber());
}

/**
 * \brief Addiert die Schwerkraft auf alle Festkörper auf
 */
void RigidBodySystem::addGravity() {

	// if there are no elements -> return
	if (mBodyList.size() == 0) 
		return;

	std::map<Id, RigidBodyPtr >::iterator endMap = mBodyList.end();
	for (std::map<Id, RigidBodyPtr >::iterator p = mBodyList.begin(); p != endMap; ++p) {
		//if(p->second->getId().getType() == Id::typeControlSphere)
		//	continue;
		//Gravitation aufaddieren F = m * g
		p->second->addForce(SimonState::exemplar()->getGravityVector()* p->second->getMass());

	}
}


/**
 * \brief Integration mit dem Integrator, der in SimonState.getIntegrator() definiert ist.
 * \param Interval in Millisekunden
 * Integrator durchläuft alle RigidBodies, die im System gespeichert sind
 */
void RigidBodySystem::integrate(float interval)
{
  switch(SimonState::exemplar()->getIntegrator()){
    case SimonState::INTEGRATE_EULER:
    	integrateEuler(interval);
        break;
    default:
		break;
  }
}

void RigidBodySystem::integrateVelocities(float interval){

  switch(SimonState::exemplar()->getIntegrator()){
    case SimonState::INTEGRATE_EULER:
    	integrateEulerVelocities(interval);
        break;
    default:
		break;
  }
}


void RigidBodySystem::integratePositions(float interval)
{
  switch(SimonState::exemplar()->getIntegrator()){
    case SimonState::INTEGRATE_EULER:
    	integrateEulerPositions(interval);
        break;
    default:
		break;
  }
}


  
/**
 * \brief Integration mit Euler Methode
 *
 * \param Interval in Millisekunden
 *
 * Integrator durchläuft alle RigidBodies, die im System gespeichert sind
 */
void RigidBodySystem::integrateEuler(float interval){

	integrateEulerVelocities(interval);
	integrateEulerPositions(interval);
}


/**
 * \brief Integration mit Euler Methode (erster Schritt)
 *
 * \param Interval in Millisekunden
 *
 * Integrator durchläuft alle RigidBodies, die im System gespeichert sind
 */
void RigidBodySystem::integrateEulerVelocities(float interval){

	std::map<Id, RigidBodyPtr >::iterator endOfItr = mBodyList.end();
for (std::map<Id, RigidBodyPtr >::iterator p = mBodyList.begin(); p != endOfItr; ++p)
	{
		//! \todo kann sein, das der integrator ein clear auf das system macht, deshalb wird hier getestet (hack)
		if(p->second)
			p->second->integrateEulerVelocities(interval);
	}
}

/**
 * \brief Integration mit Euler Methode (zweiter Schritt)
 *
 * \param Interval in Millisekunden
 *
 * Integrator durchläuft alle RigidBodies, die im System gespeichert sind
 */
void RigidBodySystem::integrateEulerPositions(float interval){

    bool useViscosity = SimonState::exemplar()->getViscosityFlag();
    float viscositySlowdownLinear = SimonState::exemplar()->getViscositySlowdownLinear();
    float viscositySlowdownAngular = SimonState::exemplar()->getViscositySlowdownAngular();    
	std::map<Id, RigidBodyPtr >::iterator endOfItr = mBodyList.end();
	for (std::map<Id, RigidBodyPtr >::iterator p = mBodyList.begin(); p != endOfItr; ++p)
	{
		p->second->integrateEulerPositions(interval, useViscosity, viscositySlowdownLinear, viscositySlowdownAngular);
	}
}
