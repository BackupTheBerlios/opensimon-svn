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


#include "ClothSystem.h"
#include "ParticleSystem.h"
#include <iostream>

//------------------------------------------------------------------------------
/**
 * \file ClothSystem.cpp
 * \brief Verwaltung der SpringConnections und Eintrag in eine Map
 *
 * 
 */
//------------------------------------------------------------------------------

//! \todo Kommentieren
//! \todo Nicht Global definieren, sondern über get/set's im ClothSystem!
//float deltaT=10.0f;                  // Intervalls-Schritte empfohlen: 0.1f

Vector3<float> force1,force2;


ClothSystem::ClothSystem(){
}


//! \diese funktion sollte entfernt werden

const std::map<Id, SpringConnectionPtr >* ClothSystem::getMap(){

return &mSpringConnectionList;
} 
    

/**
 * \brief erstellt aus zwei Partikeln eine Federverbindung und setzt Standardwerte
 */

SpringConnectionPtr ClothSystem::createSpringConnection(ParticlePtr particleBegin,ParticlePtr particleEnd, Id id)  {
	SpringConnectionPtr connect(new SpringConnection(particleBegin,particleEnd));
	connect->setId(id);
//	cout << particleBegin->getPosition() << " & " << particleEnd->getPosition() << endl;
	//connect->setSpringConst(0.05f); // Feder-Konstante gut: 0.005f
	//connect->setDampConst(0.0001f);     // Dämpfungs-Konstante gut: 0.01f
	connect->setSpringConst(0.01f); 
	connect->setDampConst(0.02f);     
	connect->setRestLength(connect->getNormalSpringLength());

	mSpringConnectionList[id] = connect;
	return connect;
}

/**
 * \brief erstellt eine Federverbindung aus Partikel und RigidBody
 * \setzt Standardwerte
 */
SpringConnectionPtr ClothSystem::createSpringConnection(ParticlePtr particleBegin, RigidBodyPtr rigidBodyEnd, Vector3<float> contactPoint, Id id){
	
	SpringConnectionPtr connect(new SpringConnection(particleBegin, rigidBodyEnd, contactPoint));
	connect->setId(id);
	
	connect->setSpringConst(1.0f);
	connect->setDampConst(0.001f);
	connect->setRestLength(connect->getNormalSpringLength());
	
	mSpringConnectionList[id] = connect;
	return connect;


}



/**
 * \brief Löscht den Partikel mit der Id des übergebenen Id-Objekts, falls vorhanden
 */
void ClothSystem::deleteSpringConnection(Id id) {
	if (mSpringConnectionList[id])
		mSpringConnectionList.erase(id);
	else 
		printf("Zu löschendes Objekt (%d) existiert nicht.", id.getNumber());
}



/**
 * \brief berechnet die Kräfte, die auf jeden Partikel wirken und addiert diese jeweils auf
 */
void ClothSystem::computeCloth(){
	
	//applyIK(0.1);
	//applyIK(1.0);

	std::map<Id, SpringConnectionPtr>::iterator endOfMap = mSpringConnectionList.end();
    // Schleifendurchlauf über alle Federn
	for (std::map<Id, SpringConnectionPtr>::iterator spring = mSpringConnectionList.begin(); spring != endOfMap; ++spring) {

//! ToDo Federverkürzung aus eigener Funktion holen-----------------------


		// aktuelle Länge der Feder als Vektor 
		Vector3<float> springVector = spring->second->getObjectA()->getPosition() - spring->second->getObjectB()->getPosition();
		// Geschwindigkeitsdifferenz der beiden verbundenen Partikel
        Vector3<float> velocity = spring->second->getObjectA()->getVelocity() - spring->second->getObjectB()->getVelocity();
		float normalSpringLength = spring->second->getNormalSpringLength();
		float mLength;
		float dampingForce;

		if (springVector.length() != 0)
		{
			mLength = springVector.length();
			dampingForce = ((dot(velocity,springVector) / mLength)* spring->second->getDampConst());
			
//------------mehrere SpringVector-Abfragen------------------------------------
			/*
			// falls aktuelle Federlänge kleiner als Ruhelänge wird die Ruhelänge gesetzt
			// ansonsten die aktuelle Federlänge
			if (springVector.length() <= normalSpringLength)
			{
				mLength = normalSpringLength;
				float difference = normalSpringLength - springVector.length();
				springVector.normalize();
				
				Vec3 differenceVector = (difference/2) * springVector;
				if (spring->second->getObjectA()->getIsDynamicFlag())
					spring->second->getObjectA()->setPosition(spring->second->getObjectA()->getPosition() + differenceVector);
				if (spring->second->getObjectB()->getIsDynamicFlag())
					spring->second->getObjectB()->setPosition(spring->second->getObjectB()->getPosition() - differenceVector);
				
				springVector = springVector * normalSpringLength;
				dampingForce = ((dot(velocity,springVector) / mLength)* spring->second->getDampConst());
			}
			else 
				if (springVector.length() > normalSpringLength && springVector.length() <= spring->second->getMaxSpringLength())
				{
					mLength = springVector.length();
					dampingForce = ((dot(velocity,springVector) / mLength)* spring->second->getDampConst());
				}
				else
				{
					
					mLength = spring->second->getMaxSpringLength();
					float difference = springVector.length() - mLength;
					springVector.normalize();

			
					Vec3 differenceVector = (difference/2) * springVector;
					if (spring->second->getObjectA()->getIsDynamicFlag())
						spring->second->getObjectA()->setPosition(spring->second->getObjectA()->getPosition() - differenceVector);
					if (spring->second->getObjectB()->getIsDynamicFlag())
						spring->second->getObjectB()->setPosition(spring->second->getObjectB()->getPosition() + differenceVector);
					
					springVector = springVector * spring->second->getMaxSpringLength();
					if (spring->second->getObjectA())
						spring->second->getObjectA()->addForce(-(SimonState::exemplar()->getGravityVector()* spring->second->getObjectA()->getMass()));
					spring->second->getObjectA()->setForce(Vec3(0.0f,0.0f,0.0f));
					if (spring->second->getObjectB())
						spring->second->getObjectB()->addForce(-(SimonState::exemplar()->getGravityVector()* spring->second->getObjectB()->getMass()));
					spring->second->getObjectB()->setForce(Vec3(0.0f,0.0f,0.0f));
					dampingForce = ((dot(velocity,springVector) / mLength)* spring->second->getDampConst());
				
				}
			*/
//---------------Ende der Abfragen-----------------------------------------------
			// Berechnung der Federkraft Kraft = Restlänge * Federkonstante
			float springForce = 1.0f * (mLength - normalSpringLength) * spring->second->getSpringConst();
			float force = (-1) * (springForce + dampingForce);
		
			// Kraftrichtung
			force1 = (springVector / mLength) * force;
			force2 = (-1) * force1;

			if (spring->second->getObjectA()->getIsDynamicFlag()){
				spring->second->getObjectA()->addForce(force1);
			}
    
			if (spring->second->getObjectB()->getIsDynamicFlag()){
				spring->second->getObjectB()->addForce(force2);
			}
		
		}
		else {

			SimonState::exemplar()->errors 
				<< "Simon: Two Particles have the same Position" 
				<< endl
				<< "therefore SpringVector Normalization failed!" 
				<< endl
				<< "This is not good!"
				<< SimonState::endm;

			deleteSpringConnection(spring->second->getId());
			continue;
		}
	}    

}
/**
 * \brief Federverkürzung
 */
//---------aus altem stoff---------------------
//Federverkürzung nach Provot at al.
float testFaktor = -0.5f;
void ClothSystem::applyIK(float dt) {
	Vector3<float> c(0.0,0.0,0.0);
	Vec3 cA(0.0,0.0,0.0);
	Vec3 cB(0.0,0.0,0.0);
	Vector3<float> o(0.0,0.0,0.0);
	float t=0.0;
	float amount=0.0;

	Vector3<float> dir(0.0,0.0,0.0);

	for (std::map<Id, SpringConnectionPtr>::iterator spring = mSpringConnectionList.begin(); spring != mSpringConnectionList.end(); ++spring)
{


		c = spring->second->getObjectB()->getPosition() - spring->second->getObjectA()->getPosition();
		cA = spring->second->getObjectA()->getForce();
		cB = spring->second->getObjectB()->getForce();
		o = spring->second->getObjectB()->getPosition() - spring->second->getObjectA()->getPosition();
		t = (c.length() - spring->second->getRestLength() ) / spring->second->getRestLength();
		
		if (t > dt) {
			dir = c;
			if (cA.length() != 0.0)
				cA.normalize();
			if (cB.length() != 0.0)
				cB.normalize();
			if (cB.length() != 0.0)
				dir.normalize();
			amount = c.length() - spring->second->getRestLength();
			
			bool isCollidingA = SimonState::exemplar()->
				getParticleSystem()->
				isColliding(boost::static_pointer_cast<Particle>(spring->
																 second->
																 getObjectA()));
			bool isCollidingB = SimonState::exemplar()->
				getParticleSystem()->
				isColliding(boost::static_pointer_cast<Particle>(spring->
																 second->
																 getObjectB()));
			int colWhereA =  SimonState::exemplar()->
				getParticleSystem()->
				isCollidingWhere(boost::static_pointer_cast<Particle>(spring->
																	  second->
																	  getObjectA()));
			int colWhereB =  SimonState::exemplar()->
				getParticleSystem()->
				isCollidingWhere(boost::static_pointer_cast<Particle>(spring->
																	  second->
																	  getObjectB()));

				if (!(isCollidingA && isCollidingB)){
					if (spring->second->getObjectA()->getIsDynamicFlag() || spring->second->getObjectB()->getIsDynamicFlag()) {//wenn mindestens ein Particle beweglich ist gehe in Schleife
						if (!(spring->second->getObjectA()->getIsDynamicFlag()) || 
							isCollidingA) { //Object B näher an A rücken
							dir = (-1) * dir * amount;
							spring->second->getObjectB()->setPosition(spring->second->getObjectB()->getPosition() + dir);
							if (spring->second->getObjectA()->getIsDynamicFlag()){
							spring->second->getObjectA()->setPosition(spring->second->getObjectA()->getPosition() + testFaktor * cA * (float)colWhereA);
							}
						} else if (!(spring->second->getObjectB()->getIsDynamicFlag())|| isCollidingB) { //Objekt A  näher an B rücken
							dir = dir *	amount;
							spring->second->getObjectA()->setPosition(spring->second->getObjectA()->getPosition() + dir);
							if (spring->second->getObjectB()->getIsDynamicFlag())
							spring->second->getObjectB()->setPosition(spring->second->getObjectB()->getPosition() + testFaktor * cB * (float)colWhereB);
						} else { //Beide bewegen
							dir = (-0.5) * dir * amount;
							spring->second->getObjectB()->setPosition(spring->second->getObjectB()->getPosition() + dir);
							
							dir = (-1.0) * dir;
							
							spring->second->getObjectA()->setPosition(spring->second->getObjectA()->getPosition() + dir);
							
						}
					}
					
				} else if (isCollidingA && isCollidingB){
					spring->second->getObjectA()->setPosition(spring->second->getObjectA()->getPosition() + testFaktor * cA);
					spring->second->getObjectB()->setPosition(spring->second->getObjectB()->getPosition() + testFaktor * cB);
				}
		}
}
}
//--------------aus altem stoff-----------------------------
 
