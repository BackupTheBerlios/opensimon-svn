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


#include "SpringConnection.h"
#include <iostream>
using namespace std;
//------------------------------------------------------------------------------
/**
 * \class SpringConnection
 * $Author: trappe $
 * $Date: 2004/10/05 17:09:16 $
 * $Revision: 1.21 $
 * \brief Enthält Methoden für das Feder-Masse-System, Status: grob 
 *
 *
 * Konstruktor holt Partikel aus der Connection und setzt verschiedene 
 * Eigenschaften der Federn.
 * Runge Kutta schon mal aus Rigid Body übernommen.
 * 
 */
//------------------------------------------------------------------------------



/**
 * Aus Connection werden Partikelpositionen geholt, Längenberechnung 
 */

#include "RigidBody.h"

SpringConnection::SpringConnection():Connection() {
	//Federeigenschaften setzen
		    
}


SpringConnection::SpringConnection(ParticlePtr pa, ParticlePtr pb) : Connection(pa, pb){ 
    //Connection connection(particleA, particleB);
    mBegin = pa;
    mEnd = pb;
    mSpringConst = 0.0;
	mRestLength = 0.0;
	mDamping = 0.0;
	
    //Länge berechnen
	Vector3<float> b = mBegin->getPosition();
	Vector3<float> e = mEnd->getPosition();
	Vector3<float> b_to_e = e - b;
	mNormalSpringLength = b_to_e.length();
	mMaxSpringLength = 1.5f * mNormalSpringLength;
	mlength = mNormalSpringLength;
}

SpringConnection::SpringConnection(ParticlePtr pa, ParticlePtr pb, float restlength): Connection(pa, pb){
	mBegin = pa;
	mEnd = pb;
	mSpringConst = 0.0;
	mRestLength = restlength;
	mDamping = 0.0;
	
	//Ruhelänge berechnen
	Vector3<float> b = mBegin->getPosition();
	Vector3<float> e = mEnd->getPosition();
	Vector3<float> b_to_e = e - b;
	mNormalSpringLength = b_to_e.length();
	mMaxSpringLength = 1.5f * mNormalSpringLength;
	mlength = mNormalSpringLength;
}

//! \todo Kommentieren!!!
SpringConnection::SpringConnection(ParticlePtr pa, RigidBodyPtr rb, Vec3 /*contact*/) : Connection(pa,rb){
	mBegin = pa;
	mEnd = rb;
	
	mSpringConst = 0.0;
	mRestLength = 0.0;
	mDamping = 0.0;

	Vector3<float> b = mBegin->getPosition();
	Vector3<float> e = mEnd->getPosition();
	Vector3<float> b_to_e = e - b;
	mNormalSpringLength = b_to_e.length();
	mMaxSpringLength = 1.5f * mNormalSpringLength; 
	mlength = mNormalSpringLength;
}

SpringConnection::~SpringConnection(){
}

/**
 * Federkonstante abfragen
 */

float SpringConnection::getSpringConst() {
	return mSpringConst;
}

/**
 * Federkonstante setzen
 */

void SpringConnection::setSpringConst(float sconst) {
	mSpringConst = sconst;
}


/**
 * Restlänge abfragen
 */
float SpringConnection::getRestLength() {
	return mRestLength;
}

/**
 * Ruhelänge abfragen
 */
float SpringConnection::getNormalSpringLength() {
	return mNormalSpringLength;
}

/**
 * maximale Federlänge abfragen
 */
float SpringConnection::getMaxSpringLength() {
	return mMaxSpringLength;
}

/**
 * Restlänge setzen
 */

void SpringConnection::setRestLength(float restLength) {
	mRestLength = restLength;
}

/**
 * Dämpfungskonstante abfragen
 */
 
float SpringConnection::getDampConst() {
	return mDamping;
}

/**
 * Dämpfungskonstante setzen
 */

void SpringConnection::setDampConst(float dconst) {
	mDamping = dconst;
}

/**
 * Länge der Connection abfragen
 */

float SpringConnection::getLength() {
	return mlength;
}

/**
 * Länge der Connection updaten
 */
void SpringConnection::updateLength() {
	
    //aktuelle Federlänge berechnen

    
    Vector3<float> b = mBegin->getPosition();
	
         
	Vector3<float> e = mEnd->getPosition();
    
	
     Vector3<float> b_to_e = e - b;
	     mlength = b_to_e.length();
   
}


