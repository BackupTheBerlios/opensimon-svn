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


#include "Interference.h"
#include "Geometry.h"
#include <cassert>
#include <iostream>
using namespace std;

/** \brief creates an empty Interference Objekt  */
Interference::Interference() {   
	mHasBeenSwapped = false;
}

/** 
 * \brief creates an Interference Objekt.
*/
Interference::Interference(GeometryPtr objectA, GeometryPtr objectB)
:mObjectA(objectA), mObjectB(objectB) {
	assert(objectA);
	assert(objectB);
	assert(objectA != objectB);
    mHasBeenSwapped = false;
}


/** \brief adds a contact point
 * 
 * \param worldPosition position of contact point in world coordinates
 * \param normal the normal of the contact point from objectB
 */
void Interference::addContactPoint(Vec3 worldPosition, Vec3 normal)
{
	const Vec3& rA = worldPosition - mObjectA->getRigidBody()->getPosition();
	const Vec3& rB = worldPosition - mObjectB->getRigidBody()->getPosition();
//    cout << "Interference::addContactPoint normalB= " << normalB << "\n";
	mContactPoints.push_back( ContactPoint(worldPosition, normal, rA, rB));
}


/** 
 * \brief adds a contact point
 *
 * \param worldPosition position of contact point in world coordinates
 * \param normal the normal of the contact point from object B 
 * \param distanceFromB distance of contact point from object B
 *  normal scaled with distance should be at the border of B
 */
void Interference::addContactPoint(Vec3 worldPosition, Vec3 normalB, float distanceFromB)
{
	const Vec3& rA = worldPosition - mObjectA->getRigidBody()->getPosition();
	const Vec3& rB = worldPosition - mObjectB->getRigidBody()->getPosition();
//    cout << "Interference::addContactPoint normalB= " << normalB << "\n";
	mContactPoints.push_back( ContactPoint(worldPosition, normalB, rA, rB, distanceFromB));
}


ContactPoint& Interference::getContactPoint(int index){
	return mContactPoints[index];
}

int Interference::getNumOfContacts(){
	return mContactPoints.size();
}

/** \brief swaps ObjectA and ObjectB
  * Swap objects A and B. We need this to find out for which object findInterference was called in the first place
*/
void Interference::swap()
{
	mHasBeenSwapped = !mHasBeenSwapped;
    GeometryPtr temp = mObjectA;
	mObjectA = mObjectB;
	mObjectB = temp;

	int numOfContacts = getNumOfContacts();
	for (int i = 0; i < numOfContacts; ++i){
		mContactPoints[i].swap();
   }
}

const bool& Interference::hasBeenSwapped() const
{
	return mHasBeenSwapped;	
}
	
GeometryPtr Interference::getObjectA() { 
	return mObjectA; 
}

GeometryPtr Interference::getObjectB() { 
	return mObjectB; 
}

void Interference::setObjectA(GeometryPtr a) { 
	assert(a);
	mObjectA = a;
}

void Interference::setObjectB(GeometryPtr b) { 
	assert(b);
	mObjectB = b;
}

std::ostream& operator <<(std::ostream& os, Interference& interference)
{
	os << "Interference between A=" << interference.mObjectA->getRigidBody()->getId() 
	   << " & B=" << interference.mObjectB->getRigidBody()->getId() 
	   << "; contact points: ";

	for (int i = 0; i < interference.getNumOfContacts(); ++i){
		os << "[Position=" << interference.getContactPoint(i).getPosition() 
		   << " | NormalB=" << interference.getContactPoint(i).getNormalB()
		   << "]";
	}

	return os;
}
