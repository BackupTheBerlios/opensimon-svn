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
 * \file ContactPoint.cpp
 * \class ContactPoint
 *
 * \author Kilian
 *
 *
 * \brief Steuerklasse zur Kollisionserkennung
 *
 * Diese Klassse hält eine Liste von zu testenden Objekten. Mit checkCollisions
 * wird die Kollisionserkennung gestartet.
 */
//------------------------------------------------------------------------------

#include "ContactPoint.h"
#include "SmartPointer.h"

ContactPoint::ContactPoint()
{
	mWorldPosition = Vec3(0,0,0); 
	mNormalB = Vec3(0,0,0);
	mRA = Vec3(0,0,0);
	mRB = Vec3(0,0,0);
    mImpulseMagnitude=0.0;
    mGrindMagnitude=0.0;
    mDistanceFromB=0.0;
    mDistanceFromBProvided=false;
    mContactPointInA = Vec3(0,0,0);
    mContactPointInB = Vec3(0,0,0);
}

ContactPoint::ContactPoint(
	const Vec3& worldPosition,
	const Vec3& normalB,
	const Vec3& rA,
	const Vec3& rB)
{
	mWorldPosition = worldPosition; 
	mNormalB = normalB; 
	mRA = rA; 
	mRB = rB;
    mImpulseMagnitude=0.0;
    mGrindMagnitude=0.0; 
    mDistanceFromB=0.0;
    mDistanceFromBProvided=false;
    mContactPointInA = Vec3(0,0,0);
    mContactPointInB = Vec3(0,0,0);
}

ContactPoint::ContactPoint(
	const Vec3& worldPosition,
	const Vec3& normalB,
	const Vec3& rA,
	const Vec3& rB,
	float distanceFromB)
{
	mWorldPosition = worldPosition; 
	mNormalB = normalB; 
	mRA = rA; 
	mRB = rB;
    mImpulseMagnitude=0.0;
    mGrindMagnitude=0.0; 
    mDistanceFromB=distanceFromB;
    mDistanceFromBProvided=true;
    mContactPointInA = Vec3(0,0,0);
    mContactPointInB = Vec3(0,0,0);
}

Vec3 ContactPoint::getPosition() const {
	return mWorldPosition;
}

Vec3 ContactPoint::getNormalA() const { 
	return -mNormalB;
}

const Vec3& ContactPoint::getNormalB() const {
	return mNormalB;
}

const Vec3& ContactPoint::getRA() const { 
	return mRA;
}

const Vec3& ContactPoint::getRB() const {
	return mRB;
}    

const Vec3& ContactPoint::getContactPointInA() const
{
  return mContactPointInA;
}

const Vec3& ContactPoint::getContactPointInB() const
{
  return mContactPointInB;
}
  
const float& ContactPoint::getImpulseMagnitude() const
{
	return mImpulseMagnitude;
}
const float& ContactPoint::getGrindMagnitude() const
{
	return mGrindMagnitude;
}

const float& ContactPoint::getDistanceFromB() const
{
	return mDistanceFromB;
}

const bool& ContactPoint::isDistanceFromBSet() const
{
	return mDistanceFromBProvided;
}

void ContactPoint::setImpulseMagnitude(float impulseMagnitude)
{
	mImpulseMagnitude = impulseMagnitude;
}

void ContactPoint::setGrindMagnitude(float grindMagnitude)
{
	mGrindMagnitude = grindMagnitude;
}

void ContactPoint::setContactPointInA(Vec3& contactPointInA)
{
    mContactPointInA = contactPointInA;
}

/* Dies ist der Contaktpunkt, in lokalen koordinaten von B
 * wird in CollisionSystem::recheckInterference gebraucht um den Contakt-punkt
 * nach der kompletten Behandlung zu berechnen.*/
void ContactPoint::setContactPointInB(Vec3& contactPointInB)
{
    mContactPointInB = contactPointInB;
}

void ContactPoint::setDistanceFromB(float distanceFromB)
{
	mDistanceFromB = distanceFromB;
	mDistanceFromBProvided = true;
}

/** \brief swaps ObjectA and ObjectB
  * Swap objects A and B. We need this to find out for which object findInterference was called in the first place
*/
void ContactPoint::swap()
{
    Vec3 temp = mRA;
    mRA = mRB;
    mRB = temp;
    mNormalB = -mNormalB;

	/* die contaktpunkte sind im ersten swap (doCollision) noch gar nicht gesetzt worden.
	 * sie werden erst in collisionResponse gesetzt. man muss sie hier aber für den
	 * swap in recheckInterference() vertauschen, damits wieder stimmt.*/
    temp = mContactPointInA;
    mContactPointInA = mContactPointInB;
    mContactPointInB = temp;
}
