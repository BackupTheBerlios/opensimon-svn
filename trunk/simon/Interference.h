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


#ifndef INTERFERENCE_H
#define INTERFERENCE_H

//! \todo In SimonState speichern.
//* \brief define threshhold for contacts to be regarded resting *//
#define ENTERING_RESTING_CONTACT_THRESHHOLD 0.01
//* \brief define threshhold for resting contacts to seperate *//
#define LEAVING_RESTING_CONTACT_THRESHHOLD 0.1

#include <simon/Vector3.h>
#include <simon/SmartPointer.h>
#include <simon/ContactPoint.h>
#include <simon/Pool.h>

#include <vector>

class Geometry;

/**
 * \class Interference
 * \file Interference.h
 *
 * \brief Stores informationons about the interference beween two rigid bodies.
 *
 */
class Interference{

public:
	Interference();	
	Interference(GeometryPtr objectA, GeometryPtr objectB);
	
	GeometryPtr getObjectA();
	GeometryPtr getObjectB();
	
	void setObjectA(GeometryPtr a);
	void setObjectB(GeometryPtr b);

	// Swap objects A and B. We need this to find out for which object findInterference was called in the first place
	void swap();
	
	const bool& hasBeenSwapped() const;	

	void addContactPoint(Vec3 worldPosition, Vec3 normalB);
	void addContactPoint(Vec3 worldPosition, Vec3 normalB, float distanceFromB);

	//! gives access to a single contactpoint
	ContactPoint& getContactPoint(int index);

	//! tells how many contact points are stored
	int getNumOfContacts();

	//Überladenen << Operator, der information über die interference ausgibt
	friend std::ostream& operator <<(std::ostream& os, Interference& interference);

protected:
	GeometryPtr mObjectA;
	GeometryPtr mObjectB;

	std::vector< ContactPoint > mContactPoints;
	
	// Liegt der Kontaktpunkt (auf A + in B)=false oder (auf B und in A)=true
	// er liegt normalerweise auf A. wenn das objekt aber geswappt wurde
	// liegt er auf B, weil B ja dann A ist.
	bool mHasBeenSwapped;	

};
	
	
#endif
