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


#include "Id.h"

#include <cassert>

//------------------------------------------------------------------------------
/**
 * \class Id
 * $Author: alangs $
 * $Date: 2004/12/14 18:22:18 $
 * $Revision: 1.10 $
 * \brief Ermöglicht die Identifikation von Objekten
 *
 *
 *
 */
//------------------------------------------------------------------------------

const char* const Id::mTypeNames[] = {
	"None",
	"Box",
	"Sphere",
	"Cylinder",
	"Capsule",
	"Plane",
	"Thread",
	"Cloth",
	"Hair",
	"Balljoint",
	"Hingejoint",
	"Particle",
	"Connection",
	"Camera",
	"InternalSimon",
	"ControlSphere"
};

const Id Id::mNone(Id::typeNone, 0);

Id::Number Id::mLastNumberUsedNone=0;
Id::Number Id::mLastNumberUsedBox=0;
Id::Number Id::mLastNumberUsedSphere=0;
Id::Number Id::mLastNumberUsedCylinder=0;
Id::Number Id::mLastNumberUsedCapsule=0;
Id::Number Id::mLastNumberUsedPlane=0;
Id::Number Id::mLastNumberUsedThread=0;
Id::Number Id::mLastNumberUsedCloth=0;
Id::Number Id::mLastNumberUsedHair=0;
Id::Number Id::mLastNumberUsedBalljoint=0;
Id::Number Id::mLastNumberUsedHingeJoint=0;
Id::Number Id::mLastNumberUsedParticle=0;
Id::Number Id::mLastNumberUsedConnection=0;
Id::Number Id::mLastNumberUsedCamera=0;
Id::Number Id::mLastNumberUsedInternalSimon=0;
Id::Number Id::mLastNumberUsedControlSphere=0;

/**
 * \brief Default-Construktor
 */
Id::Id(){
	mType = typeNone;
	mNumber = 0;
}

/**
 * \brief ID constructor
 *
 * \param type type to set (see the enum Type)
 * \param number number of the object
*/
Id::Id(const Id::Type type, const unsigned int number) {

	mType = type;
	mNumber = number;
}


/**
 * \brief Returns the type of the object.
 * \return Type of objects.
 */
Id::Type Id::getType() const {
  return mType;
}

/**
* \brief Liefert den Namen des Objekttypes.
* \return Name des Objekttypes.
*/
const char* Id::getTypeName(const Type type){
	return mTypeNames[type];
}

/**
* \brief Setzt den Typ der Id
*/
void Id::setType(const Type type) {
	mType = type;
}

/**
* \brief Setzt die Id Nummer
*/
void Id::setNumber(const unsigned int number) {
	mNumber = number;
}

/**
 * \brief Liefert die Nummer des Objektes.
 * \return Nummer des Objektes.
 */
unsigned int Id::getNumber() const {
  return mNumber;
}

/**
 * \brief Liefert true, wenn beide Ids gleich sind, sonst false
 * \param Ident, Id to compare with
 * \return true wenn gleich Ids, sonst false
 */
bool Id::operator == (const Id& Ident) const {

	return (
		(Ident.getType() == mType) &&
		(Ident.getNumber() == mNumber));
}

bool Id::operator != (const Id& Ident) const {

	return (
		(Ident.getType() != mType) ||
		(Ident.getNumber() != mNumber));
}

bool operator < (const Id& id1, const Id& id2) {
	const Id::Type t1 = id1.getType();
	const Id::Type t2 = id2.getType();

	if(t1 == t2) {
		return id1.getNumber() < id2.getNumber();
	} else {
		return t1 < t2;
	}
}

bool operator > (const Id& id1, const Id& id2) {
	const Id::Type t1 = id1.getType();
	const Id::Type t2 = id2.getType();

	if(t1 == t2) {
		return id1.getNumber() > id2.getNumber();
	} else {
		return t1 > t2;
	}
}

std::ostream& operator <<(std::ostream& os, const Id& id) {

	os << id.mNumber << " (" << Id::mTypeNames[id.mType] << ")";
	return os;
}

Id Id::factor(const Type type) {
	
	switch (type) {
		case Id::typeNone:			return Id(type, ++mLastNumberUsedNone);
		case Id::typeBox:			return Id(type, ++mLastNumberUsedBox);
		case Id::typeSphere:		return Id(type, ++mLastNumberUsedSphere);
		case Id::typeCylinder:		return Id(type, ++mLastNumberUsedCylinder);
		case Id::typeCapsule:		return Id(type, ++mLastNumberUsedCapsule);
		case Id::typePlane:			return Id(type, ++mLastNumberUsedPlane);
		case Id::typeThread:		return Id(type, ++mLastNumberUsedThread);
		case Id::typeCloth:			return Id(type, ++mLastNumberUsedCloth);
		case Id::typeHair:			return Id(type, ++mLastNumberUsedHair);
		case Id::typeBallJoint:		return Id(type, ++mLastNumberUsedBalljoint);
		case Id::typeHingeJoint:	return Id(type, ++mLastNumberUsedHingeJoint);
		case Id::typeParticle:		return Id(type, ++mLastNumberUsedParticle);
		case Id::typeConnection:	return Id(type, ++mLastNumberUsedConnection);
		case Id::typeCamera:		return Id(type, ++mLastNumberUsedCamera);
		case Id::typeInternalSimon:	return Id(type, ++mLastNumberUsedInternalSimon);
		case Id::typeControlSphere:	return Id(type, ++mLastNumberUsedSphere);
		default: assert(false);
	}

	assert(false);
	return Id(Id::typeNone, 0);

}
