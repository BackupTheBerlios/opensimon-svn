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


#ifndef ID_H
#define ID_H

#include <iostream>

/**
 * \class Id
 *
 * \brief Provides identification for all physical objects of Simon.
 *
 *
 * This is a combianted identification. First by a type, second with 
 * a number.
 * Only the combination of both is unic. So there is could be a 
 * id (sphere, 1) and a (box, 1).
 */
class Id {

	public:	

		/// defines the different types of physical objects
		enum Type {
			typeNone = 0,
			typeBox,
			typeSphere,
			typeCylinder,
			typeCapsule,
			typePlane,
			typeThread,
			typeCloth,
			typeHair,
			typeBallJoint,
			typeHingeJoint,
			typeParticle,
			typeConnection,
			typeCamera,
			typeInternalSimon,
			typeControlSphere
		};
		
		// i think, there is no need for in simon anymore. (trappe)
		typedef unsigned int Number;

		static const Id mNone;

		Id();
		Id(const Type type, const Number number);

		void setType(const Type type);
		Type getType(void) const;
		static const char* getTypeName(const Type type);
		Number getNumber(void) const;
		void setNumber(const Number number);
		bool operator==(const Id&) const;
		bool operator!=(const Id&) const;
		/// Kleiner-als-Operator, lexikografische Ordnung auf Typ-Nummer-Tupel.
		/// Wird z.B. für das Einfügen in eine std::map benötigt.
		friend bool operator<(const Id& id1, const Id& id2);
		friend bool operator>(const Id& id1, const Id& id2);
		friend std::ostream& operator <<(std::ostream& os, const Id& id);
		// liefert eine fortlaufende eindeutige Id
		static Id factor(const Type type=typeNone);

	private:
		// Typenbezeichnung 
		Type mType;
		// Fortlaufende Nummer der Objekte
		Number mNumber;
		// Array mit Namen der Typen als Strings
		static const char* const mTypeNames[];
		// Letzte vergeben fortlaufenden Nummer
		static Number mLastNumberUsedNone;
		static Number mLastNumberUsedBox;
		static Number mLastNumberUsedSphere;
		static Number mLastNumberUsedCylinder;
		static Number mLastNumberUsedCapsule;
		static Number mLastNumberUsedPlane;
		static Number mLastNumberUsedThread;
		static Number mLastNumberUsedCloth;
		static Number mLastNumberUsedHair;
		static Number mLastNumberUsedBalljoint;
		static Number mLastNumberUsedHingeJoint;
		static Number mLastNumberUsedParticle;
		static Number mLastNumberUsedConnection;
		static Number mLastNumberUsedCamera;
		static Number mLastNumberUsedInternalSimon;
		static Number mLastNumberUsedControlSphere;
};

#endif
