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


#ifndef CONTACT_INFORMATION_CONTAINER_H
#define CONTACT_INFORMATION_CONTAINER_H

//------------------------------------------------------------------------------
/**
 * \class ContactInformationContainer
 *
 * \author Trappe
 *
 * 
 * \brief Speichern und Verarbeiten von KollisionsPunkten.
 *
 * Es werden jeweils die Beteiligten Materialien und Positionen der Kollsion
 * abgespeichert  und stehen so bei der Weiterverarbeitung zur Verfügung.
 * Hauptaufgabe: In Simon erkannte Kollisionen zwischen "Sensor"-Objekten
 * speichern und an Garfunkel weiterleiten.
 * 
 */
//------------------------------------------------------------------------------

#include <vector>
#include "Vector3.h"
#include "Id.h"

class ContactInformationContainer {

public:

	struct ContactInformation {
		Vec3 position;
		Id objectA;
		Id objectB;
		float impulse; // Stärke der Kollision
		float grinding; // Reibung
	};

	ContactInformationContainer();
	~ContactInformationContainer();

	//! Merken der Position und der am Kontakt beteiligten Materialien, sowie de
	//! Stärke des Zusammenstoßes und der dabei entstandenen Reibung.
	void addContactInformation(
		const Vec3& position, 
		Id objectA, 
		Id objectB,
		float implulse,
		float grinding);
	
	//! Entfernt alle gespeicherten Kontakt-Informationen.
	void clear();

	//! Testet ob etwas gespeicht wurde.
	bool isEmtpy();

	int getNumOfContactInformations();
	ContactInformation getContactInformation(int);

	//! wird zum kopieren der Daten in das ContactsNotificationEvent benötigt
	std::vector<ContactInformation> getContactInformationContainer();

private:
		
	//! stores all contacts
	std::vector<ContactInformation> mContactInformationContainer;
};

#endif // !CONTACT_INFORMATION_H
