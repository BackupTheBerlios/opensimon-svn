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
 * \class ContactInformationContainer
 *
 * \author Trappe
 *
 */
//------------------------------------------------------------------------------

#include "ContactInformationContainer.h"

using namespace std;

ContactInformationContainer::ContactInformationContainer() {

	// Sicherstellen das contacts leer ist.
	mContactInformationContainer = vector<ContactInformation>();
}

ContactInformationContainer::~ContactInformationContainer() {

}

void ContactInformationContainer::addContactInformation(
	const Vec3& position, 
	Id objectA, 
	Id objectB,
	float impulse,
	float grinding){

	ContactInformation contact;
	contact.position = position;
	contact.objectA = objectA;
	contact.objectB = objectB;
	contact.impulse = impulse;
	contact.grinding = grinding;

	mContactInformationContainer.push_back(contact);
}
	

std::vector<ContactInformationContainer::ContactInformation> 
ContactInformationContainer::getContactInformationContainer() {
	return mContactInformationContainer;
}

int ContactInformationContainer::getNumOfContactInformations(){
	return mContactInformationContainer.size();
}

ContactInformationContainer::ContactInformation 
ContactInformationContainer::getContactInformation(int index){
	return mContactInformationContainer[index];
}

void ContactInformationContainer::clear() {
	mContactInformationContainer.clear();
}
	
bool ContactInformationContainer::isEmtpy() {
	if (mContactInformationContainer.empty())
		return true;
	else
		return false;
}
