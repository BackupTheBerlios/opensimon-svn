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


#include "Connection.h"
#include "WorldObject.h"
#include "stdlib.h"

//------------------------------------------------------------------------------
/**
 * \class Connection
 *
 * \brief Abstrakte Klasse für alle Verbindungen zwischen zwei Objekten
 *
 *
 * 
 */
//------------------------------------------------------------------------------


/**
 * \brief Default-Konstruktor setzt die beiden Objekt Pointer auf Null
 *  
 */
Connection::Connection() {
	mObjectA = WorldObjectPtr();
	mObjectB = WorldObjectPtr();
}

/**
 * \brief Konstruktor mit Übergabe der verbundenen Objekte
 * \param objectA Das erste WorldObject der Verbindung
 * \param objectB Das zweite WorldObject der Verbindung
 * 
 * Diesem Konstruktor können direkt die zu verbindenden Objekte mitgegeben werden.
 */
Connection::Connection(WorldObjectPtr objectA, WorldObjectPtr objectB) {
	mObjectA = objectA;
	mObjectB = objectB;
}

/**
 * \brief Destruktor
 */
Connection::~Connection() {

}

/**
* \brief Gibt ObjectA zurück
*/

WorldObjectPtr Connection :: getObjectA () {
	assert(mObjectA);
	return mObjectA;
}

/**
* \brief Gibt ObjectB zurück
*/

WorldObjectPtr Connection :: getObjectB () {
	assert(mObjectB);
	return mObjectB;
}

/**
* \brief Gibt Id zurück
*/
Id const Connection :: getId () {
	return mId;
}

/**
* \brief Setzt ObjectA
* \param objectA Das erste WorldObject der Verbindung
*/
void Connection::setObjectA (WorldObjectPtr objectA){
	assert(objectA);
	mObjectA = objectA;	

}

/**
* \brief Setzt ObjectB
* \param objectB Das zweite WorldObject der Verbindung
*/
void Connection::setObjectB (WorldObjectPtr objectB){
	assert(objectB);
	mObjectB = objectB;	

}

/**
* \brief Setzt eine Id
* \param id ein Objekt der Klasse Id. Wird zur identifikation gebraucht
*/
void Connection::setId(Id id) {
  mId = id;
}

