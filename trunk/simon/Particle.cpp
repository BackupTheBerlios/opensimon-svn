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


#include "Particle.h"

//------------------------------------------------------------------------------
/**
 * \class Particle
 * \brief Particle-Klasse abgeleitet von WorldObject
 *
 *
 * 
 */
//------------------------------------------------------------------------------


/**
* \brief Default-Konstruktor
*/
Particle::Particle(){
	// Benötigt für Kollision mit Stoff
	mIsColliding = false;
	//mIsCollidingWhere = 1;
}

/**
* \brief Destruktor
*/

Particle::~Particle(){
}

/**
* \brief Setzt die Normale des Patrikels
* 
* \param normal Normale als Vector3<float>
*/
void Particle::setNormal(const Vector3<float>& normal) {
	mNormal = normal;
}

/**
 * \brief Gibt zurück ob das Partikel versetzt werden darf
 */
void Particle::setIsMoveableFlag(bool flag){
	mIsMoveableFlag = flag;
}

/**
 * \brief Setzt ob das Partikel versezt werden darf
 */
bool Particle::getIsMoveableFlag(){
	return mIsMoveableFlag;
}



/**
* \brief Gibt die Normale als Vektor zurück
* \return Vector3<float>
*/
Vector3<float> const Particle::getNormal() {
	return mNormal;
}

/**
 * \brief Setzt die CollidingFlag
*/
void Particle::setIsColliding(bool flag)
{
	mIsColliding = flag;
}

/**
* \breif gibt an ob eine Kollision vorliegt (nur für Stoff notwendig)
*/
bool const Particle::getIsColliding()
{
	return mIsColliding;
}

/**
 * \brief Setzt die CollidingBottomFlag
*/
void Particle::setIsCollidingBottom(bool flag)
{
	mIsCollidingBottom = flag;
}

/**
* \breif gibt an ob eine Kollision am Boden vorliegt (nur für Stoff notwendig)
*/
bool const Particle::getIsCollidingBottom()
{
	return mIsCollidingBottom;
}

void Particle::setIsCollidingWhere(int num){
	mIsCollidingWhere = num;
}

int Particle::getIsCollidingWhere(){
	return mIsCollidingWhere;
}
