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
*  \class Plane
*  \author Kilian, Kipermann, Haeusler
*
*  \brief Eben für Kollisionstest
*
*/
//------------------------------------------------------------------------------

#include "Plane.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Box.h"
#include <assert.h>

#include <math.h>

Vec3 Plane::mStandardNormal = Vec3(0,1,0); 

/**
* \brief Default-Konstruktor
*/
Plane::Plane( RigidBodyPtr& rigidBody)
:Geometry(rigidBody){

}


/**
 * \see normal
 *
 * Die standard Normale wird um die Orientierung des zugehörigen 
 * Festkörpers gedreht und zurück geliefert.
 */
Vec3 Plane::getNormal() {
    return qRotate(mStandardNormal, mRigidBody->getOrientation());
}

Plane::~Plane(){

}

/**
* \brief liefert die Querschnittsfläche
*/
float Plane::getArea()
{
	return 0.0;
}
