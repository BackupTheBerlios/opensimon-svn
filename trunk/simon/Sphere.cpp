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
*  \class Sphere
*  \author Kilian, Kipermann, Haeusler
*
*
*  \brief Kugel für Kollisionstest
*
*  
*
*/
//------------------------------------------------------------------------------
#include <simon/Sphere.h>
#include <simon/Plane.h>
#include <simon/Box.h>
#include <simon/Capsule.h>
#include <simon/SimonState.h>
#include <stdio.h>
#include <cassert>

/**
* \brief Constructor
* \param rigidBody RididBody-Objekt, dem die Geometrie gehört
* \param radius Radius der Kugel
*/
Sphere::Sphere(SmartPointer<RigidBody>& rigidBody, float radius)
:Geometry(rigidBody)
{
    mRadius = radius;
	float tElement = 100*(2.0f/5.0f) * mRigidBody->getMass() * mRadius;
	mRigidBody->setInertiaTensor(tElement, tElement, tElement);

}



float Sphere::getArea() {

	const float cw = 3.f/4.f;
	return (float)(M_PI * mRadius * mRadius * cw);
}

