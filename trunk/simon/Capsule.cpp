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
 * \class Capsule
 * \author Kilian, Kipermann, Haeusler
 *
 * \brief Kapsel für KollisionTest
 *
 */
//------------------------------------------------------------------------------

#include <simon/Capsule.h>
#include <simon/Sphere.h>
#include <simon/Plane.h>
#include <simon/Box.h>

#include <simon/Quaternion.h>

#include <math.h>
#include <stdio.h>
#include <iostream>


/**
* \brief constructor
* \param radius 
* \param height 
*/

using namespace std;

Capsule::Capsule(RigidBodyPtr& rigidBody, float radius, float height)
	:Geometry(rigidBody), 
	 mRadius(radius), 
	 mHeight(height){

    float mass = mRigidBody->getMass();
	float tElement1 = 100*((1.0f/12.0f) * mass * mHeight * mHeight + 0.25f * mass * mRadius * mRadius);
	float tElement2 = 100*(0.5f * mass * mRadius * mRadius);
	mRigidBody->setInertiaTensor(tElement1, tElement1, tElement2);
}


/**
* \brief liefert die Querschnittsfläche
*/
float Capsule::getArea()
{
	float cw = 0.7f;
	
	Quaternion orgOrientation = mRigidBody->getOrientation();
	orgOrientation.normalize();
	
	Vec3 velocity = mRigidBody->getVelocity();

	velocity = qRotate(velocity, orgOrientation);
	
	float angle;
	Vec3 axis;
	
	if (velocity.length() > 0.01)	
		angle = acosf(dot(velocity, Vec3(0.0, 1.0, 0.0)) / velocity.length());
	else
		return 0.0;

	
	if (velocity[2] == 0)
		axis = Vec3(0.0, 0.0, 1.0);
	else if (velocity[0] == 0)
		axis = Vec3(1.0, 0.0, 0.0);
	else
		axis = Vec3(velocity[2] / velocity[0], 0.0, 1.0);
	
	Quaternion orientation;
	orientation.setValues(angle, axis);
	orientation.normalize();
	
	velocity = qRotate(velocity, orientation);
	
	Vec3 vertex(0.0,  mHeight, 0.0);
	vertex = qRotate(vertex, orientation);
	
	//Verbindungslinie zwischen vertex und  punktgespiegeltem vertex
	Vec3 link(vertex[0] + vertex[0], 0.0, vertex[2] + vertex[2]);
	
	float len = link.length();
	
	// annaehernd ein Kreis
	if (len < mRadius)
	{
		cw = 3.f/4.f;
		return (float)(M_PI * mRadius * mRadius * cw);
	}
	
	
	return cw * len * 2.f * mRadius;
}
