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


#include "Geometry.h"

#include <assert.h>
//------------------------------------------------------------------------------
/**
 * \class Geometry
 *
 * \author Kipermann & Häusler
 *
 * \brief Geometry Klasse für Kollisionsberechnungen.
 *        Basisklasse für Sphere, Box, Plane etc.
 * 
 */
//------------------------------------------------------------------------------

/**
 * Default Werte:
 * - Bounciness (Hüpffaktor) = 0.5
 * - Friction (Reibung) = 0.9
 * - Visibility = true
 * - Color (0.5, 0.5, 0.5)
 */
Geometry::Geometry(RigidBodyPtr& rigidBody) : 
	mRigidBody(rigidBody), 
	mBounciness(0.5f), 
	mFriction(0.6f){

	assert(rigidBody);
}

void Geometry::setBounciness(float bounciness){
    if (bounciness>=0 && bounciness<=1)
        mBounciness=bounciness;
};

void Geometry::setFriction(float friction){
    if (friction>=0 && friction<=1)
        mFriction=friction; 
};

float Geometry::getBounciness() { 
	return mBounciness; 
}    
