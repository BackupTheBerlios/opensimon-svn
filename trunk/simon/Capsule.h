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
*  \file Capsule.h
*  \class Capsule
*
*  Capsules stehen, d.h. die "Hauptachse" ist entlang der Y-Achse
*/
//------------------------------------------------------------------------------

#ifndef CAPSULE_H
#define CAPSULE_H

#include <simon/Geometry.h>

class Capsule;
class Plane;
class Sphere; 


class Capsule : public Geometry
{
public:
    Capsule(SmartPointer<RigidBody>& rigidBody, float radius, float height);
    virtual ~Capsule() {};
    
    void setHeight(float height) { mHeight = height; };
    void setRadius(float radius) { mRadius = radius; };
    
    float getHeight() const { return mHeight; };
    float getRadius() const { return mRadius; };
    
	virtual float getArea();


protected:
    float mRadius;
    float mHeight;
};

#endif
  
  
  
  
