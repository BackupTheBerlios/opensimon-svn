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


#ifndef CLOTH_SYSTEM_H
#define CLOTH_SYSTEM_H

#include <map>
#include <simon/SpringConnection.h>
#include <simon/Id.h>
#include <simon/SmartPointer.h>
#include <simon/Geometry.h>
#include <simon/SimonState.h>

class ClothSystem {
public:
    
    ClothSystem();
    
    SpringConnectionPtr createSpringConnection(ParticlePtr, ParticlePtr, Id id);
	SpringConnectionPtr createSpringConnection(ParticlePtr, RigidBodyPtr, Vector3<float>, Id id);
    
    void deleteSpringConnection(Id id);    
    void applyIK(float);

    const std::map<Id, SpringConnectionPtr >*getMap();

    void computeCloth();

private:

    typedef std::map<Id, SpringConnectionPtr> SpringConnectionList;
	SpringConnectionList mSpringConnectionList;
	
};

#endif    // !CLOTH_SYSTEM_H
