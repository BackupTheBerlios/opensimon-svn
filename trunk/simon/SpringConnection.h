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


#ifndef SPRINGCONNECTION_H
#define SPRINGCONNECTION_H

#include "Connection.h"
#include <vector>
#include "Particle.h" 
using namespace std;
// $Id: SpringConnection.h,v 1.17 2004/10/05 17:09:16 trappe Exp $

class SpringConnection : public Connection {

public:
        
        
        
        SpringConnection();
        
        SpringConnection(ParticlePtr, ParticlePtr);
		SpringConnection(ParticlePtr, ParticlePtr, float);
		SpringConnection(ParticlePtr, RigidBodyPtr, Vec3);
        ~SpringConnection();
        
        
		float getSpringConst();
		void setSpringConst(float);
		float getRestLength();
		void setRestLength(float);
		float getDampConst();
		void setDampConst(float);
		float getLength();
		void updateLength();
		float getNormalSpringLength();
		float getMaxSpringLength();

private:

		//! \todo warum diese Membervariablen? Es gibt doch schon Pointer auf die beiden Particle (siehe Connection.h)
		WorldObjectPtr mBegin;
		WorldObjectPtr mEnd;
	
		float mlength;
		float mNormalSpringLength;
		float mMaxSpringLength;
		float mSpringConst;
		float mRestLength;
		float mDamping;

};


#endif
