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


#ifndef PARTICLE_H
#define PARTICLE_H

#include <simon/WorldObject.h>
#include <simon/Id.h>

class Particle : public WorldObject {

public:
	Particle();

	~Particle();

	void setNormal(const Vector3<float>&); // Setzen der Normale als Vektor
	void setIsMoveableFlag(bool);//Setzen ob beweglich
	bool getIsMoveableFlag();
	Vector3<float> const getNormal(); // Gibt die Normale als Vektor zurück
	bool const getIsColliding(); // Gibt an ob eine Kollision erkennat wurde. (nur für Stoff notwendig)
	void setIsColliding(bool flag); // Soll anzeigen ob Stoff Kollidiert ist (nur für Stoff notwendig)
	bool const getIsCollidingBottom();
	void setIsCollidingBottom(bool flag);
	void setIsCollidingWhere(int num);
	int getIsCollidingWhere();

	
private:
	Vector3<float> mNormal;
	bool mIsMoveableFlag;

	//Wird benötigt für Kollision mit Stoff.
	bool mIsColliding;
	bool mIsCollidingBottom;
	int mIsCollidingWhere;
};


#endif
