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



#ifndef WORLD_OBJECT_H
#define WORLD_OBJECT_H

#include "../simon/Vector3.h"
#include "../simon/Id.h"

/**
 * \file WorldObject.h 
 * \class WorldObject
 *
 * \brief Oberklasse f�r alle Objekte in der Szene die f�r das PhysikSystem relevant sind
 *
 * Bitte beim Setzen der Kr�fte und Geschwindigkeiten beachten, dass nicht zu gro�e Werte
 * genommen werden. Immerhin ist unsere Schwerkraft auch nur 0,00981.
 */
class WorldObject {
  

public:

	WorldObject();

	virtual ~WorldObject();

	//@{
	//! setter
	void setId(Id); // Setzen der ID
	void setMass(float); // Setzen der Masse
	void setInvMass(float); // Setzen der inversen Masse
	void setForce(Vec3); // Setzen der Kraft als Vector
	void setPosition(Vec3); //Setzen der Position als Vektor
	void setPrevPosition(Vec3);  // Setzen der vorherigen Position
	void setVelocity(Vec3); //Setzen der Geschwindigkeit als Vektor
	void setPrevVelocity(Vec3); //Setzen der vorherigen Geschwindigkeit als Vektor
	void setLinearMomentum(Vec3); //Setzen des linearen Momentes als Vektor
	void setAcceleration(Vec3); // Setzen der Beschleunigung als Vector
	void setPrevAcceleration (Vec3);
	void setIsDynamicFlag(bool flag);	// Soll sich das Objekt bewegen,
	//@}

	//@{
	//! getter
	Id const getId() const; // Gibt die ID zur�ck
	float const getMass(); // Gibt die Masse zur�ck
	float const getInvMass(); // Gibt die Masse zur�ck	
	Vec3 const getForce() const; // Gibt die Kraft als Vektor zur�ck
	void getForce(Vec3 &force) const; // Gibt die Kraft als Vektor zur�ck
	Vec3 const getPosition() const; //Gibt Position als Vektor zur�ck
	void getPosition(Vec3& pos) const; //Gibt Position als Vektor zur�ck
	Vec3 const getVelocity() const; //Gibt Geschwindigkeit als Vektor zur�ck
	void getVelocity(Vec3 &velo) const; //Gibt Geschwindigkeit als Vektor zur�ck
	Vec3 const getLinearMomentum() const; //Gibt Geschwindigkeit als Vektor zur�ck
	void getLinearMomentum(Vec3 &linearMomentum) const; //Gibt Geschwindigkeit als Vektor zur�ck
	Vec3 const getAcceleration() const; // Gibt die Beschleunigung als Vektor zur�ck
	bool const getIsDynamicFlag() const; // gibt den aktuellen Status zur Bewegungsfreiheit.
	bool const getIsFrozen(); //gibt an ob eingefroren oder nicht...
	//@}
	
	// OTHER
	void freeze();	// vor�bergehendes einfrieren, wird f�r Shock Propagation gebraucht
	void unfreeze();	// ...und wieder ausfrieren.
	
	void addForce(Vec3); // eine weitere Kraft hinzuf�gen


protected:
				
	Id mId;
	Vec3 mPosition;
	Vec3 mPrevPosition;
	Vec3 mVelocity;
	Vec3 mPrevVelocity;
	Vec3 mLinearMomentum;
	Vec3 mPrevLinearMomentum;
	
	//! Hier speichern wir, ob sich das Objekt bewegen darf.
    bool mIsDynamic;
	// Wenn das objekt freezed ist gibt es als inverse Masse 0.0 zur�ck
	bool mIsFrozen;

	
	float mInvMass;
	Vec3 mAcceleration;
	Vec3 mPrevAcceleration;
	Vec3 mForce;
	Vec3 mPrevForce;

	Vec3 const getPrevLinearMomentum() const; //Gibt vorherige Geschwindigkeit als Vektor zur�ck

private:

	void undoSetPosition();	
	Vec3 const getPrevPosition() const; // Gibt die vorherige Position zur�ck

	void undoSetLinearMomentum();

	void undoSetVelocity();
	void setPrevLinearMomentum(Vec3); //Setzen der vorherigen Geschwindigkeit als Vektor
	Vec3 const getPrevAcceleration () const;

	Vec3 const getPrevVelocity() const; //Gibt vorherige Geschwindigkeit als Vektor zur�ck
};

#endif
