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

#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <simon/WorldObject.h>
#include <simon/Quaternion.h>
#include <simon/Matrix.h>

#include <cstdio>
#include <vector>

//! \todo Zu debugginzwecken, bis simonsState verwendet wird!
#include <iostream>

class Plane;
class Sphere;
class RigidBody : public WorldObject {
	

public:

	RigidBody();
	~RigidBody();


	//SETTER
		
	void setOrientation(Quaternion); 			//Setzt die Orientierung als Quaternion
	void setPrevOrientation(Quaternion); 			//Setzt die vorherige Orientierung als Quaternion
	void undoSetOrientation(); 				// macht den letzten schritt Rückgängig
	void setTorque(Vec3); 			//Setzt das Drehmoment
	void setAngularVelocity(Vec3); 		//Setzt die angulaere Geschwindigkeit
	void setPrevAngularVelocity(Vec3); 		//Setzt die vorherige angulaere Geschwindigkeit
	void undoSetAngularVelocity(); 				//Macht den Aufruf von setAngularVelocity rueckgaengig
	void setAngularMomentum(Vec3); 		//Setzt das angulaere Moment
	void setPrevAngularMomentum(Vec3);		//Setzt das vorherige angulaere Moment
	void undoSetAngularMomentum(); 				//Macht den Aufruf von setAngularMomentum rueckgaengig
	void setAngularAcceleration(Vec3); 		//setzt die angulaere Beschleunigung
	void setPrevAngularAcceleration(Vec3);	//setzt die vorherige angulaere Beschleunigung
	void setInertiaTensor(const Matrix3x1); 		//Setzt den InertiaTensor in Koerperkoordinaten als Matrix
	void setInertiaTensor(float, float, float); 		//Setzt den InertiaTensor in Koerperkoordinaten als drei float
	
	
	//GETTER

	Quaternion getOrientation() const;			//Gibt die Orientierung als Quaternion zurueck
	void getOrientation(Quaternion& ori) const;			//Gibt die Orientierung als Quaternion zurueck
	Quaternion getPrevOrientation() const;			//Gibt die vorherige Orientierung als Quaternion zurueck
	Vec3 getTorque() const;			//Gibt den Drehmoment als Vektor zurueck
	void getTorque(Vec3 &torque) const;			//Gibt den Drehmoment als Vektor zurueck
	void getTorque (Matrix3x1 & torque) const;          //Gibt den Drehmoment als Matrix zurueck
	Vec3 getAngularVelocity() const;		//Gibt die angulaere Geschwindigkeit als Vektor zurueck
	void getAngularVelocity(Vec3 &aVelo) const;		//Gibt die angulaere Geschwindigkeit als Vektor zurueck
	void getAngularVelocity(Matrix3x1 &aVelo) const;		//Gibt die angulaere Geschwindigkeit als Vektor zurueck
	Vec3 getPrevAngularVelocity() const;		//Gibt die vorherige angulaere Geschwindigkeit als Vektor zurueck
	Vec3 getAngularMomentum() const;		//Gibt das angulaere Moment als Vektor zurueck
	void getAngularMomentum(Vec3 &angularMomentum) const;		//Gibt das angulaere Moment als Vektor zurueck
	Vec3 getPrevAngularMomentum() const;		//Gibt das vorherige angulaere Moment als Vektor zurueck
	Vec3 getAngularAcceleration() const;		//Gibt die angulaere Beschleunigung als Vektor zurueck
	Vec3 getPrevAngularAcceleration() const;	//Gibt die vorherige angulaere Beschleunigung als Vektor zurueck
	Matrix3x3 getInertiaTensor() const;			//Gibt den Inertia Tensor in Koerperkoordinaten als Matrix zurueck
	Matrix3x3 getWorldInertiaTensor() const;		//Gibt den Inertia Tensor in Weltkoordinaten als Matrix zurueck
	Matrix3x3 getInvInertiaTensor() const;		//Gibt den inversen Inertia Tensor in Koerperkoordinaten als Matrix zurueck
	Matrix3x3 getInvWorldInertiaTensor () const;	//Gibt den inversen Inertia Tensor in Weltkoordinaten als Matrix zurueck
	void getInvWorldInertiaTensor (Matrix3x3 &invWorldTensor) const; // Gibt den inversen Inertia Tensor in Weltkoordinaten als Matrix zurueck
	
	
	//OTHER
	void backupVelocities();
	void restoreVelocities();

	void backupPositions();
	void restorePositions();
	
	void addTorque(Vec3); 			//addiert ein Drehmoment hinzu
	void processConstraints(Matrix6x1);			//Verarbeitet Aufruf von den Gelenken
	void processConstraintPostStabilization (Matrix6x1);//Verarbeitet die Poststabilisation der Gelenke
	void computeForceAndTorque(Matrix6x1);		//Berechnet aus einer 6x1-Matrix (0-2 -> Angriffspunkt in Weltkoordinaten; 3-5 -> Force) den Drehmoment
	
	
	
	//INTEGRATOREN
	
	//! Integration mit dem in SimonState.getIntegrator() definierten Integrator 
	void integrate(float, bool, float, float, bool absorbForce = true);
	//! Integration mit dem in SimonState.getIntegrator() definierten
	//! Integrator (nur die neue lineare und angulaere Geschwindigkeit)
	void integrateVelocities(float);
	//! Integration mit dem in SimonState.getIntegrator() definierten
	//! Integrator (nur neue Orientierung und Position)
	void integratePositions(float, bool, float, float, bool absorbForce = true);
	
	
	//EULER
	//! Integration mit Euler	
	void integrateEuler(float, bool, float, float, bool absorbForce = true);
	
	//! Integration mit Euler (nur die neue lineare und angulaere Geschwindigkeit)
	void integrateEulerVelocities(float);
	
	//! Integration mit Euler (nur neue Orientierung und Position)
	void integrateEulerPositions(float, bool, float, float, bool absorbForce = true);
	
private:

	/** Orientierung */
	Quaternion mOrientation;

	/** Vorherige Orientierung */
	Quaternion mPrevOrientation;
		
	/** angulaeres Moment */
	Vec3 mAngularMomentum; 
	
	/** vorheriges angulaeres Moment */
	Vec3 mPrevAngularMomentum; 

	/** Drehmoment */
	Vec3 mTorque;

	/** Vorheriges Drehmoment */
	Vec3 mPrevTorque;

	/** Winkelgeschwindigkeit */
	Vec3 mAngularVelocity;
	
	/** Vorherige Winkelgeschwindigkeit */
	Vec3 mPrevAngularVelocity;

	/** Winkelbeschleunigung */
	Vec3 mAngularAcceleration;

	/** Vorherige Winkelbeschleunigung */
	Vec3 mPrevAngularAcceleration;

	/** inverser Trägheitstensor in Body-Koordinaten */
	Matrix3x3 mInvInertiaTensor;
	
	
	/** Variablen fuer Integratoren */
	Vec3 mIntegratorForce;
	Vec3 mIntegratorTorque;
	Vec3 mIntegratorPosition;
	Vec3 mIntegratorOldLinearMomentum;
	Vec3 mIntegratorNewLinearMomentum;
	Vec3 mIntegratorOldAngularMomentum;
	Vec3 mIntegratorNewAngularMomentum;
	Vec3 mIntegratorOldAngularVelocity;
	
	Quaternion mIntegratorOldOrientation;
	Quaternion mIntegratorONewOrientation;
	Quaternion mIntegratorQuaternionVelocity;
	Quaternion mIntegratorQuaternionAbbreviation;

};


#endif // !RIGID_BODY_H
