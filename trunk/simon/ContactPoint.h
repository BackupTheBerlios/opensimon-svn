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


#include "Vector3.h"

class ContactPoint {
public:

	ContactPoint();
  	ContactPoint(
		const Vec3& worldPosition,
		const Vec3& normalB,
		const Vec3& rA,
		const Vec3& rB);	
  	ContactPoint(
		const Vec3& worldPosition,
		const Vec3& normalB,
		const Vec3& rA,
		const Vec3& rB,
		float distanceFromB);
  	
	Vec3 getPosition() const;	
	Vec3 getNormalA() const;
	const Vec3& getNormalB() const;
	const Vec3& getRA() const;
	const Vec3& getRB() const;
	const Vec3& getContactPointInA() const;  
	const Vec3& getContactPointInB() const;  	
	const float& getDistanceFromB() const;
	const bool& isDistanceFromBSet() const;
	
	const float& getImpulseMagnitude() const;
	const float& getGrindMagnitude() const; 
	void setImpulseMagnitude(float impulseMagnitude);
	void setGrindMagnitude(float grindMagnitude);
	void setContactPointInA(Vec3& contactPointInA); 
	void setContactPointInB(Vec3& contactPointInB);	  
	void setDistanceFromB(float distanceFromB);

	// Swap objects A and B. We need this to find out for which object findInterference was called in the first place
	void swap();  
protected:

  	Vec3 mWorldPosition;
	Vec3 mNormalB;
	// Abstand von contactPoint zu den Mittelpunkten nach Vorberechnung.
    Vec3 mRA;
	Vec3 mRB;
	
	// abstand des ContactPunktes von B vor Behandlung (=Eindringtiefe)
	// WARNUNG: B ist hier immer das Originale B, d.h. ein evt. swap() muss nochmal
	// ausgeführt werden, damit es wieder konsistent ist.
	float mDistanceFromB;
	// wurde der abstand gesetzt ?
	bool mDistanceFromBProvided;	


	//ContactPunkt vor Behandlung in lokalen Koordinaten von A
    Vec3 mContactPointInA;
  //ContactPunkt vor Behandlung in lokalen Koordinaten von A    
    Vec3 mContactPointInB;    
    
    // stärke des Impulses, der in der Behandlung berechnet und gesetzt wird.
	float mImpulseMagnitude;
    // stärke der Reibung, die in der Behandlung berechnet und gesetzt wird.	
	float mGrindMagnitude;	
};
