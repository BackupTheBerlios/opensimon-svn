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


// $Id: Quaternion.h,v 1.17 2004/12/15 17:04:49 trappe Exp $
//------------------------------------------------------------------------------
/**
*  \file Quaternion.h
*  \class Quaternion
*/
//------------------------------------------------------------------------------

#ifndef QUATERNION_H
#define QUATERNION_H

#include <simon/Matrix.h>
#include <simon/Vector3.h>

#include <simon/config.h>

#define	DEGTORAD(x)	( ((x) * M_PI) / 180.0f )
#define	RADTODEG(x)	( ((x) * 180.0f) / M_PI )

//--- Class definition ---------------------------------------------------------

class Quaternion {


public:		
	static const Quaternion mIdentity;

	// Konstruktoren
	Quaternion();
	Quaternion(float angle, Vector3<float> axis);
	Quaternion(Vector3<float> v);
	Quaternion(float phix, float phiy, float phiz);
	Quaternion(Matrix3x3& m);

	// Destruktor
	~Quaternion();

	Quaternion& operator +=(const Quaternion& rhs);
	Quaternion& operator *=(const Quaternion& rhs);
		
	void identity();
	Quaternion inverse();
	void invert();
	float normalize();
	void normalizeAxis();
		
	void setValues(float w, float x, float y, float z);
	void setValues(float angle, Vec3& axis);
	void setValues(float x, float y, float z); 
		
	void getValues(float& w, float& x, float& y, float& z) const;

	void setVector(const Vector3<float>& v);
	Vector3<float> getVector() const;

	Vector3<float> getEulerRotation() const;
	void getEulerRotation(Vec3 &euler) const;
	Matrix3x3 getRotationMatrix() const;
	void getRotationMatrix(Matrix3x3 &rotation) const;
	void getAxisAngle(Vector3<float>& axis, float& angle) const;

	//! \return The axis of rotation
	Vec3 getAxis() const;
	//! \return The angle of rotation in radian
	float getAngle() const;

	void print();

private:

	Quaternion(float w, float x, float y, float z);

	float mqElement[4];

	float& operator [](unsigned int index);
	const float& operator [](unsigned int index) const;

	float w() const;
	float x() const;
	float y() const;
	float z() const;
	 
		
	//--- Related functions --------------------------------------------------------
	friend const Quaternion operator +(const Quaternion&, const Quaternion&);
	friend const Quaternion operator *(const Quaternion&, const Quaternion&);
	friend const Quaternion operator *(const float, const Quaternion&);
	friend const Quaternion operator *(const Quaternion&, const float);
	friend float dot(const Quaternion&, const Quaternion&);
	friend const Vector3<float> qRotate(const Vector3<float>& vector, const Quaternion& quat); 
	friend std::ostream& operator << (std::ostream& os, const Quaternion& quaternion);

};

#endif
