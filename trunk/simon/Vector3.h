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
* \file Vector3.h
* \class Vector3
* \brief Template für Vector Klasse.
*/
//------------------------------------------------------------------------------

#ifndef VECTOR3_H
#define VECTOR3_H

#include <cassert>
#include <ostream>
#include <cmath>

// forward decleration of Matrix to avoid cicrcular dependency
template<class Element, unsigned int Rows, unsigned int Columns> class Matrix;
typedef Matrix<float, 3, 1> Matrix3x1;

//--- Constants ----------------------------------------------------------------

const unsigned int X = 0;
const unsigned int Y = 1;
const unsigned int Z = 2;

//--- Class definition ---------------------------------------------------------

template<class Type=float> class Vector3
{
	public:

	static Vector3 nullVector;

	/**
	 * \brief Constructor ohne Parametern, initialisiert Vektor mit 0,0,0
	 **/
	Vector3() {
		mElements[X] = 0;
		mElements[Y] = 0;
		mElements[Z] = 0;
	}

	/**
	 * \brief Constructor mit 3 Parametern
	 * \param x Parameter X
	 * \param y Parameter Y
	 * \param z Parameter Z
	 **/
	Vector3(const Type x, const Type y, const Type z) {

		mElements[X] = x;
		mElements[Y] = y;
		mElements[Z] = z;
	}      

	/**
	 * \brief Constructor mit 3 Parametern
	 * \param x Parameter X
	 * \param y Parameter Y
	 * \param z Parameter Z
	 **/
	Vector3(Matrix3x1 matrix) {

		mElements[X] = matrix[0][0];
		mElements[Y] = matrix[1][0];
		mElements[Z] = matrix[2][0];
	}      

	/**
	 * \brief Constructor mit Polarkoordinaten
	 * \param theta Der Winkel Theta
	 * \param phi   Der Winkel Theta  
	 **/
	Vector3(const Type theta, const Type phi) {

		mElements[X] = sin(theta) * cos(phi);
		mElements[Y] = cos(theta);
		mElements[Z] = sin(theta) * sin(phi);
	}

	Type& getX() {
		return mElements[X];
	}

	Type& getY() {
		return mElements[Y];
	}

	Type& getZ() {
		return mElements[Z];
	}

	/** 
	 * \brief Array-Zugriff
	 * \param index index in das Array ( X/ Y / Z)
	 * \return Wert des Vectors[index]
	 **/
	Type& operator [](const unsigned int index) {

		assert( (index==X) || (index==Y) || (index==Z) );
		return mElements[index];
	}

	const Type& operator [](const unsigned int index) const {

		assert( (index==X) || (index==Y) || (index==Z) );
		return mElements[index];
	}


	const Vector3<Type> operator +() const {

		return *this;
	}

	const Vector3<Type> operator -() const {
			
		return Vector3(-mElements[X], -mElements[Y], -mElements[Z]);
	}


	Vector3<Type>& operator +=(const Vector3& rhs) {

		mElements[X] += rhs.mElements[X];
		mElements[Y] += rhs.mElements[Y];
		mElements[Z] += rhs.mElements[Z];

		return *this;
	}


	Vector3<Type>& operator -=(const Vector3& rhs) {

		mElements[X] -= rhs.mElements[X];
		mElements[Y] -= rhs.mElements[Y];
		mElements[Z] -= rhs.mElements[Z];

		return *this;
	}

	Vector3<Type>& operator *=(Type rhs) {

		mElements[X] *= rhs;
		mElements[Y] *= rhs;
		mElements[Z] *= rhs;

		return *this;
	}

	Vector3<Type>& operator /=(Type rhs) {

		mElements[X] /= rhs;
		mElements[Y] /= rhs;
		mElements[Z] /= rhs;

		return *this;
	}        

	//! assign a value to all elements of the vector
	Vector3<Type>& operator =(const Type rhs) {

		mElements[X] = rhs;
		mElements[Y] = rhs;
		mElements[Z] = rhs;

		return *this;
	}        

	Type sqrLength() const {

		return
		mElements[X] * mElements[X] +
		mElements[Y] * mElements[Y] +
		mElements[Z] * mElements[Z];
	}

	Type length() const	{

		return sqrt(
			mElements[X] * mElements[X] +
			mElements[Y] * mElements[Y] +
			mElements[Z] * mElements[Z]);
	}

	void normalize() {

		//! \todo Profile against multiplication by inverse.
		const float scale = length();

		assert(scale != 0);

		mElements[X] /= scale;
		mElements[Y] /= scale;
		mElements[Z] /= scale;
	}
	
	// return the maximum of the values;
	Type max() const
	{
		Type max = mElements[X];
		if (mElements[Y]>max) max=mElements[Y];
		if (mElements[Z]>max) max=mElements[Z];		
		return max;
	}	

	// return the minimum of the values;
	Type min() const
	{
		Type min = mElements[X];
		if (mElements[Y]<min) min=mElements[Y];
		if (mElements[Z]<min) min=mElements[Z];		
		return min;
	}	

	//! Yes, its a Template, but we need to paint, so look into the cpp file
	void draw();

	//Überladenen << Operator, der die drei komponenten des Vektor ausgibt
	friend std::ostream& operator <<(std::ostream& os,const Vector3<Type>& vector3) {
		os << vector3[X] << ", " << vector3[Y] << ", " << vector3[Z];
		return os;
	}

	private:
	Type mElements[3];
};


//--- Related functions --------------------------------------------------------

template <class Type> 
	const bool operator ==(
	const Vector3<Type>& lhs,
	const Vector3<Type>& rhs) {

	if (lhs[X] == rhs[X])
		if (lhs[Y] == rhs[Y])
			if (lhs[Z] == rhs[Z])
				return true;
	return false;
}

template <class Type> 
	const bool operator !=(
	const Vector3<Type>& lhs,
	const Vector3<Type>& rhs) {

	return !(lhs == rhs);
}

template <class Type> 
	const Vector3<Type> operator +(
	const Vector3<Type>& lhs,
	const Vector3<Type>& rhs) {

	return Vector3<Type>(lhs) += rhs;
}

template <class Type> 
	const Vector3<Type> operator -(
	const Vector3<Type>& lhs,
	const Vector3<Type>& rhs) {

	return Vector3<Type>(lhs) -= rhs;
}

template <class Type> 
	const Vector3<Type> operator *(
	const float lhs,
	const Vector3<Type>& rhs) {

	return Vector3<Type>(rhs) *= lhs;
}

template <class Type> 
	const Vector3<Type> operator *(
	const Vector3<Type>& lhs,
	const float rhs) {

	return Vector3<Type>(lhs) *= rhs;
}

template <class Type> 
	const Vector3<Type> operator /(
	const Vector3<Type>& lhs,
	const float rhs) {

	return Vector3<Type>(lhs) /= rhs;
}

template <class Type> 
	Type dot(
	const Vector3<Type>& lhs,
	const Vector3<Type>& rhs) {

	return lhs[X] * rhs[X] + lhs[Y] * rhs[Y] + lhs[Z] * rhs[Z];
}

template <class Type> 
	const Vector3<Type> cross(
	const Vector3<Type>& lhs,
	const Vector3<Type>& rhs) {

	return Vector3<Type>(
		lhs[Y] * rhs[Z] - lhs[Z] * rhs[Y],
		lhs[Z] * rhs[X] - lhs[X] * rhs[Z],
		lhs[X] * rhs[Y] - lhs[Y] * rhs[X]);
}

/**
* \brief Komponentenweise Multiplikation
*/
template <class Type>
	const Vector3<Type> compMul(
	const Vector3<Type>& lhs,
	const Vector3<Type>& rhs)
{
	return Vector3<Type>(
		lhs[X] * rhs[X],
		lhs[Y] * rhs[Y],
		lhs[Z] * rhs[Z]);
}

//! typedef um nicht so viel tippen zu müssen
typedef Vector3<float> Vec3;

#endif // !VECTOR3_H
