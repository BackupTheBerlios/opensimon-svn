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


// ////////////////////////////////////////////////////////////////////////////
// ----------------------------------------------------------------------------
// Universitaet Koblenz - Computergraphik
// Projektpraktikum  --- "MARIONETTE" ---
//
// ----------------------------------------------------------------------------
// ////////////////////////////////////////////////////////////////////////////

// $Id: HingeConstraint.h,v 1.8 2004/11/05 08:31:34 trappe Exp $
//------------------------------------------------------------------------------
/**
*  \file HingeConstraint.h
*  \class HingeConstraint
*  $Author: trappe $
*  $Date: 2004/11/05 08:31:34 $
*  $Revision: 1.8 $
*/
//------------------------------------------------------------------------------

#ifndef HINGE_CONSTRAINT_H
#define HINGE_CONSTRAINT_H

// ---Includes-----------------------------------------------------------------
#include <simon/PrimaryConstraint.h>
#include <simon/SimonState.h>

#include <cassert>
 
// ---Class definition---------------------------------------------------------
class HingeConstraint : public PrimaryConstraint {

	friend class ConstraintSystem;
	
protected:

// ---Members: Constraint and constraint derivate------------------------------
    Matrix5x1 mConstraint;    //!< Auswertungsvektor der Zwangsbedingung
	Matrix5x1 mConstraintDot; //!< Ableitung der Zwangsbedingung

// ---Members: Jacobian and jacobian derivate----------------------------------
	Matrix5x6 mJacobian1;     //!< Partielle Ableitung der Zwangsbed.
	//!<   (Jacobsche)
	Matrix5x6 mJacobian1Dot;  //!< Ableitung der Jacobschen Matrix
	Matrix5x6 mJacobian2;     //!< Partielle Ableitung der Zwangsbed.
	//!<   (Jacobsche)
	Matrix5x6 mJacobian2Dot;  //!< Ableitung der Jacobschen Matrix

// ---Members: Right hand side vector------------------------------------------
	Matrix5x1 mRightHandSide1; //!< Rechte Seite d. Gleichungssystems der
	//!<   Bed.ngen
	Matrix5x1 mRightHandSide2; //!< Rechte Seite d. Gleichungssystems der
	//!<   Bed.ngen

// ---Members: lambda (Lagrange multiplier)------------------------------------
	Matrix3x1 mLambda;

private:

// ---Members: Position of RigidBodies (body space) to match in world coord.---
    Matrix3x1 mPositionBody1; //!< First  constraint linear position 3x1
	Matrix3x1 mPositionBody2; //!< Second constraint linear position 3x1
	Matrix3x1 mNormalPoint1; //Normal end point for Body 1
	Matrix3x1 mNormalPoint2; //Normal end point for Body 2

public:
	void swapBodies();

// ---Constructor--------------------------------------------------------------
	HingeConstraint ();
	HingeConstraint (RigidBodyPtr objectA, RigidBodyPtr objectB,
					 Matrix3x1 positionA, Matrix3x1 positionB,
					 Matrix3x1 positionN1, Matrix3x1 positionN2,
					 bool specifyNormals = true);
	
// ---Destructor--------------------------------------------------------------
	virtual ~HingeConstraint ();

	virtual PrimaryConstraint::DegreeOfFreedom getDegreeOfFreedom();

// ---Update members: constraint and constraintDot-----------------------------
	virtual void computeConstraint ();

// ---Update members: jacobian and jacobianDot---------------------------------
// analytisch fuer HingeConstraint
	virtual void computeJacobian ();

	virtual void computeRightHandSide ();
	virtual void computeRightHandSideFast ();

// ---Update RigidBodies: force and torque-------------------------------------
	virtual void updateRigidBodies ();

	virtual void setLambda (const Matrix6x1 &lambda);

	virtual Matrix6x6 getJacobian1();
	virtual Matrix6x6 getJacobian2();

	virtual Matrix6x1 getRightHandSide1();
	virtual Matrix6x1 getRightHandSide2();

	virtual Matrix6x1 getConstraint();
};



// ----------------------------------------------------------------------------
#endif // HINGE_CONSTRAINT_H
