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

//------------------------------------------------------------------------------
/**
*  \file BallAndSocketConstraint.h
*  \class BallAndSocketConstraint
*/
//------------------------------------------------------------------------------

#ifndef BALL_AND_SOCKET_CONSTRAINT_H
#define BALL_AND_SOCKET_CONSTRAINT_H

// ---Includes-----------------------------------------------------------------
#include <simon/PrimaryConstraint.h>
#include <cassert>

 
// ---Class definition---------------------------------------------------------
class BallAndSocketConstraint : public PrimaryConstraint {

	friend class ConstraintSystem;
	
protected:

// ---Members: Constraint and constraint derivate------------------------------
    Matrix3x1 mConstraint;    //!< Auswertungsvektor der Zwangsbedingung
	Matrix3x1 mConstraintDot; //!< Ableitung der Zwangsbedingung

// ---Members: Jacobian and jacobian derivate----------------------------------
	Matrix3x6 mJacobian1;     //!< Partielle Ableitung der Zwangsbed.
	//!<   (Jacobsche)
	Matrix3x6 mJacobian1Dot;  //!< Ableitung der Jacobschen Matrix
	Matrix3x6 mJacobian2;     //!< Partielle Ableitung der Zwangsbed.
	//!<   (Jacobsche)
	Matrix3x6 mJacobian2Dot;  //!< Ableitung der Jacobschen Matrix

// ---Members: Right hand side vector------------------------------------------
	Matrix3x1 mRightHandSide1; //!< Rechte Seite d. Gleichungssystems der
	//!<   Bed.ngen
	Matrix3x1 mRightHandSide2; //!< Rechte Seite d. Gleichungssystems der
	//!<   Bed.ngen

// ---Members: lambda (Lagrange multiplier)------------------------------------
	Matrix3x1 mLambda;


	// -- some temp vars for "inhose-keeping" follows

	Matrix3x6
		mRBJacobian1,
		mRBJacobian2,
		mRBJacobian1Dot,
		mRBJacobian2Dot;
	

private:

// ---Members: Position of RigidBodies (body space) to match in world coord.---
    Matrix3x1 mPositionBody1; //!< First  constraint linear position
	Matrix3x1 mPositionBody2; //!< Second constraint linear position

public:
	void swapBodies();

// ---Constructor--------------------------------------------------------------

	BallAndSocketConstraint ();
	BallAndSocketConstraint (RigidBodyPtr objectA, RigidBodyPtr objectB,
							 Matrix3x1 positionA, Matrix3x1 positionB);
	
// ---Destructor--------------------------------------------------------------
	virtual ~BallAndSocketConstraint ();

	virtual PrimaryConstraint::DegreeOfFreedom getDegreeOfFreedom();

// ---Update members: constraint and constraintDot-----------------------------
	virtual void compute();
	virtual void computeConstraint ();
	virtual void computeJacobian ();
	virtual void computeRightHandSide ();
	virtual void computeRightHandSideFast ();

// ---Test: test jacobian------------------------------------------------------
	void testJacobianForBallAndSocket ();
	void testRightHandSideForBallAndSocket();

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
#endif // SIMON_BALL_AND_SOCKET_CONSTRAINT_H
