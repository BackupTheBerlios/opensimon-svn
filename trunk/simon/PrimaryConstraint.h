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

// $Id: PrimaryConstraint.h,v 1.29 2004/12/14 18:29:46 alangs Exp $
//------------------------------------------------------------------------------
/**
*  \file PrimaryConstraint.h
*  \class PrimaryConstraint
*/
//------------------------------------------------------------------------------

#ifndef PRIMARY_CONSTRAINT_H
#define PRIMARY_CONSTRAINT_H

// ---Includes-----------------------------------------------------------------
#include <simon/Connection.h>
#include <simon/RigidBody.h>
#include <simon/Matrix.h>
#include <simon/SimonState.h>
 

// ---Class definition---------------------------------------------------------
class PrimaryConstraint : public Connection
{

protected:

// ---Members: Step size for computation of the jacobian-----------------------
	double mStepSize;
    
 	//Values for after computation correction of forces
	/*float mMaxForceLengthAngular;
	  float mMaxForceLengthLinear;
	  bool mDoMaxForceLengthCheck;*/
	float mTau; //Drift stabilization factor
	static bool mDoBaumgarteStabilisation;
    
	// -- some temp vars for "inhose-keeping" follows
 
	Matrix3x1
		mRBAngularVelocity1,
		mRBAngularVelocity2,
		mRBTorque1,
		mRBTorque2,
		mRBAngularAcceleration1,
		mRBAngularAcceleration2,

		mTemporal3x1Matrix1,
		mTemporal3x1Matrix2,
		mTemporal3x1Matrix3,
		mTemporal3x1Matrix4,

		mTemporal3x1Matrix;

	Matrix3x3
		mRBRotationMatrix1,
		mRBRotationMatrix2,
		mRBInertiaInv1,
		mRBInertiaInv2,

		mRBJacobian1Part1,
		mRBJacobian1Part2,
		mRBJacobian2Part1,
		mRBJacobian2Part2,
		mRBJacobian1DotPart1,
		mRBJacobian1DotPart2,
		mRBJacobian2DotPart1,
		mRBJacobian2DotPart2,

		mTemporal3x3Matrix;

	Vec3
		mRBPosition1,
		mRBPosition2,
		mRBVelocity1,
		mRBVelocity2,
		mRBForce1,
		mRBForce2,
		mRBAcceleration1,
		mRBAcceleration2;

	Quaternion
		mRBOrientation1,
		mRBOrientation2;

	RigidBodyPtr mRB1;
	RigidBodyPtr mRB2;
    

public:

	enum DegreeOfFreedom {
		dof0 = 0,
		dof1,
		dof2,
		dof3,
		dof4,
		dof5,
		dof6
	};
		

// ---Constructor--------------------------------------------------------------
	PrimaryConstraint();
	PrimaryConstraint(RigidBodyPtr objectA, RigidBodyPtr objectB);
	
// ---Destructor---------------------------------------------------------------
	virtual ~PrimaryConstraint ();

// ---Convenients--------------------------------------------------------------
	void setStepSize (double h);
	virtual void setLambda (const Matrix6x1 &lambda) = 0;
	float getTau();
	void setTau(float);

	virtual DegreeOfFreedom getDegreeOfFreedom();

	virtual void swapBodies();
	static void doBaumgarteStabilisation(bool);
	static bool doesBaumgarteStabilisation();

	//! Access to the first RigidBody 
	RigidBodyPtr getRigidBodyA();
	//! Access to the second RigidBody 
	RigidBodyPtr getRigidBodyB();

// protected:

	//! compute the constraint, jacobian and rightHandSide
	virtual void compute();

// ---Update members: constraint and constraintDot-----------------------------
	virtual void computeConstraint () = 0;

// ---Update members: jacobian and jacobianDot---------------------------------
// (Partielle Ableitung s. "Numerical Recipes")
	virtual void computeJacobian () = 0;

// ---Update members: rightHandSide--------------------------------------------
	virtual void computeRightHandSide () = 0;
	virtual void computeRightHandSideFast () = 0;

// ---Update members: inhouse keeping------------------------------------------
	virtual void updateInhouseKeeping ();

// ---Update all members: Simulation-------------------------------------------
	virtual void simulate ();

// ---Update RigidBodies: force and torque-------------------------------------
	virtual void updateRigidBodies () = 0;

	virtual Matrix6x6 getJacobian1() = 0;
	virtual Matrix6x6 getJacobian2() = 0;

	virtual Matrix6x1 getRightHandSide1() = 0;
	virtual Matrix6x1 getRightHandSide2() = 0;

	virtual Matrix6x1 getConstraint() = 0;
};

// ----------------------------------------------------------------------------
#endif // PRIMARY_CONSTRAINT_H
