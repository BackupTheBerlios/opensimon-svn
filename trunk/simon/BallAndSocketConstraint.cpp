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
// File: BallAndSocketConstraint.cpp
// ----------------------------------------------------------------------------
// ////////////////////////////////////////////////////////////////////////////
// $Id: BallAndSocketConstraint.cpp,v 1.37 2004/12/14 18:29:46 alangs Exp $
/**
 *  \class BallAndSocketConstraint
 *  \author Palmer, Hornung
 *
 * CVS:
 *   - $Author: alangs $
 *   - $Date: 2004/12/14 18:29:46 $
 *   - $Revision: 1.37 $
 *
 * \brief Klasse, die eine Zwangsbedingung für ein Kugelgelenk darstellt.
 *
 * Die Klasse BallAndSocketConstraint verwaltet Daten und leitet Informationen
 * ab, die für die Berechnung der Zwangskräfte auf Festkoerper oder Partikel
 * benötigt werden, wenn diese durch ein Kugelgelenk verbunden sind. Siehe
 * besonders die abstrakte Klasse PrimaryConstraint. Die Funktion
 * computeConstraint () stellt sicher, dass die Weltkoordinaten zweier
 * Punkte der beiden beteiligten Festkörper übereinstimmen, bzw. deren
 * relative Geschwindigkeit in Weltkoordinaten Null ist.
 */

// ////////////////////////////////////////////////////////////////////////////

// ----------------------------------------------------------------------------
// includes
// ----------------------------------------------------------------------------
#include <simon/BallAndSocketConstraint.h>
#include <iostream>  // NUR VORLAEUFIG!!!
#include <cmath>
#include <float.h> //für ISNAN/ISINF

#include <simon/SmartPointer.h>

using namespace std;

//@{
//  ---------------------------------------------------------------------------
/// Konstruktoren.
///
/// \param objectA   Erster  beteiligter Festkörper
/// \param objectB   Zweiter beteiligter Festkörper
/// \param positionA Punkt des 1. Festkörper in körperfesten Koordinaten
/// \param positionB Punkt des 2. Festkörper in körperfesten Koordinaten
//  ---------------------------------------------------------------------------
BallAndSocketConstraint::BallAndSocketConstraint ()
{
	assert(false);
}

BallAndSocketConstraint::BallAndSocketConstraint (RigidBodyPtr objectA, 
												  RigidBodyPtr objectB,
												  Matrix3x1 positionA, 
												  Matrix3x1 positionB) :
	PrimaryConstraint (objectA, objectB),

	// store anchor positions in members
	mPositionBody1(positionA),
	mPositionBody2(positionB),

	// stuff
    mConstraint(0.0),
	mConstraintDot(0.0),

	mJacobian1(0.0),
	mJacobian1Dot(0.0),
	mJacobian2(0.0),
	mJacobian2Dot(0.0),
	mRightHandSide1(0.0),
	mRightHandSide2(0.0),
	
	mLambda(0.0),

	mRBJacobian1(0.0),
	mRBJacobian2(0.0),
	mRBJacobian1Dot(0.0),
	mRBJacobian2Dot(0.0) {
}

//@}

//  ---------------------------------------------------------------------------
/// Destruktor.
//  ---------------------------------------------------------------------------
BallAndSocketConstraint::~BallAndSocketConstraint ()
{

  // nn

}

PrimaryConstraint::DegreeOfFreedom BallAndSocketConstraint::getDegreeOfFreedom() {
	return dof3;
}

// protected functions follws


void BallAndSocketConstraint::compute() {
	
/*
// this is the old slow style 
computeConstraint();
computeJacobian();
computeRightHandSide();
return;
*/

	// --- ersteinmal C, den Constraint berechnen

	assert (mRBRotationMatrix1.getSizeM () == 3 && mRBRotationMatrix1.getSizeN () == 3);
	assert (mRBRotationMatrix2.getSizeM () == 3 && mRBRotationMatrix2.getSizeN () == 3);
	assert (mPositionBody1.getSizeM () == 3 && mPositionBody1.getSizeN () == 1);
	assert (mPositionBody2.getSizeM () == 3 && mPositionBody2.getSizeN () == 1);

	// Position und Rotation aus Objekt A und B in den Matrizen speichern
	mRB1->getPosition(mRBPosition1);
	mRB2->getPosition(mRBPosition2);
	mRB1->getOrientation (mRBOrientation1);
	mRB2->getOrientation (mRBOrientation2);
	mRBOrientation1.getRotationMatrix (mRBRotationMatrix1);
	mRBOrientation2.getRotationMatrix (mRBRotationMatrix2);

	// Weltkoordinaten des Gelenkpunktes von Body1&2 mit Hilfe der Rotationsmatrizen berechnen
	//   evtl. kann hier noch was an Geschwindigkeit rausgeholt werden.
	//   Die WorldPositions werden temporär erzeugt, da ein Konstruktor schneller ist, als
	//   der Zuweisungsoperator '=' :(
	Matrix3x1 positionWorld1((mRBRotationMatrix1 * mPositionBody1) + mRBPosition1);
	Matrix3x1 positionWorld2((mRBRotationMatrix2 * mPositionBody2) + mRBPosition2);

	// Constraint: Diff Weltkoord. d. Gelenkpkt == 0
	mConstraint.setValues(positionWorld1 - positionWorld2);

	// --- jetzt das ganze noch fuer die Ableitung von C
	
	// Geschwindigkeit und Winkelgeschwindigkeit von A und B als Matrix
	mRB1->getVelocity(mRBVelocity1);
	mRB2->getVelocity(mRBVelocity2);
	mRB1->getAngularVelocity(mRBAngularVelocity1);
	mRB2->getAngularVelocity(mRBAngularVelocity2);

	// Weltgeschwindigkeiten des Gelenkpunktes von Body1/2
	Matrix3x1 velocityWorld1(
		(mRBAngularVelocity1.star () * (mRBRotationMatrix1 * mPositionBody1)) +
		mRBVelocity1);
	Matrix3x1 velocityWorld2(
		(mRBAngularVelocity2.star () * (mRBRotationMatrix2 * mPositionBody2)) + 
		mRBVelocity2);

	// ConstraintDot: Relative Geschw. der Gelenkpunkte (in Weltkoord.)
	mConstraintDot.setValues(velocityWorld1 - velocityWorld2);


	// --- compute the jacobian
	                                                                              
 	mRBJacobian1 = 0;
	mRBJacobian2 = 0;
	mRBJacobian1Dot = 0;
	mRBJacobian2Dot = 0;
                    
	// setting to ident
	mRBJacobian1[0][0] = 1.0f;
	mRBJacobian1[1][1] = 1.0f;
	mRBJacobian1[2][2] = 1.0f;

	mTemporal3x3Matrix = ((mRBRotationMatrix1 * mPositionBody1).star()).T();

	for (int i=0; i<3; ++i)
		for (int j=0; j<3; ++j)
			mRBJacobian1[i][j+3] = mTemporal3x3Matrix[i][j];

	mRBJacobian2[0][0] = 1.0f;
	mRBJacobian2[1][1] = 1.0f;
	mRBJacobian2[2][2] = 1.0f;
	mTemporal3x3Matrix = ((mRBRotationMatrix2 * mPositionBody2).star ()).T ();

	for (int i=0; i<3; ++i)
		for (int j=0; j<3; ++j)
			mRBJacobian2[i][j+3] = mTemporal3x3Matrix[i][j];

	mRBJacobian2 = -1.0f * mRBJacobian2;

	mRBJacobian1Dot = mRBJacobian1;
	mRBJacobian2Dot = mRBJacobian2;
                                                                                
	mRBJacobian1Dot[0][0] = 0.0f;
	mRBJacobian1Dot[1][1] = 0.0f;
	mRBJacobian1Dot[2][2] = 0.0f;
                                                                                
	mRBJacobian2Dot[0][0] = 0.0f;
	mRBJacobian2Dot[1][1] = 0.0f;
	mRBJacobian2Dot[2][2] = 0.0f;
                                                               
	mRBJacobian1Dot = mRBAngularVelocity1.star () * mRBJacobian1Dot;
	mRBJacobian2Dot = mRBAngularVelocity2.star () * mRBJacobian2Dot;
                                               
	//overwrite 
	//! \todo is this neccesary? Maybe we can directly work with mJacobian1, ...
	mJacobian1.setValues(mRBJacobian1);
	mJacobian2.setValues(mRBJacobian2);
	mJacobian1Dot.setValues(mRBJacobian1Dot);
	mJacobian2Dot.setValues(mRBJacobian2Dot);

	// --- compute righthandside

	computeRightHandSide();

// 	updateInhouseKeeping();
	
// 	computeRightHandSideFast(); // this function comes from the PrimaryConstraint
	
}


//@{
//  ---------------------------------------------------------------------------
/// Update members: constraint and constraintDot.
///
/// \todo Doppelten Code in neue Funktion auslagern
/// \todo Unbedingt den Code testen!!!!!!!
//  ---------------------------------------------------------------------------
void BallAndSocketConstraint::computeConstraint() {

  // --- ersteinmal C, den Constraint berechnen

  // Position und Rotation aus Objekt A und B als Matrix speichern
  getObjectA()->getPosition(mRBPosition1);
  getObjectB()->getPosition(mRBPosition2);
  boost::static_pointer_cast <RigidBody>
                           (getObjectA())->getOrientation(mRBOrientation1);
  boost::static_pointer_cast <RigidBody>
                           (getObjectB())->getOrientation(mRBOrientation2);
  mRBOrientation1.getRotationMatrix(mRBRotationMatrix1);
  mRBOrientation2.getRotationMatrix(mRBRotationMatrix2);

  // Weltkoordinaten des Gelenkpunktes von Body1/2
  Matrix3x1 positionWorld1 = (mRBRotationMatrix1 * mPositionBody1) + mRBPosition1;
  Matrix3x1 positionWorld2 = (mRBRotationMatrix2 * mPositionBody2) + mRBPosition2;

  // Constraint: Diff Weltkoord. d. Gelenkpkt == 0
  mConstraint.setValues(positionWorld1 - positionWorld2);

  // --- jetzt das ganze noch fuer die Ableitung von C

  // Geschwindigkeit und Winkelgeschwindigkeit von A und B als Matrix
  Matrix3x1 velocity1 ((getObjectA ())->getVelocity ());
  Matrix3x1 velocity2 ((getObjectB ())->getVelocity ());
  Matrix3x1 angularVelocity1 (boost::static_pointer_cast <RigidBody> 
                                  (getObjectA ())->getAngularVelocity ());
  Matrix3x1 angularVelocity2 (boost::static_pointer_cast <RigidBody>
                                  (getObjectB ())->getAngularVelocity ());

  // Weltgeschwindigkeiten des Gelenkpunktes von Body1/2
  Matrix3x1 velocityWorld1 =
       (angularVelocity1.star () * (mRBRotationMatrix1 * mPositionBody1)) + velocity1;
  Matrix3x1 velocityWorld2 =
       (angularVelocity2.star () * (mRBRotationMatrix2 * mPositionBody2)) + velocity2;

  // ConstraintDot: Relative Geschw. der Gelenkpunkte (in Weltkoord.)
  mConstraintDot.setValues(velocityWorld1 - velocityWorld2);

}


//  ---------------------------------------------------------------------------
/// Jacobian for a BallAndSocketConstraint
///
/// This code is to speed up computation. It should be included in a release
/// version. The code bases on the idea that mConstraint can be totally
/// derived by analytical operations for a BallAndSocketConstraint.
/// See Barzel and Barr "A Modeling System Based On Dynamic Constraints",
/// Computer Graphics , vol. 22, no 4, Aug 1988, Appendix C.2 for more
/// detail.
//  ---------------------------------------------------------------------------
void BallAndSocketConstraint::computeJacobian ()
{
  
	//PrimaryConstraint::computeJacobian ();
                                                                              
	Matrix3x6 
		jacobian1(0.0),
		jacobian2(0.0),
		jacobian1Dot,
		jacobian2Dot;
	Matrix3x3 temporal;
	mRBOrientation1 = boost::static_pointer_cast <RigidBody>
		(getObjectA ())->getOrientation ();
	mRBOrientation2 = boost::static_pointer_cast <RigidBody>
		(getObjectB ())->getOrientation ();
	Matrix3x3 
		euler1 (mRBOrientation1.getEulerRotation ()),
		euler2 (mRBOrientation2.getEulerRotation ()),
		rotation1 (mRBOrientation1.getRotationMatrix ()),
		rotation2 (mRBOrientation2.getRotationMatrix ());
	Matrix3x1 omega1 = boost::static_pointer_cast <RigidBody>
		(getObjectA ())->getAngularVelocity ();
	Matrix3x1 omega2 = boost::static_pointer_cast <RigidBody>
		(getObjectB ())->getAngularVelocity ();
                                                                                
	jacobian1[0][0] = 1.0f;
	jacobian1[1][1] = 1.0f;
	jacobian1[2][2] = 1.0f;
  
	temporal = ((rotation1 * mPositionBody1).star ()).T ();

	for (int i=0; i<3; ++i)
		for (int j=0; j<3; ++j)
			jacobian1[i][j+3] = temporal[i][j];

	jacobian2[0][0] = 1.0f;
	jacobian2[1][1] = 1.0f;
	jacobian2[2][2] = 1.0f;

	temporal = ((rotation2 * mPositionBody2).star()).T ();

	for (int i=0; i<3; ++i)
		for (int j=0; j<3; ++j)
			jacobian2[i][j+3] = temporal[i][j];

	jacobian2 = -1.0f * jacobian2;

	jacobian1Dot = jacobian1;
	jacobian2Dot = jacobian2;
                                                                                
	jacobian1Dot[0][0] = 0.0f;
	jacobian1Dot[1][1] = 0.0f;
	jacobian1Dot[2][2] = 0.0f;
                                                                                
	jacobian2Dot[0][0] = 0.0f;
	jacobian2Dot[1][1] = 0.0f;
	jacobian2Dot[2][2] = 0.0f;
                                                                                
	jacobian1Dot = omega1.star () * jacobian1Dot;
	jacobian2Dot = omega2.star () * jacobian2Dot;

/*
  mJacobian1.show ();
  jacobian1.show ();
  mJacobian2.show ();
  jacobian2.show ();
*/

/*
  for (unsigned int i=0; i<3; ++i)
  for (unsigned int j=0; j<6; ++j) {
  cout << "1\n";
  assert (fabs (jacobian1[i][j] - mJacobian1[i][j]) < 0.5);
  cout << "2\n";
  assert (fabs (jacobian2[i][j] - mJacobian2[i][j]) < 0.5);
  }
                                                                                
  for (unsigned int i=0; i<3; ++i)
  for (unsigned int j=0; j<6; ++j) {
  cout << "3\n";
  assert (fabs (jacobian1Dot[i][j] - mJacobian1Dot[i][j]) < 0.5);
  cout << "4\n";
  assert (fabs (jacobian2Dot[i][j] - mJacobian2Dot[i][j]) < 0.5);
  }
*/

	//cout << "new code successfully tested\n";
                                                                                
	//overwrite
	mJacobian1.setValues(jacobian1);
	mJacobian2.setValues(jacobian2);
	mJacobian1Dot.setValues(jacobian1Dot);
	mJacobian2Dot.setValues(jacobian2Dot);

	//PrimaryConstraint::computeJacobian ();

}



// ----------------------------------------------------------------------------
/// \brief Update members: rightHandSide.
///
/// Die Funktion wird pro Simulationsschritt aufgerufen, um die rechte
/// Seite des zu lösenden Gleichungssystems der Zwangsbedingung zu
/// aktualisieren.
///
/// \todo Testen!!!
/// \todo mRightHandSide *= -1;
// ----------------------------------------------------------------------------
void BallAndSocketConstraint::computeRightHandSide () {

	// space Geschwindigkeit lesen
	Matrix3x1 velocity1 ((getObjectA ())->getVelocity ());
	Matrix3x1 velocity2 ((getObjectB ())->getVelocity ());
	Matrix3x1 angularVelocity1 (boost::static_pointer_cast <RigidBody>
								(getObjectA ())->getAngularVelocity ());
	Matrix3x1 angularVelocity2 (boost::static_pointer_cast <RigidBody>
								(getObjectB ())->getAngularVelocity ());

	// Kräfte lesen
	Matrix3x1 force1 ((getObjectA ())->getForce ());
	Matrix3x1 force2 ((getObjectB ())->getForce ());
	Matrix3x1 torque1
		(boost::static_pointer_cast <RigidBody> (getObjectA ())->getTorque ());
	Matrix3x1 torque2
		(boost::static_pointer_cast <RigidBody> (getObjectB ())->getTorque ());

	// Inverse Masse lesen
	float massInv1 = 1.0f / ((getObjectA ())->getMass ());
	float massInv2 = 1.0f / ((getObjectB ())->getMass ());

	// Iversen Inertia tensor lesen
	Matrix3x3 inertiaInv1 =
		(boost::static_pointer_cast <RigidBody> (getObjectA ())->getInvWorldInertiaTensor ());
	Matrix3x3 inertiaInv2 =
		(boost::static_pointer_cast <RigidBody> (getObjectB ())->getInvWorldInertiaTensor ());

	// asserts
	assert (force1.getSizeM      () == 3 && force1.getSizeN      () == 1);
	assert (force2.getSizeM      () == 3 && force2.getSizeN      () == 1);
	assert (torque1.getSizeM     () == 3 && torque1.getSizeN     () == 1);
	assert (torque2.getSizeM     () == 3 && torque2.getSizeN     () == 1);
	assert (inertiaInv1.getSizeM () == 3 && inertiaInv1.getSizeN () == 3);
	assert (inertiaInv2.getSizeM () == 3 && inertiaInv2.getSizeN () == 3);

	// space Forces und Masses erstellen
	Matrix6x1 spaceForce1(0.0);
	Matrix6x1 spaceForce2(0.0);
	Matrix6x6 spaceMassInv1(0.0);
	Matrix6x6 spaceMassInv2(0.0);
	Matrix6x1 spaceVelocity1(0.0);
	Matrix6x1 spaceVelocity2(0.0);

	for (unsigned int i=0; i<3; i++) {
		for (unsigned int j=0; j<3; j++) {
			spaceMassInv1[i+3][j+3] = inertiaInv1 [i][j];
			spaceMassInv2[i+3][j+3] = inertiaInv1 [i][j];
		}

		// diagonale Massenmatrix
		spaceMassInv1  [i]  [i] = massInv1;
		spaceMassInv2  [i]  [i] = massInv2;
		spaceForce1    [i]  [0] = force1           [i][0];
		spaceForce2    [i]  [0] = force2           [i][0];
		spaceForce1    [i+3][0] = torque1          [i][0];
		spaceForce2    [i+3][0] = torque2          [i][0];
		spaceVelocity1 [i]  [0] = velocity1        [i][0];
		spaceVelocity2 [i]  [0] = velocity2        [i][0];
		spaceVelocity1 [i+3][0] = angularVelocity1 [i][0];
		spaceVelocity2 [i+3][0] = angularVelocity2 [i][0];
	}

 
	// space Acceleration berechnen
	Matrix6x1 spaceAcceleration1 = spaceMassInv1 * spaceForce1;
	Matrix6x1 spaceAcceleration2 = spaceMassInv2 * spaceForce2;

	// rechter Summand
	Matrix3x1 rightAddend1 = mJacobian1 * spaceAcceleration1;
	Matrix3x1 rightAddend2 = mJacobian2 * spaceAcceleration2;

	// linker Summand
	Matrix3x1 leftAddend1 = mJacobian1Dot * spaceVelocity1;
	Matrix3x1 leftAddend2 = mJacobian2Dot * spaceVelocity2;

	// Summe
	Matrix3x1 sum1 = leftAddend1 + rightAddend1;
	Matrix3x1 sum2 = leftAddend2 + rightAddend2;
  
	if (mDoBaumgarteStabilisation) {
		float tau = mTau;
	
		float k1 = 1/(tau*tau);
		float k2 = 2/tau;
	
		//! \todo Was soll der float * matrix operator genau tun? Hoffentlich das selbe wie matrix * float. Bitte überprüfen.
		sum1 = sum1 + k1 * mConstraint + k2 * mConstraintDot; 	
		sum2 = sum2 + k1 * mConstraint + k2 * mConstraintDot;
	}
  
	/*Matrix <float> sum1 = leftAddend1 + rightAddend1;
	  Matrix <float> sum2 = leftAddend2 + rightAddend2;*/

	// right hand side, finally!!!
	mRightHandSide1 = sum1 * (-1);
	mRightHandSide2 = sum2 * (-1);

}


// ----------------------------------------------------------------------------
/// \brief Update members: rightHandSide.
///
/// Die Funktion wird pro Simulationsschritt aufgerufen, um die rechte
/// Seite des zu lösenden Gleichungssystems der Zwangsbedingung zu
/// aktualisieren. Sie ist optimiert auf Geschwindigkeit.
///
// ----------------------------------------------------------------------------
void BallAndSocketConstraint::computeRightHandSideFast () {

  // Inverse Masse lesen
  float massInv1 = 1.0f / mRB1->getMass ();
  float massInv2 = 1.0f / mRB2->getMass ();

  mJacobian1.getMxNMatrix (0, 0, mRBJacobian1Part1);
  mJacobian1.getMxNMatrix (0, 3, mRBJacobian1Part2);
  mJacobian2.getMxNMatrix (0, 0, mRBJacobian2Part1);
  mJacobian2.getMxNMatrix (0, 3, mRBJacobian2Part2);

  mJacobian1Dot.getMxNMatrix (0, 0, mRBJacobian1DotPart1);
  mJacobian1Dot.getMxNMatrix (0, 3, mRBJacobian1DotPart2);
  mJacobian2Dot.getMxNMatrix (0, 0, mRBJacobian2DotPart1);
  mJacobian2Dot.getMxNMatrix (0, 3, mRBJacobian2DotPart2);

  mRBAcceleration1 = massInv1 * mRBForce1;
  mRBAcceleration2 = massInv2 * mRBForce2;

  mRBInertiaInv1.multiply (mRBTorque1, mRBAngularAcceleration1);
  mRBInertiaInv2.multiply (mRBTorque2, mRBAngularAcceleration2);

  mRBJacobian1Part2.multiply (mRBAngularAcceleration1, mTemporal3x1Matrix1);
  mRBJacobian1Part1.multiply (mRBAcceleration1,        mTemporal3x1Matrix2);
  mTemporal3x1Matrix1.add (mTemporal3x1Matrix2, mTemporal3x1Matrix3);

  mRBJacobian1DotPart2.multiply (mRBAngularVelocity1, mTemporal3x1Matrix1);
  mRBJacobian1DotPart1.multiply (mRBVelocity1, mTemporal3x1Matrix2);
  mTemporal3x1Matrix1.add (mTemporal3x1Matrix2, mTemporal3x1Matrix4);
  cout << "Achtung! Buggy code wird ausgeführt (computeRightHandSideFast() )!" <<endl;
  //! \todo machen das das geht
  //mTemporal3x1Matrix3.add (mTemporal3x1Matrix4, mRightHandSide1);

  mRBJacobian2Part2.multiply (mRBAngularAcceleration2, mTemporal3x1Matrix1);
  mRBJacobian2Part1.multiply (mRBAcceleration2,       mTemporal3x1Matrix2);
  mTemporal3x1Matrix1.add (mTemporal3x1Matrix2, mTemporal3x1Matrix3);

  mRBJacobian2DotPart2.multiply (mRBAngularVelocity2, mTemporal3x1Matrix1);
  mRBJacobian2DotPart1.multiply (mRBVelocity2,        mTemporal3x1Matrix2);
  mTemporal3x1Matrix1.add (mTemporal3x1Matrix2, mTemporal3x1Matrix4);
  //! \todo machen das das geht
  //mTemporal3x1Matrix3.add (mTemporal3x1Matrix4, mRightHandSide2);

  if (mDoBaumgarteStabilisation) {
    float tau = mTau;
    
    float k1 = 1/(tau*tau);
    float k2 = 2/tau;
    
    mRightHandSide1 = mRightHandSide1 + (k1 * mConstraint + k2 * mConstraintDot);
    mRightHandSide2 = mRightHandSide2 + (k1 * mConstraint + k2 * mConstraintDot);
  }
  
  // right hand side, finally!!!
  mRightHandSide1 = mRightHandSide1 * (-1);
  mRightHandSide2 = mRightHandSide2 * (-1);
  
}

// ----------------------------------------------------------------------------
/// \brief Update RigidBodies
///
/// Die Funktion setzt bei den RigidBodies die berechneten zusätzlichen
/// Kräfte...
// ----------------------------------------------------------------------------
void BallAndSocketConstraint::updateRigidBodies ()
{

	Matrix6x1 spaceForce1(0.0);
	Matrix6x1 spaceForce2(0.0);

	spaceForce1 = mJacobian1.T () * mLambda;
	spaceForce2 = mJacobian2.T () * mLambda;

	// Hack, der bei einem NaN-Fehler die spaceForce Null setzt
	if (ISNAN(spaceForce1[0][0]) || ISNAN(spaceForce1[1][0]) ||
		ISNAN(spaceForce1[2][0]) || ISNAN(spaceForce1[3][0]) ||
		ISNAN(spaceForce1[4][0]) || ISNAN(spaceForce1[5][0])) {
		spaceForce1 = Matrix6x1(0.0f);
	}
	if (ISINF(spaceForce1[0][0]) || ISINF(spaceForce1[1][0]) ||
		ISINF(spaceForce1[2][0]) || ISINF(spaceForce1[3][0]) ||
		ISINF(spaceForce1[4][0]) || ISINF(spaceForce1[5][0])) {
		spaceForce1 = Matrix6x1(0.0f);
	}

	if (ISNAN(spaceForce2[0][0]) || ISNAN(spaceForce2[1][0]) ||
		ISNAN(spaceForce2[2][0]) || ISNAN(spaceForce2[3][0]) ||
		ISNAN(spaceForce2[4][0]) || ISNAN(spaceForce2[5][0])) {
		spaceForce1 =  Matrix6x1(0.0f);
	}
	if (ISINF(spaceForce2[0][0]) || ISINF(spaceForce2[1][0]) ||
		ISINF(spaceForce2[2][0]) || ISINF(spaceForce2[3][0]) ||
		ISINF(spaceForce2[4][0]) || ISINF(spaceForce2[5][0])) {
		spaceForce2 = Matrix6x1(0.0f);
	}

	assert (spaceForce1.getSizeM () == 6);
	assert (spaceForce2.getSizeM () == 6);
	assert (spaceForce1.getSizeN () == 1);
	assert (spaceForce2.getSizeN () == 1);
  
	boost::static_pointer_cast <RigidBody> (getObjectA ())->processConstraints (spaceForce1);
	boost::static_pointer_cast <RigidBody> (getObjectB ())->processConstraints (spaceForce2);

}

//  ---------------------------------------------------------------------------
/// Test: test Jacobian for a BallAndSocketConstraint
///
/// This code is for test purposes only. It need not be included in a release
/// version. The code bases on the idea that mConstraint can be partially
/// derived by analytical operations.
//  ---------------------------------------------------------------------------
void BallAndSocketConstraint::testJacobianForBallAndSocket ()
{

	Matrix3x6 
		testJacobian1(0.0),
		testJacobian2(0.0),
		testJacobian1Dot,
		testJacobian2Dot;
	Quaternion orientation1 = boost::static_pointer_cast <RigidBody>
		(getObjectA ())->getOrientation ();
	Quaternion orientation2 = boost::static_pointer_cast <RigidBody>
		(getObjectB ())->getOrientation ();
	Matrix3x1 euler1 (orientation1.getEulerRotation ());
	Matrix3x1 euler2 (orientation2.getEulerRotation ());
	Matrix3x1 omega1 = boost::static_pointer_cast <RigidBody>
		(getObjectA ())->getAngularVelocity ();
	Matrix3x1 omega2 = boost::static_pointer_cast <RigidBody>
		(getObjectB ())->getAngularVelocity ();
	float s11 = sin (euler1[0][0]);
	float s12 = sin (euler1[1][0]);
	float s13 = sin (euler1[2][0]);
	float c11 = cos (euler1[0][0]);
	float c12 = cos (euler1[1][0]);
	float c13 = cos (euler1[2][0]);

	testJacobian1[0][0] = 1.0f;
	testJacobian1[1][1] = 1.0f;
	testJacobian1[2][2] = 1.0f;

	testJacobian1[0][3] = (s12 * c11 *c13 + s11*s13) *
		mPositionBody1[1][0] +
		(-1 * s12 * s11 * c13 + c11 * s13) *
		mPositionBody1[2][0];
	testJacobian1[0][4] = (-1 * s12 * c13) *
		mPositionBody1[0][0] +
		(c12 * s11 * c13 - c11 * s13) *
		mPositionBody1[1][0] +
		(c12 * c11 * c13 + s11 * s13) *
		mPositionBody1[2][0];
	testJacobian1[0][5] = (-1 * c12 * s13) *
		mPositionBody1[0][0] +
		(-1 * s12 * s11 * s13 - c11 * c13) *
		mPositionBody1[1][0] +
		(-1 * s12 * c11 * s13 + s11 * c13) *
		mPositionBody1[2][0];
	testJacobian1[1][3] = (s12 * c11 * s13 - s11 * c13) *
		mPositionBody1[1][0] +
		(-1 * s12 * s11 * s13 - c11 * c13) *
		mPositionBody1[2][0];
	testJacobian1[1][4] = (-1 * s12 * s13) * 
		mPositionBody1[0][0] +
		(c12 * s11 * s13) *
		mPositionBody1[1][0] +
		(-1 * c12 * c11 * s13) *
		mPositionBody1[2][0];
	testJacobian1[1][5] = (c12 * c13) *
		mPositionBody1[0][0] +
		(s12 * s11 * c13 - c11 * s13) *
		mPositionBody1[1][0] +
		(s12 * c11 * c13 + s11 * s13) *
		mPositionBody1[2][0];
	testJacobian1[2][3] = (c12 * c11) *
		mPositionBody1[1][0] +
		(-1 * c12 * s11) *
		mPositionBody1[2][0];
	testJacobian1[2][4] = (-1 * c12) * 
		mPositionBody1[0][0] +
		(-1 * s12 * s11) *
		mPositionBody1[1][0] +
		(-1 * s12 * c11) *
		mPositionBody1[2][0];
	//testJacobian1[2][5] = 0.0;

	s11 = sin (euler2[0][0]);
	s12 = sin (euler2[1][0]);
	s13 = sin (euler2[2][0]);
	c11 = cos (euler2[0][0]);
	c12 = cos (euler2[1][0]);
	c13 = cos (euler2[2][0]);

	testJacobian2[0][0] = 1.0f;
	testJacobian2[1][1] = 1.0f;
	testJacobian2[2][2] = 1.0f;

	testJacobian2[0][3] = (s12 * c11 *c13 + s11*s13) *
		mPositionBody2[1][0] +
		(-1 * s12 * s11 * c13 + c11 * s13) *
		mPositionBody2[2][0];
	testJacobian2[0][4] = (-1 * s12 * c13) *
		mPositionBody2[0][0] +
		(c12 * s11 * c13 - c11 * s13) *
		mPositionBody2[1][0] +
		(c12 * c11 * c13 + s11 * s13) *
		mPositionBody2[2][0];
	testJacobian2[0][5] = (-1 * c12 * s13) *
		mPositionBody2[0][0] +
		(-1 * s12 * s11 * s13 - c11 * c13) *
		mPositionBody2[1][0] +
		(-1 * s12 * c11 * s13 + s11 * c13) *
		mPositionBody2[2][0];
	testJacobian2[1][3] = (s12 * c11 * s13 - s11 * c13) *
		mPositionBody2[1][0] +
		(-1 * s12 * s11 * s13 - c11 * c13) *
		mPositionBody2[2][0];
	testJacobian2[1][4] = (-1 * s12 * s13) *
		mPositionBody2[0][0] +
		(c12 * s11 * s13) *
		mPositionBody2[1][0] +
		(-1 * c12 * c11 * s13) *
		mPositionBody2[2][0];
	testJacobian2[1][5] = (c12 * c13) *
		mPositionBody2[0][0] +
		(s12 * s11 * c13 - c11 * s13) *
		mPositionBody2[1][0] +
		(s12 * c11 * c13 + s11 * s13) *
		mPositionBody2[2][0];
	testJacobian2[2][3] = (c12 * c11) *
		mPositionBody2[1][0] +
		(-1 * c12 * s11) *
		mPositionBody2[2][0];
	testJacobian2[2][4] = (-1 * c12) *
		mPositionBody2[0][0] +
		(-1 * s12 * s11) *
		mPositionBody2[1][0] +
		(-1 * s12 * c11) *
		mPositionBody2[2][0];

	testJacobian2 = -1.0f * testJacobian2;

  

	testJacobian1Dot = testJacobian1;
	testJacobian2Dot = testJacobian2;

	testJacobian1Dot[0][0] = 0.0f;
	testJacobian1Dot[1][1] = 0.0f;
	testJacobian1Dot[2][2] = 0.0f;

	testJacobian2Dot[0][0] = 0.0f;
	testJacobian2Dot[1][1] = 0.0f;
	testJacobian2Dot[2][2] = 0.0f;

	testJacobian1Dot = omega1.star () * testJacobian1Dot;
	testJacobian2Dot = omega2.star () * testJacobian2Dot;
  
  
/*  
	cout << "testJacobian1 mJacobian1 testJacobian2 mJacobian2\n";
	testJacobian1.show ();
	mJacobian1.show ();
	testJacobian2.show ();
	mJacobian2.show ();
*/
  
/*  
	for (unsigned int i=0; i<3; ++i)
    for (unsigned int j=0; j<6; ++j) {
	assert (fabs (testJacobian1[i][j] - mJacobian1[i][j]) < 0.5);
	assert (fabs (testJacobian2[i][j] - mJacobian2[i][j]) < 0.5);
    }

	for (unsigned int i=0; i<3; ++i)
    for (unsigned int j=0; j<6; ++j) {
	assert (fabs (testJacobian1Dot[i][j] - mJacobian1Dot[i][j]) < 0.5);
	assert (fabs (testJacobian2Dot[i][j] - mJacobian2Dot[i][j]) < 0.5);
    }
*/

    //overwrite
	mJacobian1.setValues(testJacobian1);
	mJacobian2.setValues(testJacobian2);
	mJacobian1Dot.setValues(testJacobian1Dot);
	mJacobian2Dot.setValues(testJacobian2Dot);

}

void BallAndSocketConstraint::testRightHandSideForBallAndSocket() {
	Quaternion orientation1 = boost::static_pointer_cast <RigidBody>
				(getObjectA ())->getOrientation ();
	Quaternion orientation2 = boost::static_pointer_cast <RigidBody>
				(getObjectB ())->getOrientation ();
	Matrix3x3 rotation1 (orientation1.getRotationMatrix ());
	Matrix3x3 rotation2 (orientation2.getRotationMatrix ());
	Matrix3x1 omega1 = boost::static_pointer_cast <RigidBody>
				(getObjectA ())->getAngularVelocity ();
	Matrix3x1 omega2 = boost::static_pointer_cast <RigidBody>
				(getObjectB ())->getAngularVelocity ();
				
	Matrix3x1 testRightHandSide1;
	Matrix3x1 testRightHandSide2;
	
	testRightHandSide1 = -1.0f * (omega1.star() * (omega1.star() * (rotation1 * mPositionBody1)));
	testRightHandSide2 = omega2.star() * (omega2.star() * (rotation2 * mPositionBody2));
	

	for (unsigned int i=0; i<3; ++i) {
    		for (unsigned int j=0; j<1; ++j) {
			assert (fabs (testRightHandSide1[i][j] - mRightHandSide1[i][j]) < 0.05);
			assert (fabs (testRightHandSide2[i][j] - mRightHandSide2[i][j]) < 0.05);
    		}
	}
	
	//overwrite
	mRightHandSide1.setValues(testRightHandSide1);
	mRightHandSide2.setValues(testRightHandSide2);
	
}


void BallAndSocketConstraint::swapBodies() {
	PrimaryConstraint::swapBodies();

	//! \todo überprüfen, ob das wirklich sinn macht, oder wie das anders gelöst werden kann.
	Matrix3x1 temp1 = mPositionBody1;
	Matrix3x1 temp2 = mPositionBody2;
	mPositionBody1 = temp2;
	mPositionBody2 = temp1;
}

// ----------------------------------------------------------------------------


Matrix6x6 BallAndSocketConstraint::getJacobian1() {
	Matrix6x6 temporal(0.0f);
	temporal.setValues(mJacobian1);
	return temporal;
}

Matrix6x6 BallAndSocketConstraint::getJacobian2() {
	Matrix6x6 temporal(0.0f);
	temporal.setValues(mJacobian2);
	return temporal;
}

Matrix6x1 BallAndSocketConstraint::getRightHandSide1() {
	Matrix6x1 temporal(0.0f);
	temporal.setValues(mRightHandSide1);
	return temporal;
}

Matrix6x1 BallAndSocketConstraint::getRightHandSide2() {
	Matrix6x1 temporal(0.0f);
	temporal.setValues(mRightHandSide2);
	return temporal;
}

Matrix6x1 BallAndSocketConstraint::getConstraint() {
	Matrix6x1 temporal(0.0f);
	temporal.setValues(mConstraint);
	return temporal;
}


void BallAndSocketConstraint::setLambda(const Matrix6x1 &lambda) {
	lambda.getMxNMatrix(0,0,mLambda);
}
