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
// File: HingeConstraint.cpp
// ----------------------------------------------------------------------------
// ////////////////////////////////////////////////////////////////////////////
// $Id: HingeConstraint.cpp,v 1.22 2004/12/14 18:29:46 alangs Exp $
/**
 *  \class HingeConstraint
 *  \author Palmer, Hornung
 *
 * CVS:
 *   - $Author: alangs $
 *   - $Date: 2004/12/14 18:29:46 $
 *   - $Revision: 1.22 $
 *
 * \brief Klasse, die eine Zwangsbedingung für ein Scharniergelenk darstellt.
 *
 * Die Klasse HingeConstraint verwaltet Daten und leitet Informationen
 * ab, die für die Berechnung der Zwangskräfte auf Festkoerper oder Partikel
 * benötigt werden, wenn diese durch ein Scharniergelenk verbunden sind. Siehe
 * besonders die abstrakte Klasse PrimaryConstraint. Die Funktion
 * computeConstraint () stellt sicher, dass die Weltkoordinaten zweier
 * Punkte der beiden beteiligten Festkörper übereinstimmen, bzw. deren
 * relative Geschwindigkeit in Weltkoordinaten Null ist. Ausserdem stellt sie sicher, 
 * das bestimmte Punkte der beiden Körper immer Element derselben Ebene sind.
 * So wird ein Scharniergelenkähnliches Verhalten erzeugt.
 *
 * Die Constraintbedinung ist nach der in Michael Kriegs Diplomarbeit 
 * "Simulation und Steuerung biomechanischer Mehrkörpersysteme" vorgeschlagenen 
 * Constraintbedingung modelliert
 * (http://www.tat.physik.uni-tuebingen.de/~biomechanik/publ/dipl/
 * michaelkrieg/diplom_michael_krieg.pdf).
 * 
 * Im Moment vertragen sich die HingeConstraints nicht mit der Baumgarte-Stabilisierungs-Methode.
 */

// ////////////////////////////////////////////////////////////////////////////

// ----------------------------------------------------------------------------
// includes
// ----------------------------------------------------------------------------
#include <simon/HingeConstraint.h>
#include <iostream>  // NUR VORLAEUFIG!!!

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
/// \param positionN1 Ebenenpunkt des 1. Körpers in körperfesten Koordinaten
/// \param positionN2 Ebenenpunkt des 2. Körpers in körperfesten Koordinaten
/// \param specifyNormal positionN1, positionN2 gibt Normale an
//  ---------------------------------------------------------------------------
HingeConstraint::HingeConstraint ()
{

  // nn

}


//positionA: Gelenkpunt von objectA, in Objektkoordinaten von Objekt A
//positionB: Gelenkpunt von objectB, in Objektkoordinaten von Objekt B
//positionN: Punk in Objektkoordinaten von Objekt A, A->N ist die Normale
//auf die Gelenkebene
HingeConstraint::HingeConstraint
                        (RigidBodyPtr objectA, RigidBodyPtr objectB,
                         Matrix3x1 positionA, Matrix3x1 positionB,
                         Matrix3x1 positionN1, Matrix3x1 positionN2,
                         bool specifyNormal) :
  PrimaryConstraint (objectA, objectB),
  mPositionBody1 (positionA),
  mPositionBody2 (positionB),
  mNormalPoint1 (positionN1),
  mNormalPoint2 (positionN2)
{

//!  \todo entfernen wenn sich alle sicher sind, das es nicht mehr gebraucht wird
/*
  mRightHandSide1 = Matrix <float> (5,1);
  mRightHandSide2 = Matrix <float> (5,1);
                                                                                                                                                      
  mRBJacobian1Part1 = Matrix <float> (5,3);
  mRBJacobian1Part2 = Matrix <float> (5,3);
  mRBJacobian2Part1 = Matrix <float> (5,3);
  mRBJacobian2Part2 = Matrix <float> (5,3);
  mRBJacobian1DotPart1 = Matrix <float> (5,3);
  mRBJacobian1DotPart2 = Matrix <float> (5,3);
  mRBJacobian2DotPart1 = Matrix <float> (5,3);
  mRBJacobian2DotPart2 = Matrix <float> (5,3);
*/
	if (!specifyNormal)
		return;

	else {

		cout << "Achtung! Code auskommentiert in HingeConstraint Construktor" << endl;
		//! \todo reparieren und wieder einkommentieren
/*
// Es wird vorausgesetzt, dass Pivot und Normale linear unabhängig sind
mNormalPoint1 = positionN1.star () * positionA;
mNormalPoint2 = positionN2.star () * positionB;
Matrix3x1 length1;
Matrix3x1 length2;
length1 = (mNormalPoint1).T () *
(mNormalPoint1);
length2 = (mNormalPoint2).T () *
(mNormalPoint2);
if (fabs (length1 [0][0]) < 7.41322e-39 ||
fabs (length2 [0][0]) < 7.41322e-39) {
SimonState::exemplar()->errors << "Simon: Constraints: ";
SimonState::exemplar()->errors << "HingeConstraint: Normal ";
SimonState::exemplar()->errors << "not well-defined.\n";
SimonState::exemplar()->errors << "  This will lead to bad behaviour ";
SimonState::exemplar()->errors << "during simulation.\n";
SimonState::exemplar()->errors << "  Probably it will cause a ";
SimonState::exemplar()->errors << "Matrix not symmetric and positive or ";
SimonState::exemplar()->errors << "negative\n    definite error or ";
SimonState::exemplar()->errors << "a zero devision.\n";
SimonState::exemplar()->errors << "  As a result the simulation might ";
SimonState::exemplar()->errors << "crash!\n";
SimonState::exemplar()->errors << "  Contact: hornung@uni-koblenz.de\n";
SimonState::exemplar()->errors << SimonState::endm;
}
else {
if (fabs (length1 [0][0]) < 0.001 || fabs (length2 [0][0]) < 0.001) {
SimonState::exemplar()->warnings << "Simon: Constraints: ";
SimonState::exemplar()->warnings << "HingeConstraint: Normal ";
SimonState::exemplar()->warnings << "not well-defined.\n";
SimonState::exemplar()->warnings << "  This might lead to bad ";
SimonState::exemplar()->warnings << "behaviour ";
SimonState::exemplar()->warnings << "during simulation.\n";
SimonState::exemplar()->warnings << "  It might lead to a ";
SimonState::exemplar()->warnings << "Matrix not symmetric and ";
SimonState::exemplar()->warnings << "positive or ";
SimonState::exemplar()->warnings << "negative\n    definite error or ";
SimonState::exemplar()->warnings << "a zero devision.\n";
SimonState::exemplar()->warnings << "  As a result the simulation ";
SimonState::exemplar()->warnings << "might crash!\n";
SimonState::exemplar()->warnings << "  Contact: ";
SimonState::exemplar()->warnings << "hornung@uni-koblenz.de\n";
SimonState::exemplar()->warnings << SimonState::endm;
cout << length1 [0][0] << " " << length2 [0][0] << endl;
}
}
*/
	}

}

//@}

//  ---------------------------------------------------------------------------
/// Destruktor.
//  ---------------------------------------------------------------------------
HingeConstraint::~HingeConstraint ()
{

  // nn

}

PrimaryConstraint::DegreeOfFreedom HingeConstraint::getDegreeOfFreedom() {
	return dof5;
}

// protected:

//@{
//  ---------------------------------------------------------------------------
/// Update members: constraint and constraintDot.
///
/// \param x Position und Orientierung Vektor 6x1
/// \todo Parameter x neu benennen?
/// \todo Doppelten Code in neue Funktion auslagern
/// \todo Unbedingt den Code testen!!!!!!!
//  ---------------------------------------------------------------------------
void HingeConstraint::computeConstraint ()
{

	// --- ersteinmal C, den Constraint berechnen

	// Position und Rotation aus Objekt A und B als Matrix speichern
	Matrix3x1 position1 ((getObjectA ())->getPosition ());
	Matrix3x1 position2 ((getObjectB ())->getPosition ());
	Quaternion orientation1 = boost::static_pointer_cast <RigidBody>
		(getObjectA ())->getOrientation ();
	Quaternion orientation2 = boost::static_pointer_cast <RigidBody>
		(getObjectB ())->getOrientation ();
	Matrix3x3 rotation1 = orientation1.getRotationMatrix ();
	Matrix3x3 rotation2 = orientation2.getRotationMatrix ();

	assert (rotation1.getSizeM () == 3 && rotation1.getSizeN () == 3);
	assert (rotation2.getSizeM () == 3 && rotation2.getSizeN () == 3);
	assert (mPositionBody1.getSizeM () == 3 && mPositionBody1.getSizeN () == 1);
	assert (mPositionBody2.getSizeM () == 3 && mPositionBody2.getSizeN () == 1);
	assert (position1.getSizeM () == 3 && position1.getSizeN () == 1);
	assert (position2.getSizeM () == 3 && position2.getSizeN () == 1);

 
	// Weltkoordinaten des Gelenkpunktes von Body1/2
	Matrix3x1 positionWorld1 = (rotation1 * mPositionBody1) + position1;
	Matrix3x1 positionWorld2 = (rotation2 * mPositionBody2) + position2;
	Matrix3x1 positionWorld = positionWorld1 - positionWorld2;
  
	//Ebenen-Bedingung anwenden
  
  
  
	//use .T() to realisze scalar product with matrices
	//Matrix<float> left = ((2*position1) + (rotation1 * mPositionBody1) - position2);
	//Matrix<float> right = (rotation1 * (mNormalPoint1 - mPositionBody1));
    
  
	// Matrix<float> ergebnis = left * right.T();
/*
  Matrix<float> hingeCondition1 = (position2 + 2*(rotation2*mPositionBody2) - position1).T() * ((rotation1*mPositionBody1).star() * (rotation1*mNormalPoint1));
  
  Matrix<float> hingeCondition2 = (position2 + (rotation2*mPositionBody2) + (rotation2*mNormalPoint2) - position1).T() * ((rotation1*mPositionBody1).star() * (rotation1*mNormalPoint1));
*/

	// geaendert hornung 040816
	Matrix3x1 normal1 = (rotation1 * mPositionBody1).star () *
		(rotation1 * mNormalPoint1);

	cout << "Achtung! Code auskommentiert in HingeConstraint::computeConstraint" << endl;
	//! \todo reparieren und wieder einkommentieren
	Matrix3x3 hingeCondition1;// = (rotation2 * mPositionBody2).T () * normal1;
	Matrix3x3 hingeCondition2;// = (rotation2 * mNormalPoint2 ).T () * normal1;
  
	Matrix5x1 resultConstraint;
	resultConstraint[0][0] = positionWorld[0][0];
	resultConstraint[1][0] = positionWorld[1][0];
	resultConstraint[2][0] = positionWorld[2][0];
	resultConstraint[3][0] = hingeCondition1[0][0];
	resultConstraint[4][0] = hingeCondition2[0][0];

	// Constraint: Diff Weltkoord. d. Gelenkpkt == 0
	mConstraint.setValues(resultConstraint);
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
		(angularVelocity1.star () * (rotation1 * mPositionBody1)) + velocity1;
	Matrix3x1 velocityWorld2 =
		(angularVelocity2.star () * (rotation2 * mPositionBody2)) + velocity2;
	Matrix3x1 velocityWorld = velocityWorld1 - velocityWorld2;
  
 
  
	//use .T() to realisze scalar product with matrices
	hingeCondition1 = 0;
	hingeCondition2 = 0;
  
	/* 
	   hingeCondition1 =  (velocity2 + 2*(angularVelocity2.star() * (rotation2*mPositionBody2)) - velocity1).T() * ((rotation1*mPositionBody1).star() * (rotation1*mNormalPoint1)) +
	   (position2 + 2*(rotation2*mPositionBody2) - position1).T() * (((angularVelocity1.star() * (rotation1*mPositionBody1)).star() * (rotation1*mNormalPoint1)) + ((rotation1*mPositionBody1).star() * (angularVelocity1.star() * (rotation1*mNormalPoint1))));
  
  
	   hingeCondition2 =  (velocity2 + (angularVelocity2.star() * (rotation2*mPositionBody2)) + (angularVelocity2.star() * (rotation2*mNormalPoint2)) - velocity1).T() * ((rotation1*mPositionBody1).star() * (rotation1*mNormalPoint1)) +
	   (position2 + (rotation2*mPositionBody2) + (rotation2*mNormalPoint2) - position1).T() * (((angularVelocity1.star() * (rotation1*mPositionBody1)).star() * (rotation1*mNormalPoint1)) + ((rotation1*mPositionBody1).star() * (angularVelocity1.star() * (rotation1*mNormalPoint1))));
	*/

	// geaendert hornung 040816
	Matrix3x3 rotation1Dot = angularVelocity1.star () * rotation1;
	Matrix3x3 rotation2Dot = angularVelocity2.star () * rotation2;
	Matrix3x1 normal1Dot = ((rotation1Dot * mPositionBody1).star () *
							(rotation1    * mNormalPoint1 )) +
		((rotation1    * mPositionBody1).star () *
		 (rotation1Dot * mNormalPoint1 ));

	cout << "Achtung! Code auskommentiert in HingeConstraint::computeConstraint" << endl;
	//! \todo reparieren und wieder einkommentieren
	Matrix3x1 hingeCondition1Dot; //= ((rotation2Dot * mPositionBody2).T () * normal1) +
//		((rotation2    * mPositionBody2).T () * normal1Dot);

	Matrix3x1 hingeCondition2Dot;// = ((rotation2Dot * mNormalPoint2 ).T () * normal1) +
	//((rotation2    * mNormalPoint2 ).T () * normal1Dot);
  
	Matrix5x1 resultConstraintDot;

	// ConstraintDot: Relative Geschw. der Gelenkpunkte (in Weltkoord.)
	resultConstraintDot[0][0] = velocityWorld[0][0];
	resultConstraintDot[1][0] = velocityWorld[1][0];
	resultConstraintDot[2][0] = velocityWorld[2][0];
	resultConstraintDot[3][0] = hingeCondition1Dot[0][0]; // Dot angefuegt
	resultConstraintDot[4][0] = hingeCondition2Dot[0][0]; //
  
	mConstraintDot.setValues(resultConstraintDot);

}


void HingeConstraint::swapBodies() {
	PrimaryConstraint::swapBodies();

	//! \todo genau so stehts in der BallAndSocket... sollte das dann nicht in Primary const. ?
	Matrix3x1 temp1 = mPositionBody1;
	Matrix3x1 temp2 = mPositionBody2;
	mPositionBody1 = temp2;
	mPositionBody2 = temp1;
}
//@}

void HingeConstraint::computeJacobian () {

	Matrix5x6 
		jacobian1,
		jacobian2,
		jacobian1Dot,
		jacobian2Dot;
	Matrix3x3 
		temporal;
	Matrix3x6
		jacobian1BallAndSocket,
		jacobian2BallAndSocket,
		jacobian1DotBallAndSocket,
		jacobian2DotBallAndSocket;
	Quaternion 
		orientation1 = boost::static_pointer_cast <RigidBody>
		(getObjectA ())->getOrientation (),
		orientation2 = boost::static_pointer_cast <RigidBody>
		(getObjectB ())->getOrientation ();
	Matrix3x1 
		euler1 (orientation1.getEulerRotation ()),
		euler2 (orientation2.getEulerRotation ());
	Matrix3x3 
		rotation1 (orientation1.getRotationMatrix()),
		rotation2 (orientation2.getRotationMatrix());
	Matrix3x1
		omega1 = boost::static_pointer_cast <RigidBody>
		(getObjectA ())->getAngularVelocity (),
		omega2 = boost::static_pointer_cast <RigidBody>
		(getObjectB ())->getAngularVelocity ();

	jacobian1BallAndSocket [0][0] = 1.0f;
	jacobian1BallAndSocket [1][1] = 1.0f;
	jacobian1BallAndSocket [2][2] = 1.0f;

	temporal = ((rotation1 * mPositionBody1).star ()).T ();

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			jacobian1BallAndSocket [i][j+3] = temporal [i][j];

	jacobian2BallAndSocket [0][0] = 1.0f;
	jacobian2BallAndSocket [1][1] = 1.0f;
	jacobian2BallAndSocket [2][2] = 1.0f;

	temporal = ((rotation2 * mPositionBody2).star ()).T ();

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			jacobian2BallAndSocket [i][j+3] = temporal [i][j];

	jacobian2BallAndSocket = -1.0f * jacobian2BallAndSocket;

	jacobian1DotBallAndSocket = jacobian1BallAndSocket;
	jacobian2DotBallAndSocket = jacobian2BallAndSocket;

	jacobian1DotBallAndSocket [0][0] = 0.0f;
	jacobian1DotBallAndSocket [1][1] = 0.0f;
	jacobian1DotBallAndSocket [2][2] = 0.0f;

	jacobian2DotBallAndSocket [0][0] = 0.0f;
	jacobian2DotBallAndSocket [1][1] = 0.0f;
	jacobian2DotBallAndSocket [2][2] = 0.0f;

	Matrix3x1 normal1 = (rotation1 * mPositionBody1).star () *
		(rotation1 * mNormalPoint1);

	Matrix3x3 dNormal1DOrientation =
		((rotation1 * mPositionBody1).star () *
		 (rotation1 * mNormalPoint1).star ().T ()) -
		((rotation1 * mNormalPoint1).star () *
		 (rotation1 * mPositionBody1).star ().T ());

	// this might be a new operator... don't you think?
	Matrix3x1 
		dNormal1DOrientation1,
		dNormal1DOrientation2,
		dNormal1DOrientation3;
	for (unsigned int i=0; i<3; i++) {
		dNormal1DOrientation1 [i][0] = dNormal1DOrientation [i][0];
		dNormal1DOrientation2 [i][0] = dNormal1DOrientation [i][1];
		dNormal1DOrientation3 [i][0] = dNormal1DOrientation [i][2];
	}
	Matrix1x1 
		temporal1 = (rotation2 * mPositionBody2).T () * dNormal1DOrientation1,
		temporal2 = (rotation2 * mPositionBody2).T () * dNormal1DOrientation2,
		temporal3 =	(rotation2 * mPositionBody2).T () * dNormal1DOrientation3;
	Matrix3x1 dHinge1DOrientationBody1;
	dHinge1DOrientationBody1 [0][0] = temporal1 [0][0];
	dHinge1DOrientationBody1 [0][1] = temporal2 [0][0];
	dHinge1DOrientationBody1 [0][2] = temporal3 [0][0];

                                                                                                                                                            
	cout << "Achtung! Auskommentierter Code in HingeConstraint::computeJacobian " << endl;
	//! \todo reparieren und wieder rein packen
/*

	Matrix3x1 dHinge1DOrientationBody2 =
		((rotation2 * mPositionBody2).star () * normal1).T ();

	// this might be a new operator... don't you think?
	temporal1 = (rotation2 * mNormalPoint2).T () * dNormal1DOrientation1;
	temporal2 = (rotation2 * mNormalPoint2).T () * dNormal1DOrientation2;
	temporal3 = (rotation2 * mNormalPoint2).T () * dNormal1DOrientation3;
	Matrix1x3 dHinge2DOrientationBody1;
	dHinge2DOrientationBody1 [0][0] = temporal1 [0][0];
	dHinge2DOrientationBody1 [0][1] = temporal2 [0][0];
	dHinge2DOrientationBody1 [0][2] = temporal3 [0][0];

	Matrix1x3 dHinge2DOrientationBody2 =
		((rotation2 * mNormalPoint2).star () * normal1).T ();

	for (unsigned int i=0; i<3; i++)
		for (unsigned int j=0; j<6; j++) {
			jacobian1 [i][j] = jacobian1BallAndSocket [i][j];
			jacobian2 [i][j] = jacobian2BallAndSocket [i][j];
		}


	for (unsigned int i=0; i<3; i++) {
		jacobian1 [3][i+3] = dHinge1DOrientationBody1 [0][i];
		jacobian2 [3][i+3] = dHinge1DOrientationBody2 [0][i];
		jacobian1 [4][i+3] = dHinge2DOrientationBody1 [0][i];
		jacobian2 [4][i+3] = dHinge2DOrientationBody2 [0][i];
	}

	jacobian1DotBallAndSocket = omega1.star () * jacobian1DotBallAndSocket;
	jacobian2DotBallAndSocket = omega2.star () * jacobian2DotBallAndSocket;

	// here we go again, uff.
	temporal1 = (omega2.star () * rotation2 * mPositionBody2).T () *
		dNormal1DOrientation1;
	temporal2 = (omega2.star () * rotation2 * mPositionBody2).T () *
		dNormal1DOrientation2;
	temporal3 = (omega2.star () * rotation2 * mPositionBody2).T () *
		dNormal1DOrientation3;
	Matrix1x3 dHinge1DotDOrientationBody1;
	dHinge1DotDOrientationBody1 [0][0] = temporal1 [0][0];
	dHinge1DotDOrientationBody1 [0][1] = temporal2 [0][0];
	dHinge1DotDOrientationBody1 [0][2] = temporal3 [0][0];

	Matrix1x3 dNormal1DotDOrientation =
		((rotation1 * mPositionBody1).star () * omega1.star () *
		 (rotation1 * mNormalPoint1 ).star ().T ()) -
		((rotation1 * mNormalPoint1 ).star () * omega1.star () *
		 (rotation1 * mPositionBody1).star ().T ());

	// here we go again, uff
	Matrix3x1 dNormal1DotDOrientation1;
	Matrix3x1 dNormal1DotDOrientation2;
	Matrix3x1 dNormal1DotDOrientation3;
	for (unsigned int i=0; i<3; i++) {
		dNormal1DotDOrientation1 [i][0] = dNormal1DotDOrientation [i][0];
		dNormal1DotDOrientation2 [i][0] = dNormal1DotDOrientation [i][1];
		dNormal1DotDOrientation3 [i][0] = dNormal1DotDOrientation [i][2];
	}
	temporal1 = (rotation2 * mPositionBody2).T () * dNormal1DotDOrientation1;
	temporal2 = (rotation2 * mPositionBody2).T () * dNormal1DotDOrientation2;
	temporal3 = (rotation2 * mPositionBody2).T () * dNormal1DotDOrientation3;

	dHinge1DotDOrientationBody1 [0][0] = dHinge1DotDOrientationBody1 [0][0] +
		temporal1 [0][0];
	dHinge1DotDOrientationBody1 [0][1] = dHinge1DotDOrientationBody1 [0][1] +
		temporal2 [0][0];
	dHinge1DotDOrientationBody1 [0][2] = dHinge1DotDOrientationBody1 [0][2] +
		temporal3 [0][0];

	Matrix3x3 rotation1Dot = omega1.star () * rotation1;
	Matrix3x3 normal1Dot = ((rotation1Dot * mPositionBody1).star () *
								 (rotation1    * mNormalPoint1 )) +
		((rotation1    * mPositionBody1).star () *
		 (rotation1Dot * mNormalPoint1 ));

	Matrix1x3 dHinge1DotDOrientationBody2 =
		(omega2.star () * (rotation2 * mPositionBody2).star () * normal1).T () +
		((rotation2 * mPositionBody2).star () * normal1Dot).T ();

	// and again...
	temporal1 = (omega2.star () * rotation2 * mNormalPoint2).T () *
		dNormal1DOrientation1;
	temporal2 = (omega2.star () * rotation2 * mNormalPoint2).T () *
		dNormal1DOrientation2;
	temporal3 = (omega2.star () * rotation2 * mNormalPoint2).T () *
		dNormal1DOrientation3;
	Matrix1x3 dHinge2DotDOrientationBody1;
	dHinge2DotDOrientationBody1 [0][0] = temporal1 [0][0];
	dHinge2DotDOrientationBody1 [0][1] = temporal2 [0][0];
	dHinge2DotDOrientationBody1 [0][2] = temporal3 [0][0];

	temporal1 = (rotation2 * mNormalPoint2).T () * dNormal1DotDOrientation1;
	temporal2 = (rotation2 * mNormalPoint2).T () * dNormal1DotDOrientation2;
	temporal3 = (rotation2 * mNormalPoint2).T () * dNormal1DotDOrientation3;

	dHinge2DotDOrientationBody1 [0][0] = dHinge1DotDOrientationBody1 [0][0] +
		temporal1 [0][0];
	dHinge2DotDOrientationBody1 [0][1] = dHinge1DotDOrientationBody1 [0][1] +
		temporal2 [0][0];
	dHinge2DotDOrientationBody1 [0][2] = dHinge1DotDOrientationBody1 [0][2] +
		temporal3 [0][0];

	Matrix1x3 dHinge2DotDOrientationBody2 =
		(omega2.star () * (rotation2 * mNormalPoint2).star () * normal1).T () +
		((rotation2 * mNormalPoint2).star () * normal1Dot).T ();

	// schreiben jacobian?Dot
	for (unsigned int i=0; i<3; i++)
		for (unsigned int j=0; j<6; j++) {
			jacobian1Dot [i][j] = jacobian1DotBallAndSocket [i][j];
			jacobian2Dot [i][j] = jacobian2DotBallAndSocket [i][j];
		}

	for (unsigned int i=0; i<3; i++) {
		jacobian1Dot [3][i+3] = dHinge1DotDOrientationBody1 [0][i];
		jacobian1Dot [4][i+3] = dHinge2DotDOrientationBody1 [0][i];
		jacobian2Dot [3][i+3] = dHinge1DotDOrientationBody2 [0][i];
		jacobian2Dot [4][i+3] = dHinge2DotDOrientationBody2 [0][i];
	}
 
	//overwrite
	mJacobian1 = jacobian1;
	mJacobian2 = jacobian2;
	mJacobian1Dot = jacobian1Dot;
	mJacobian2Dot = jacobian2Dot;

	//PrimaryConstraint::computeJacobian ();

*/
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
void HingeConstraint::computeRightHandSide () {

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
	Matrix6x1 spaceForce1;
	Matrix6x1 spaceForce2;
	Matrix6x6 spaceMassInv1;
	Matrix6x6 spaceMassInv2;
	Matrix6x1 spaceVelocity1;
	Matrix6x1 spaceVelocity2;

	for (unsigned int i=0; i<3; i++) {
		for (unsigned int j=0; j<3; j++) {
			spaceMassInv1[i][j] = 0.0f;
			spaceMassInv2 [i]  [j]   = 0;
			spaceMassInv1 [i]  [j+3] = 0;
	
			spaceMassInv2 [i+3][j]   = 0;
			spaceMassInv1 [i+3][j+3] = inertiaInv1 [i][j];
			spaceMassInv2 [i+3][j+3] = inertiaInv1 [i][j];
	
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
	Matrix5x1 rightAddend1 = mJacobian1 * spaceAcceleration1;
	Matrix5x1 rightAddend2 = mJacobian2 * spaceAcceleration2;

	// linker Summand
	Matrix5x1 leftAddend1 = mJacobian1Dot * spaceVelocity1;
	Matrix5x1 leftAddend2 = mJacobian2Dot * spaceVelocity2;

	// Summe
	Matrix5x1 sum1 = leftAddend1 + rightAddend1;
	Matrix5x1 sum2 = leftAddend2 + rightAddend2;
  
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
void HingeConstraint::computeRightHandSideFast () {

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
void HingeConstraint::updateRigidBodies ()
{

	Matrix6x1 spaceForce1;
	Matrix6x1 spaceForce2;

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


Matrix6x6 HingeConstraint::getJacobian1() {
	Matrix6x6 temporal(0.0f);
	temporal.setValues(mJacobian1);
	return temporal;
}

Matrix6x6 HingeConstraint::getJacobian2() {
	Matrix6x6 temporal(0.0f);
	temporal.setValues(mJacobian2);
	return temporal;
}

Matrix6x1 HingeConstraint::getRightHandSide1() {
	Matrix6x1 temporal(0.0f);
	temporal.setValues(mRightHandSide1);
	return temporal;
}

Matrix6x1 HingeConstraint::getRightHandSide2() {
	Matrix6x1 temporal(0.0f);
	temporal.setValues(mRightHandSide2);
	return temporal;
}

Matrix6x1 HingeConstraint::getConstraint() {
	Matrix6x1 temporal(0.0f);
	temporal.setValues(mConstraint);
	return temporal;
}

void HingeConstraint::setLambda(const Matrix6x1 &lambda) {
	lambda.getMxNMatrix(0,0,mLambda);
}
