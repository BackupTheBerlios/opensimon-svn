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

/**
 *  \class PrimaryConstraint
 *  \author Palmer, Hornung
 *
 * \brief Abstrakte Klasse, die eine Zwangsbedingung darstellt.
 *
 * Die Klasse PrimaryConstraint verwaltet Daten und leitet Informationen ab,
 * die für die Berechnung der Zwangskräfte auf Festkörper oder Partikel
 * benötigt werden. Siehe auch: ConstraintSystem, ConstraintMatrixBlock.
 * Aus der Klasse PrimaryConstraint können beliebige Klassen fuer
 * Zwangsbedingungen abgeleitet werden. In einer solchen Ableitung muss
 * speziell die virtuelle Funktion computeConstraint () definiert werden.
 */

// ////////////////////////////////////////////////////////////////////////////

// ----------------------------------------------------------------------------
//  \brief Includes
// ----------------------------------------------------------------------------
#include <simon/PrimaryConstraint.h>
#include <iostream>  // NUR VORLAEUFIG!!!

#include <simon/SmartPointer.h>

using namespace std;

// ----------------------------------------------------------------------------
/**
 * \brief Default-Constructor.
 *
 * Der Konstruktor macht bislang nichts.
 */
// ----------------------------------------------------------------------------
PrimaryConstraint::PrimaryConstraint () : Connection()
{

  // nn

}

// ----------------------------------------------------------------------------
/**
 * \brief Constructor.
 *
 * Der Konstruktor initialisiert den (binaeren) Constraint mit den beteiligten
 * Festkörpern.
 *
 * \param body1 Erster  beteiligter Festkörper
 * \param body2 Zweiter beteiligter Festkörper 
 */
// ----------------------------------------------------------------------------
PrimaryConstraint::PrimaryConstraint (RigidBodyPtr objectA,
                                      RigidBodyPtr objectB) :
	Connection(objectA, objectB),
	mTau(100.0),

	mRBAngularVelocity1(0.0),
	mRBAngularVelocity2(0.0),
	mRBTorque1(0.0),
	mRBTorque2(0.0),
	mRBAngularAcceleration1(0.0),
	mRBAngularAcceleration2(0.0),

	mTemporal3x1Matrix1(0.0),
	mTemporal3x1Matrix2(0.0),
	mTemporal3x1Matrix3(0.0),
	mTemporal3x1Matrix4(0.0),

	mTemporal3x1Matrix(0.0),

	mRBRotationMatrix1(0.0),
	mRBRotationMatrix2(0.0),
	mRBInertiaInv1(0.0),
	mRBInertiaInv2(0.0),

	mRBJacobian1Part1(0.0),
	mRBJacobian1Part2(0.0),
	mRBJacobian2Part1(0.0),
	mRBJacobian2Part2(0.0),
	mRBJacobian1DotPart1(0.0),
	mRBJacobian1DotPart2(0.0),
	mRBJacobian2DotPart1(0.0),
	mRBJacobian2DotPart2(0.0),

	mTemporal3x3Matrix(0.0)

{
	
	// defining the size and startvalues of in-house-keeped vars
 
	mRBPosition1 = Vec3(0,0,0);
	mRBPosition2 = Vec3(0,0,0);
	mRBVelocity1 = Vec3(0,0,0);
	mRBVelocity2 = Vec3(0,0,0);
	mRBForce1 = Vec3 (0.0,0.0,0.0);
	mRBForce2 = Vec3 (0.0,0.0,0.0);
	mRBAcceleration1 = Vec3 (0.0,0.0,0.0);
	mRBAcceleration2 = Vec3 (0.0,0.0,0.0);

	mRBOrientation1 = Quaternion();
	mRBOrientation2 = Quaternion();

	mRB1 = objectA;
	mRB2 = objectB;

	
}

// ----------------------------------------------------------------------------
/// Destructor.
// ----------------------------------------------------------------------------
PrimaryConstraint::~PrimaryConstraint ()
{

  // nn

}

float PrimaryConstraint::getTau() {
	return mTau;
}

void PrimaryConstraint::setTau(float tau) {
	mTau = tau;
}

//! initialize static variable ...
bool PrimaryConstraint::mDoBaumgarteStabilisation = true;

/**
* \brief toggle Baumgarte stabilisation
*/
void PrimaryConstraint::doBaumgarteStabilisation(bool doBaumgarte) {
	mDoBaumgarteStabilisation = doBaumgarte;
}

/** 
* \brief get Baumgarte stabilisation state
*/
bool PrimaryConstraint::doesBaumgarteStabilisation() {
	return mDoBaumgarteStabilisation;
}

void PrimaryConstraint::compute() {
	computeConstraint();
	computeJacobian();
	computeRightHandSide();
}

// ----------------------------------------------------------------------------
/// \brief Update members: constraint and constraintDot.
///
/// Die abstrakte Funktion wird pro Simulationsschritt aufgerufen, um den
/// Constraint-Vektor zu berechnen. In jeder Ableitung der Klasse muss
/// speziell diese Funktion definiert werden.
// ----------------------------------------------------------------------------
void PrimaryConstraint::computeConstraint ()
{

  // nn

}



// ----------------------------------------------------------------------------
/// \brief Update members: jacobian and jacobianDot.
///
/// Die Funktion wird pro Simulationsschritt aufgerufen, um aus dem
/// Constraint-Vektor \f$\mathbf{C}\f$ die Jacobsche Matrix \f$\mathbf{J}_i\f$
/// --- sowie ihre Ableitung nach der Zeit --- zu berechnen,
/// \f$i \in \{1, 2\}\f$ für die beiden Festkörper (resp. Partikel). Dazu wird
/// der Vektor partiell nach Position \f$\mathbf{x}_i\f$ bzw. Orientierung
/// \f$\mbox{\boldmath$\varphi$}_i\f$ der beteiligten Festkörper abgeleitet,
/// also \f[\mathbf{J}_i = \frac{\partial \mathbf{C}}{\partial
/// \mathbf{Y}_i} \qquad \mbox{analog}\quad \mathbf{\dot{J}}_i =
/// \frac{\partial \mathbf{\dot{C}}}{\partial \mathbf{Y}_i}\f] für
/// \f$\mathbf{Y}_i = {{\mathbf{x}_i} \choose \mbox{\boldmath$\varphi$}_i}\f$.
/// Siehe dazu auch W. H. Press et al. "Numerical Recipes", Kap. 5.7.
/// Verbesserung: Leitet nicht nach dem "phase-space", sondern nach dem reinen
/// Positionsvektor ab!!!
///
/// \todo computeJacobianDot...
/// \todo das ganze ist überhaupt noch nicht getestet!!!
// ----------------------------------------------------------------------------
void PrimaryConstraint::computeJacobian () {


	if (mStepSize != 0.0) {
		SimonState::exemplar()->errors << "Simon: Constraints: ";
		SimonState::exemplar()->errors << "PrimaryConstraint: ";
		SimonState::exemplar()->errors << "computeJacobian:\n";
		SimonState::exemplar()->errors << "  stepSize is zero. This will cause ";
		SimonState::exemplar()->errors << "a division by zero.\n";
		SimonState::exemplar()->errors << SimonState::endm;
	}
	assert (mStepSize != 0.0);

	//! \todo wird das noch verwendet? sollte diese Funktion nicht einfach raus fliegen?
	assert(false); // ab hier wirds ungemütlich (alte Matrizen)
	/*
	// Initialisierung der Jakobschen Matrizen in der richtigen Groesse
	mJacobian1 = Matrix <float> (mConstraint.getSizeM (), 6);
	mJacobian2 = Matrix <float> (mConstraint.getSizeM (), 6);
	mJacobian1Dot = Matrix <float> (mConstraint.getSizeM (), 6);
	mJacobian2Dot = Matrix <float> (mConstraint.getSizeM (), 6);

	// Erstellung der 12x1 Matrix spacePosition
	Matrix <float> position1 ((getObjectA ())->getPosition ());
	Matrix <float> position2 ((getObjectB ())->getPosition ());
	Quaternion orientation1 = boost::static_pointer_cast <RigidBody> (getObjectA ())->getOrientation();
	Quaternion orientation2 = boost::static_pointer_cast <RigidBody>
	(getObjectB ())->getOrientation ();
	Matrix <float> euler1 (orientation1.getEulerRotation ());
	Matrix <float> euler2 (orientation2.getEulerRotation ());
	Matrix <float> spacePosition (12, 1);
	for (unsigned int i=0; i<3; i++) {
    spacePosition [i]  [0] = position1 [i][0];
    spacePosition [i+6][0] = position2 [i][0];
    spacePosition [i+3][0] = euler1    [i][0];
    spacePosition [i+9][0] = euler2    [i][0];
	}

	// Erstellung der 12x1 Matrix spaceVelocity
	Matrix <float> velocity1 ((getObjectA ())->getVelocity ());
	Matrix <float> velocity2 ((getObjectB ())->getVelocity ());
	Matrix <float> angularVelocity1 (boost::static_pointer_cast <RigidBody>
	(getObjectA ())->getAngularVelocity ());
	Matrix <float> angularVelocity2 (boost::static_pointer_cast <RigidBody>
	(getObjectB ())->getAngularVelocity ());
	Matrix <float> spaceVelocity (12, 1);
	for (unsigned int i=0; i<3; i++) {
    spaceVelocity [i]  [0] = velocity1        [i][0];
    spaceVelocity [i+6][0] = velocity2        [i][0];
    spaceVelocity [i+3][0] = angularVelocity1 [i][0];
    spaceVelocity [i+9][0] = angularVelocity2 [i][0];
	}

	double h; // zur Initialisierung der Schrittweite
	Matrix <float> spacePositionPlusH;
	Matrix <float> spacePositionMinusH;
	Matrix <float> spaceVelocityPlusH;
	Matrix <float> spaceVelocityMinusH;
	Matrix <float> constraintPlusH;
	Matrix <float> constraintMinusH;
	Matrix <float> constraintDotPlusH;
	Matrix <float> constraintDotMinusH;

  

	// Berechne Jakobsche Matrizen, d.h. partielle Ableitung (nur nach spacePos.)
	for (unsigned int j=0; j<6; j++) {

    // (in-)finite Abweichungen
    spacePositionPlusH  = spacePosition;
    spacePositionMinusH = spacePosition;

    assert (spacePositionPlusH.getSizeM () == 12 &&
	spacePositionPlusH.getSizeN () == 1);
    assert (spacePositionMinusH.getSizeM () == 12 &&
	spacePositionMinusH.getSizeN () == 1);

    spacePositionPlusH  [j][0] += (float)mStepSize;
    spacePositionMinusH [j][0] -= (float)mStepSize;
    
    constraintPlusH  = computeConstraint(spacePositionPlusH,  spaceVelocity);
    constraintMinusH = computeConstraint(spacePositionMinusH, spaceVelocity);
    constraintDotPlusH  = computeConstraintDot (spacePositionPlusH,
	spaceVelocity);
    constraintDotMinusH = computeConstraintDot (spacePositionMinusH,
	spaceVelocity);

    assert (constraintPlusH.getSizeM () == mConstraint.getSizeM ());
    assert (constraintPlusH.getSizeN () == 1);

    // Initialisierung von StepSize (siehe "Numerical Recipes")
    h = spacePositionPlusH [j][0] - spacePosition [j][0];

    
    
    for (unsigned int i=0; i<mConstraint.getSizeM (); i++) {

	// Um diesen Wert ging es eigentlich!
	mJacobian1 [i][j] = constraintPlusH [i][0] - constraintMinusH [i][0];
	mJacobian1 [i][j] *= (float)(0.5 / h);
	mJacobian1Dot [i][j] =
	constraintDotPlusH [i][0] - constraintDotMinusH [i][0];
	mJacobian1Dot [i][j] *= (float)(0.5 / h);

	}
    
    
    
    // und das ganze noch fuer Koerper2 !!!
	// (in-)finite Abweichungen
    spacePositionPlusH  = spacePosition;
    spacePositionMinusH = spacePosition;
    spacePositionPlusH  [j+6][0] += (float)mStepSize;
    spacePositionMinusH [j+6][0] -= (float)mStepSize;
    constraintPlusH  = computeConstraint(spacePositionPlusH,  spaceVelocity);
    constraintMinusH = computeConstraint(spacePositionMinusH, spaceVelocity);
    constraintDotPlusH  = computeConstraintDot (spacePositionPlusH,
	spaceVelocity);
    constraintDotMinusH = computeConstraintDot (spacePositionMinusH,
	spaceVelocity);
    // Initialisierung von StepSize (siehe "Numerical Recipes")
    h = spacePositionPlusH [j+6][0] - spacePosition [j+6][0];

    
    
    for (unsigned int i=0; i<mConstraint.getSizeM (); i++) {

	// Um diesen Wert ging es eigentlich! (vgl. oben)
	mJacobian2 [i][j] = constraintPlusH [i][0] - constraintMinusH [i][0];
	mJacobian2 [i][j] *= (float)(0.5 / h);
	mJacobian2Dot [i][j] =
	constraintDotPlusH [i][0] - constraintDotMinusH [i][0];
	mJacobian2Dot [i][j] *= (float)(0.5 / h);

    }

	}
	*/
}


// ----------------------------------------------------------------------------
/// \brief Update members: Inhouse Keeping
/// Diese Funktion übernimmt die Aktualisierung der vorgehaltenen
/// Festkörpereigenschaften, die in fast-Funktionen verwendet werden, um
/// Kopieren zu vermeiden (Geschwindigkeitsgewinn).
// ----------------------------------------------------------------------------
void PrimaryConstraint::updateInhouseKeeping () {

  //cout << "PrimaryConstraint::updateInhouseKeeping 1\n";

  mRB1->getVelocity (mRBVelocity1);
  mRB2->getVelocity (mRBVelocity2);
  mRB1->getForce    (mRBForce1);
  mRB2->getForce    (mRBForce2);

  //cout << "PrimaryConstraint::updateInhouseKeeping 2\n";

  mRB1->getPosition (mRBPosition1);
  mRB2->getPosition (mRBPosition2);
  mRB1->getOrientation (mRBOrientation1);
  mRB2->getOrientation (mRBOrientation2);
  mRBOrientation1.getRotationMatrix (mRBRotationMatrix1);
  mRBOrientation2.getRotationMatrix (mRBRotationMatrix2);

  //cout << "PrimaryConstraint::updateInhouseKeeping 3\n";

  mRB1->getAngularVelocity (mRBAngularVelocity1);
  mRB2->getAngularVelocity (mRBAngularVelocity2);

  //cout << "PrimaryConstraint::updateInhouseKeeping 4\n";

  mRB1->getTorque (mRBTorque1);
  mRB2->getTorque (mRBTorque2);

  //cout << "PrimaryConstraint::updateInhouseKeeping 4\n";

  mRB1->getInvWorldInertiaTensor (mRBInertiaInv1);
  mRB2->getInvWorldInertiaTensor (mRBInertiaInv2);

  //cout << "PrimaryConstraint::updateInhouseKeeping end\n";

/*

  mRBJacobian1
  mRBJacobian2
  mRBJacobian1Dot
  mRBJacobian2Dot

  mTemporal3x3Matrix

*/

}

// ----------------------------------------------------------------------------
/// \brief Update all: Simulation.
///
/// Die Funktion bietet ein Interface nach außen, indem sie alle an einem
/// Simulationsschritt beteiligten Operationen in der richtigen
/// Reihenfolge ausführt.
// ----------------------------------------------------------------------------
void PrimaryConstraint::simulate () {

  computeConstraint ();
  computeJacobian ();
  computeRightHandSide ();
}


//@{
/// \brief Convenient functions
///
/// Get- und set-Funktionen: setStepSize (h) für die partielle Ableitung,
/// getConstraint, getConstraintDot, getJacobian<i>[Dot] mit i = 1,2.
void PrimaryConstraint::setStepSize (double h)
{mStepSize = h;}


PrimaryConstraint::DegreeOfFreedom PrimaryConstraint::getDegreeOfFreedom() {
	return dof0;
}

//@}

void PrimaryConstraint::swapBodies() {
	// \todo nur noch die rigidbodys verwenden

	WorldObjectPtr temp1 = getObjectA();
	WorldObjectPtr temp2 = getObjectB();
	setObjectA(temp2);
	setObjectB(temp1);

	
	RigidBodyPtr tempRB = mRB1;
	mRB1 = mRB2;
	mRB2 = tempRB; 

}

RigidBodyPtr PrimaryConstraint::getRigidBodyA(){
	return mRB1;
}

RigidBodyPtr PrimaryConstraint::getRigidBodyB(){
	return mRB2;
}
