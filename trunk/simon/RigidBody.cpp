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


#include <simon/RigidBody.h>
#include <simon/SimonState.h>

#include <simon/config.h>

using namespace std;
//------------------------------------------------------------------------------
/**
 * \class RigidBody
 * \brief Rigid-Body Klasse für Festkörperberechnungen.
 *        Basisklasse für Sphere, Box, Plane etc.
 *
 */
//------------------------------------------------------------------------------


/**
* \brief Konstruktor initialisiert den RigidBody mit default-werten
* 
* \todo ist der Defaultwert für InertiaTensor überhaupt sinnvoll? 
*
* - InertiaTensor mit Identität 
* - OrientierungsQuaternion Drehung um null Grad um y-Achse
* - AngularVelocity auf null
* - AngularAcceleration auf null
* - Torque auf null
*/
RigidBody::RigidBody() : WorldObject() {

	//Default InvInertiaTensor setzen
	mInvInertiaTensor = Matrix3x3::createIdentity();
	
	//Default Orientierung setzen
	mOrientation = Quaternion(0.0, Vec3(0.0, 1.0, 0.0));
	mPrevOrientation = Quaternion(0.0, Vec3(0.0, 1.0, 0.0));
	
	//Default AngularVelocity setzen
	mAngularVelocity = Vec3(0.0, 0.0, 0.0);
	mPrevAngularVelocity = Vec3(0.0, 0.0, 0.0);

	mAngularMomentum = Vec3(0.0, 0.0, 0.0);
	mPrevAngularMomentum = Vec3(0.0, 0.0, 0.0);

	//Default AngularAcceleration setzen
	mAngularAcceleration = Vec3(0.0, 0.0, 0.0);
	mPrevAngularAcceleration = Vec3(0.0, 0.0, 0.0);
	
	//Default Torque setzen
	mTorque = Vec3(0.0, 0.0, 0.0);
	mPrevTorque = Vec3(0.0, 0.0, 0.0);
}

/**
* \brief Destruktor
*/
RigidBody::~RigidBody(){

}


// *********************************
// S E T T E R
// *********************************




/**
* \brief Setzt die Orientierung als Quaternion
*/
void RigidBody::setOrientation(Quaternion orientation) {
	mPrevOrientation = mOrientation;
	orientation.normalize();
	mOrientation = orientation;
}

/**
* \brief Setzt die vorherige Orientierung als Quaternion
*/
void RigidBody::setPrevOrientation(Quaternion prevOrientation) {
	mPrevOrientation = prevOrientation;
}

/**
* \brief Macht den letzten aufruf von setOrientation rückgängig.
*/
void RigidBody::undoSetOrientation()
{
	mOrientation = mPrevOrientation;
}



/**
* \brief Setzt das Drehmoment als Vektor
* \todo Für welchen Fall gilt die AngularAcceleration Berechnung? Keyword: Drehachsen?
*/
void RigidBody::setTorque(Vec3 torque) {
	mTorque = torque;
	if (torque.length() == 0){
		mAngularAcceleration[0] = 0;
		mAngularAcceleration[1] = 0;
		mAngularAcceleration[2] = 0;
		return;
	}

	/*
	//neue angularAcceleration setzen
	Matrix3x1 tempA = getInvWorldInertiaTensor() * mTorque;
	Matrix3x1 test(getAngularVelocity());
	Matrix3x3 star = (test).star();
	Matrix3x1 tempB = (star * getWorldInertiaTensor());
	Matrix3x1 angularAcceleration = (
		 tempA - 
		(getInvWorldInertiaTensor() * 
		 (
			 (tempB * (Matrix3x1 (getAngularVelocity()))
				 )))
);
		mAngularAcceleration[0] = angularAcceleration[0][0];
	mAngularAcceleration[1] = angularAcceleration[1][0];
	mAngularAcceleration[2] = angularAcceleration[2][0];
	*/
	cout << "Achtung! Angular Acceleration wird nicht geupdatet (in setTorque(..). Viel Spass." << endl;
}

/**
* \brief Setzt das Winkelgeschwindigkeit als Vektor
*/
void RigidBody::setAngularVelocity(Vec3 angularVelocity)
{
	assert(!ISNAN(angularVelocity[0]) || 
		   !ISNAN(angularVelocity[1]) || 
		   !ISNAN(angularVelocity[2]));
	assert(!ISINF(angularVelocity[0]) || 
		   !ISINF(angularVelocity[1]) || 
		   !ISINF(angularVelocity[2]));

	mPrevAngularVelocity = mAngularVelocity;
	mPrevAngularMomentum = mAngularMomentum;
	mAngularVelocity = angularVelocity;
	if (angularVelocity.length() != 0)
		mAngularMomentum = Vec3(getWorldInertiaTensor() * angularVelocity);
	else 
		mAngularMomentum = Vec3(0.0, 0.0, 0.0);
}

/**
* \brief Setzt die vorherige Winkelgeschwindigkeit als Vektor
* \TODO brauchen wir diese funktion überhaupt ?
*/ 
void RigidBody::setPrevAngularVelocity(Vec3 prevAngularVelocity) {
	mPrevAngularVelocity = prevAngularVelocity;
	if (prevAngularVelocity.length() != 0)
		mPrevAngularMomentum = getWorldInertiaTensor() * prevAngularVelocity;
	else 
		mPrevAngularMomentum = Vec3(0.0, 0.0, 0.0);
}

/**
* \brief Macht den letzten aufruf von setAngularVelocity rückgängig.
*/
void RigidBody::undoSetAngularVelocity(){

	mAngularVelocity = mPrevAngularVelocity;
	mAngularMomentum = mPrevAngularMomentum;
}


/**
* \brief Setzt das angulaere Moment als Vektor
*/
void RigidBody::setAngularMomentum(Vec3 angularMomentum){

	assert(!ISNAN(angularMomentum[0]) || 
		   !ISNAN(angularMomentum[1]) || 
		   !ISNAN(angularMomentum[2]));
	assert(!ISINF(angularMomentum[0]) || 
		   !ISINF(angularMomentum[1]) || 
		   !ISINF(angularMomentum[2]));

	mPrevAngularMomentum = mAngularMomentum;
	mPrevAngularVelocity = mAngularVelocity;
	mAngularMomentum = angularMomentum;
	if (angularMomentum.length() != 0)
		mAngularVelocity = getInvWorldInertiaTensor() * angularMomentum;
	else 
		mAngularVelocity = Vec3(0.0, 0.0, 0.0);
}

/**
* \brief Setzt das vorherige angulaere Moment als Vektor
*/ 
void RigidBody::setPrevAngularMomentum(Vec3 prevAngularMomentum) {
	mPrevAngularMomentum = prevAngularMomentum;
	if (prevAngularMomentum.length() != 0)
		mPrevAngularVelocity = getInvWorldInertiaTensor() * prevAngularMomentum;
	else setPrevAngularVelocity(Vec3(0.0, 0.0, 0.0));
}

/**
* \brief Macht den letzten Aufruf von setAngularMomentum rückgängig.
*/
void RigidBody::undoSetAngularMomentum()
{
	mAngularMomentum = mPrevAngularMomentum;
	mAngularVelocity = mPrevAngularVelocity;
}


/**
* \brief Setzt die Winkelbeschleunigung als Vektor
* \todo Für welchen Fall gilt die Torque Berechnung? Keyword: Drehachsen?
*
* 
*/
void RigidBody::setAngularAcceleration(Vec3 /*angularAcceleration*/) {
	cout << "Achtung: setAngularAcceleration darf nicht verwendet werden." << endl;
	assert(false);
/*
	mAngularAcceleration = angularAcceleration;
	//neuen Torque setzen
	Matrix3x1 torque = (getWorldInertiaTensor() * 
				Matrix <float> (mAngularAcceleration)) +
				(Matrix3x1(getAngularVelocity()).star() * 
				 getWorldInertiaTensor() *
				 Matrix3x1(getAngularVelocity()));
	mTorque[0] = torque[0][0];
	mTorque[1] = torque[1][0];
	mTorque[2] = torque[2][0];
*/
}

/**
* \brief Setzt diei vorige Winkelbeschleunigung als Vektor
* \todo Für welchen Fall gilt die Torque Berechnung? Keyword: Drehachsen?
* \todo eigentlich WorldInertiaTensor als vorigen...
*
*/
void RigidBody::setPrevAngularAcceleration(Vec3 /*angularAcceleration*/) {
	cout << "Achtung! Angular Acceleration wird nicht geupdatet (in setPrevAngularAcceleration(..). Viel Spass." << endl;
/*
        mPrevAngularAcceleration = angularAcceleration;
        //neuen Torque setzen
        Matrix3x1 torque = (getWorldInertiaTensor() *
                 Matrix <float> (mPrevAngularAcceleration)) +
                (Matrix3x1(getPrevAngularVelocity()).star() *
                 getWorldInertiaTensor() *
                 Matrix3x1(getPrevAngularVelocity()));
        mPrevTorque[0] = torque[0][0];
        mPrevTorque[1] = torque[1][0];
        mPrevTorque[2] = torque[2][0];
*/
}

/**
* \brief Setzt den Trägheitstensor in Körperkoordinaten als Matrix
*/
void RigidBody::setInertiaTensor(const Matrix3x1 inertiaTensor)
{
	mInvInertiaTensor = 0;

	mInvInertiaTensor[0][0] = 1/inertiaTensor[0][0];
	mInvInertiaTensor[1][1] = 1/inertiaTensor[1][1];
	mInvInertiaTensor[2][2] = 1/inertiaTensor[2][2];
}

/**
* \brief Setzt den Trägheitstensor in Körperkoordinaten als 3 float
*/
void RigidBody::setInertiaTensor(float a, float b, float c)
{
	mInvInertiaTensor = 0;
	mInvInertiaTensor[0][0] = 1/a;
	mInvInertiaTensor[1][1] = 1/b;
	mInvInertiaTensor[2][2] = 1/c;
}





// *********************************
// G E T T E R
// *********************************


/**
* \brief Gibt die Orientierung zurück
* \return Quaternion
*/
Quaternion RigidBody::getOrientation() const{
	return mOrientation;
}

/**
* \brief Gibt die Orientierung zurück
* \param Quaternion
*/
void RigidBody::getOrientation(Quaternion &ori) const{
	ori = mOrientation;
}

/**
* \brief Gibt die vorherige Orientierung zurück
* \return Quaternion
*/
Quaternion RigidBody::getPrevOrientation() const{
	return mPrevOrientation;
}


/**
* \brief Gibt das Drehmoment zurück
* \return Vector3<flaot>
*/
Vec3 RigidBody::getTorque() const{
	return mTorque;
}

/**
* \brief Gibt das Drehmoment zurück
* \param torque Drehmoment
*/
void RigidBody::getTorque(Vec3 &torque) const{
	torque = mTorque;
}

/**
* \brief Gibt das Drehmoment zurück
* \param torque Rückgabe per Referenz
*/
void RigidBody::getTorque (Matrix3x1 & torque) const {

	assert (torque.getSizeM () == 3);
	assert (torque.getSizeN () == 1);
	torque [0][0] = mTorque [0];
	torque [1][0] = mTorque [1];
	torque [2][0] = mTorque [2];
}

/**
* \brief Gibt die Winkelgeschwindigkeit zurück
* \return Vec3
*/
Vec3 RigidBody::getAngularVelocity() const{
	return mAngularVelocity;
}

/**
* \brief Gibt die Winkelgeschwindigkeit zurück
* \param aVelo rückgabe per referenz
*/
void RigidBody::getAngularVelocity(Vec3 &aVelo) const{
	aVelo = mAngularVelocity;
}

/**
* \brief Gibt die Winkelgeschwindigkeit zurück
* \param aVelo rückgabe per referenz (3x1 Matrix)
*/
void RigidBody::getAngularVelocity(Matrix3x1 &aVelo) const{

	
	assert(aVelo.getSizeM() == 3 && aVelo.getSizeN() == 1);
	aVelo[0][0] = mAngularVelocity[0];
	aVelo[1][0] = mAngularVelocity[1];
	aVelo[2][0] = mAngularVelocity[2];
}

/**
* \brief Gibt die vorherige Winkelgeschwindigkeit zurück
* \return Vec3
*/
Vec3 RigidBody::getPrevAngularVelocity() const{
	return mPrevAngularVelocity;
}

/**
* \brief Gibt angulaeres Moment zurück
* \return Vec3
*/
Vec3 RigidBody::getAngularMomentum() const{
	return mAngularMomentum;
}

/**
* \brief Gibt angulaeres Moment zurück
* \param angularMomentum
*/
void RigidBody::getAngularMomentum(Vec3 &angularMomentum) const{
	angularMomentum = mAngularMomentum;
}

/**
* \brief Gibt vorheriges angulaeres Moment zurück
* \return Vec3
*/
Vec3 RigidBody::getPrevAngularMomentum() const{
	return mPrevAngularMomentum;
}

/**
* \brief Gibt die Winkelbeschleunigung zurück
* \return Vec3
*/
Vec3 RigidBody::getAngularAcceleration() const{
	return mAngularAcceleration;
}

/**
* \brief Gibt die vorige Winkelbeschleunigung zurück
* \return Vec3
*/
Vec3 RigidBody::getPrevAngularAcceleration() const{
        return mPrevAngularAcceleration;
}


/**
* \brief Gibt den Trägheitstensor zurück
* \return inertia tensor
*/
Matrix3x3 RigidBody::getInertiaTensor() const {
	Matrix3x3 inertiaTensor(0.0);
    assert ( mInvInertiaTensor[0][0] );    
    assert ( mInvInertiaTensor[1][1] );
    assert ( mInvInertiaTensor[2][2] );    

	inertiaTensor[0][0] = 1/mInvInertiaTensor[0][0];
	inertiaTensor[1][1] = 1/mInvInertiaTensor[1][1];
	inertiaTensor[2][2] = 1/mInvInertiaTensor[2][2];

	return inertiaTensor;
}

/**
 * \brief Gibt den Trägheitstensor in Weltkooridnaten zurück
 * \return Matrix3x1
 * \todo if "freeze" flag is set we should return zero
 */
Matrix3x3 RigidBody::getWorldInertiaTensor() const {
	return	
		mOrientation.getRotationMatrix() *
		getInertiaTensor() *
		mOrientation.getRotationMatrix().T();
}

/**
* \brief Gibt die Inverse des Trägheitstensor in Körperkoordinaten zurück
* \return Matrix3x1
*/
Matrix3x3 RigidBody::getInvInertiaTensor() const {
	if (mIsFrozen || !mIsDynamic)
		return Matrix3x3(0.0f);
	else
		return mInvInertiaTensor;
}

/**
* \brief Gibt die Inverse des Trägheitstensor in Weltkoordinaten als Matrix zurück
* \return Matrix3x1
*/
Matrix3x3 RigidBody::getInvWorldInertiaTensor() const {
	if (mIsFrozen || !mIsDynamic)
		return Matrix3x3(0.0f);
    
	return  mOrientation.getRotationMatrix () *
		getInvInertiaTensor() *
		mOrientation.getRotationMatrix ().T ();
}

/**
* \brief Gibt die Inverse des Trägheitstensor in Weltkoordinaten als Matrix zurück
* \param invWorldTensor Rückgabe per Referenz
*/
void RigidBody::getInvWorldInertiaTensor (Matrix3x3 &invWorldTensor) const
{

  if (mIsFrozen || !mIsDynamic) {
    invWorldTensor = Matrix3x3(0.0f);
    return;
  }

  Matrix3x3 rotation(mOrientation.getRotationMatrix());

  invWorldTensor = rotation * getInvInertiaTensor () * rotation.T ();
}



// *********************************
// OTHER
// *********************************




/**
* \brief Fügt ein Drehmoment hinzu
*/
void RigidBody::addTorque(Vec3 torque){
	assert(!ISNAN(torque[0]) || !ISNAN(torque[1]) || !ISNAN(torque[2]));
	assert(!ISINF(torque[0]) || !ISINF(torque[1]) || !ISINF(torque[2]));

mTorque += torque;
	//neue angularAcceleration setzen
	Matrix3x1 m(getInvWorldInertiaTensor() * mTorque);
	mAngularAcceleration[0] = m[0][0];
	mAngularAcceleration[1] = m[1][0];
	mAngularAcceleration[2] = m[2][0];
}


/**
* \brief Verarbeiten von Gelenkkräften
*/
void RigidBody::processConstraints(Matrix6x1 constrain){

	assert(!ISNAN(constrain[0][0]) || !ISNAN(constrain[1][0]) || 
		   !ISNAN(constrain[2][0]) || !ISNAN(constrain[3][0]) || 
		   !ISNAN(constrain[4][0]) || !ISNAN(constrain[5][0]));
	assert(!ISINF(constrain[0][0]) || !ISINF(constrain[1][0]) || 
		   !ISINF(constrain[2][0]) || !ISINF(constrain[3][0]) || 
		   !ISINF(constrain[4][0]) || !ISINF(constrain[5][0]));
	addForce(Vec3(constrain[0][0], constrain[1][0], constrain[2][0]));
	addTorque(Vec3(constrain[3][0], constrain[4][0], constrain[5][0]));
}

/**
 * \BRIEF Verarbeiten von Gelenk Poststabilisieru// *********************************
// OTHERS
// *********************************
ng
 *
 * Poststabilisierung (post stabilization) für Gelenkbedingungen
 * nach Cline und Pai "Post-Stabilization for Rigid Body Simulation with
 * Contact and Constraints", Abschnitt 5. Stabilisierungsmethoden
 * wirken dem numerischen Fehler entgegen, der bei der Berechnung
 * und Umsetzung von Gelenkkräften entsteht. Cline und Pai beschreiben
 * eine Formulierung von Poststabilisation als LCP. Es wurde nur die Lösung
 * eines linearen Gleichungssystems und nicht die Lösung eines
 * LCP (linear complementary problem) implementiert. D.h. Poststabilisierung
 * von Kontaktbehandlung ist nicht möglich. Alternativ zur Poststabilisierung
 * kann die Baumgartemethode verwendet werden, die keiner gesonderten
 * Verarbeitung in der Festkoerperklasse bedarf. Die vorliegende Funktion
 * bekommt das Ergebnis der Poststabilisiationsberechnung als 6x1-Vektor
 * übergeben, der Positionsänderung und Orientierungsänderung enthält
 * letzteres als Eulerwinkel gemäß der verwendeten Konvention, siehe die
 * Quaternion-Klasse. Diese werden als Vektor der Dimension 3 und als
 * Eulerwinkel auf die Position und Orientierung addiert.
 *
 * \author Nils Hornung, Uni-Kennung <hornung>
 */
void RigidBody::processConstraintPostStabilization
                (Matrix6x1 spaceDeltaPosition)
{

	// Separation von linearer Position und Orientierung als Eulerwinkel
	Vector3 <float> deltaPosition;
	Vector3 <float> deltaEuler;
	for (int i=0; i<3; i++) {
		assert(!ISNAN(spaceDeltaPosition[i][0]));
		assert(!ISINF(spaceDeltaPosition[i][0]));

		deltaPosition [i] = spaceDeltaPosition [i][0];
		deltaEuler    [i] = spaceDeltaPosition [i+3][0];
	}

	// Verarbeitung der linearen Position
	setPosition (getPosition () + deltaPosition);

	// Verarbeitung der angulaeren Komponente
	Vector3 <float> euler = getOrientation ().getEulerRotation (); // alt
	euler = euler + deltaEuler;                                    // neu
	Quaternion orientation (euler [0], euler [1], euler [2]);      // als Quat
	setOrientation (orientation);

}

/**
* \brief Berechnen und aufaddieren von Force und Torque
* \param 6x1-Matrix (0-2 -> Angriffspunkt in Weltkoordinaten; 3-5 -> Force)
*/
void RigidBody::computeForceAndTorque(Matrix6x1 collisionMatrix){	
	Vec3 force = Vec3(collisionMatrix[3][0], collisionMatrix[4][0], collisionMatrix[5][0]);

	//Berechnen und Aufaddieren der Torque
	addTorque(Vec3 (cross(Vec3 (Vec3(collisionMatrix[0][0], collisionMatrix[1][0], 
		collisionMatrix[2][0]) - mPosition), force)));

	//Aufaddieren der Force
	addForce(force);
}


void RigidBody::backupVelocities(){
	mPrevVelocity = mVelocity;
	mPrevAngularVelocity = mAngularVelocity;

	mPrevLinearMomentum = mLinearMomentum;
	mPrevAngularMomentum = mAngularMomentum;
}

void RigidBody::restoreVelocities(){
	mVelocity = mPrevVelocity;
	mAngularVelocity = mPrevAngularVelocity;

	mLinearMomentum = mPrevLinearMomentum;
	mAngularMomentum = mPrevAngularMomentum;
}


void RigidBody::backupPositions(){
	mPrevPosition = mPosition;
	mPrevOrientation = mOrientation;
}

void RigidBody::restorePositions(){
	mPosition = mPrevPosition;
	mOrientation = mPrevOrientation;
}


// *********************************
// INTEGRATOREN
// *********************************




/**
 * \brief Integration mit dem Integrator, der in SimonState.getIntegrator() definiert ist.
 * \param Interval in Millisekunden
 * Integrator durchläuft alle RigidBodies, die im System gespeichert sind
 */
void RigidBody::integrate(float interval, 
						  bool useViscosity, 
						  float viscositySlowdownLinear, 
						  float viscositySlowdownAngular, 
						  bool absorbForce){

	switch(SimonState::exemplar()->getIntegrator()){

    case SimonState::INTEGRATE_EULER:
    	integrateEuler(interval, 
					   useViscosity, 
					   viscositySlowdownLinear, 
					   viscositySlowdownAngular, 
					   absorbForce);
        break;
    default:
		SimonState::exemplar()->errors << "Sorry, the integrator " 
									  << SimonState::exemplar()->getIntegrator()
									  << " dose not exist" << endl;
	}
}

void RigidBody::integrateVelocities(float interval){
	switch(SimonState::exemplar()->getIntegrator()){
    case SimonState::INTEGRATE_EULER:
    	integrateEulerVelocities(interval);
        break;
    default:
		SimonState::exemplar()->errors << "Sorry, the integrator " 
									  << SimonState::exemplar()->getIntegrator()
									   << " dose not exist" << endl;
	}
}


void RigidBody::integratePositions(float interval, 
								  bool useViscosity, 
								  float viscositySlowdownLinear, 
								  float viscositySlowdownAngular, 
								  bool absorbForce){

	switch(SimonState::exemplar()->getIntegrator()) {
    case SimonState::INTEGRATE_EULER:
    	integrateEulerPositions(interval, 
								useViscosity, 
								viscositySlowdownLinear, 
								viscositySlowdownAngular, 
								absorbForce);
        break;
	default:
		SimonState::exemplar()->errors << "Sorry, the integrator " 
									   << SimonState::exemplar()->getIntegrator()
									   << " dose not exist" << endl;
	}
}

  
/**
 * \brief Integration mit Euler Methode
 * \param Interval in Millisekunden
 */
void RigidBody::integrateEuler(float interval, 
							   bool useViscosity, 
							   float viscositySlowdownLinear, 
							   float viscositySlowdownAngular, 
							   bool absorbForce){
		
		integrateEulerVelocities(interval);
		integrateEulerPositions(interval, 
								useViscosity, 
								viscositySlowdownLinear, 
								viscositySlowdownAngular, 
								absorbForce);
}

/**
 * \brief Integration mit Euler Methode (compute and set velocities)
 * \param Interval in Millisekunden
 */
void RigidBody::integrateEulerVelocities(float interval)
{
	
	//! üerspringe den RigidBody, wenn die isDynamic Flag false ist
	if (!(getIsDynamicFlag())) {
		//Setzen von Kraft und Drehmoment auf null
		setForce(Vec3::nullVector);
		setTorque(Vec3::nullVector);
		return;
	}
	
	// lineare Beschleunigung 
	getForce(mIntegratorForce);

	// Winkelbeschleunigung 
	getTorque(mIntegratorTorque);

	// Velocity berechnen
	getLinearMomentum(mIntegratorOldLinearMomentum);
	mIntegratorNewLinearMomentum 
		= mIntegratorOldLinearMomentum + mIntegratorForce * interval;
	setLinearMomentum(mIntegratorNewLinearMomentum);

	// Angular Velocity berechnen
	getAngularMomentum(mIntegratorOldAngularMomentum);
	mIntegratorNewAngularMomentum 
		= mIntegratorOldAngularMomentum + mIntegratorTorque * interval;
	setAngularMomentum(mIntegratorNewAngularMomentum);

	//! wenn die kräfte zu groß werden, lösche alles
	if (mIntegratorNewLinearMomentum[X]> 1000000.00 || mIntegratorNewLinearMomentum[Y] > 1000000.00 || mIntegratorNewLinearMomentum[Z] > 1000000.00 ||
		mIntegratorNewAngularMomentum[X]> 1000000.00 || mIntegratorNewAngularMomentum[Y] > 1000000.00 || mIntegratorNewAngularMomentum[Z] > 1000000.00){
		SimonState::exemplar()->clearSimulationSystems();
		SimonState::exemplar()->errors << "Woa! Simon is getting instable. Start again if you want." << SimonState::endm;
		cout << "linar :" <<  mIntegratorNewLinearMomentum << endl;
		cout << "angular: " << mIntegratorNewAngularMomentum << endl;
		return;
	}
}


/**
 * \brief Integration mit Euler Methode (zweiter Schritt)
 * \param Interval in Millisekunden
 */
void RigidBody::integrateEulerPositions(float interval, 
										bool useViscosity, 
										float viscositySlowdownLinear, 
										float viscositySlowdownAngular, 
										bool absorbForce){
	
	//! Überspringe den RigidBody, wenn die isDynamic Flag false ist
	if (!(getIsDynamicFlag())) {
		return;
	}

	// Position berechnen
	setPosition(getPosition() + mInvMass * getPrevLinearMomentum() * interval);

	// Orientierung berechnen
	mIntegratorOldAngularVelocity 
		= getInvWorldInertiaTensor() * getPrevAngularMomentum();
	mIntegratorOldOrientation 
		= getOrientation();
	mIntegratorOldOrientation.normalize();
	mIntegratorQuaternionVelocity 
		= Quaternion(mIntegratorOldAngularVelocity);		
	mIntegratorQuaternionAbbreviation 
		= 0.5f * mIntegratorQuaternionVelocity * mIntegratorOldOrientation;
	mIntegratorONewOrientation 
		= mIntegratorOldOrientation + (interval * mIntegratorQuaternionAbbreviation);
	setOrientation(mIntegratorONewOrientation);
	
	//!Verlangsamung der Bewegung
	if (useViscosity){
		mIntegratorNewAngularMomentum 
			= getAngularMomentum() * viscositySlowdownAngular;
		setAngularMomentum(mIntegratorNewAngularMomentum);
		mIntegratorNewLinearMomentum 
			= getLinearMomentum() * viscositySlowdownLinear;
		setLinearMomentum(mIntegratorNewLinearMomentum);
	}

	if (absorbForce){
		//Setzen von Kraft und Drehmoment auf null
		setForce(Vec3(0, 0, 0));
		setTorque(Vec3(0, 0, 0));
	}	
}
