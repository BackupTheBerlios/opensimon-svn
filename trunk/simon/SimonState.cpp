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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 *
 */


//------------------------------------------------------------------------------
/**
*  \class SimonState
*  \author Trappe & Rueger & Büchler
*
*
*  \brief Speicher globale Variblen für die Physik-Simulation
*
*  
*
*/
//------------------------------------------------------------------------------


#include <cstdio>

#include <simon/SimonState.h>
#include <simon/RigidBodySystem.h>
#include <simon/GeometrySystem.h>
#include <simon/ClothSystem.h>
#include <simon/ParticleSystem.h>
#include <simon/ConstraintSystem.h>

#include <simon/ContactInformationContainer.h>

//! Qualifier, der angibt, wo eine Message zu ende ist.
const std::string SimonState::endm= "<end>\n";
 
/**
* \brief Default-Konstruktor 
* Setzt Vektor (0.0, -1.0, 0.0) und skaliert den entsprechend der eingegebenen Gravitation
* Legt die einzelnen Systeme an
*/
SimonState::SimonState()
{
    setIntegrator(INTEGRATE_EULER);
	mGravityVector = Vector3<float>(0.0, -1.0, 0.0);
	mGravityVector.normalize();
	mGravityVector = mGravityVector * (float)0.00981;

	//set default values for viscosity
	mUseViscosity = true;
	mViscositySlowdownAngular = 0.99f;
	mViscositySlowdownLinear = 0.99f;
  
	mContactInformationThreshold = 0.7;
	mIntegrationIntervall = 5;

	// systeme erzeugen
	mBodySystem = RigidBodySystemPtr (new RigidBodySystem);
	mGeoSystem = GeometrySystemPtr (new GeometrySystem);
	mConstraintSystem = ConstraintSystemPtr (new ConstraintSystem);
	mParticleSystem = ParticleSystemPtr (new ParticleSystem);
	mClothSystem = ClothSystemPtr (new ClothSystem);

	mContactInformationContainer = ContactInformationContainerPtr(new ContactInformationContainer);
	
	setAirDensityMultiplyer(1.0);

	mProcessSimulation = true;
}

/**
* \brief Destruktor
*/
SimonState::~SimonState(){
}

SimonState* SimonState::instance = 0;

/**
 * \brief gibt das aktuelle RBSystem zurück
 *
 * \return mBodySystem aktuelles RBSystem
 */
RigidBodySystemPtr SimonState::getBodySystem() {
	return mBodySystem;
}

/**
 * \brief gibt das aktuelle RBSystem zurück
 *
 * \return actual GeometrySystem
 */
GeometrySystemPtr SimonState::getGeometrySystem() {
	return mGeoSystem;
}

/**
 * \brief gibt das aktuelle ConstraintSystem zurück
 *
 * \return mConstraintSystem aktuelles ConstraintSystem
 */
ConstraintSystemPtr SimonState::getConstraintSystem() {
	return mConstraintSystem;
}
/**
 * \brief gibt das aktuelle ParticleSystem
 * 
 * \return mParticleSystem aktuelles ParticleSystem
*/
ParticleSystemPtr SimonState::getParticleSystem(){
	return mParticleSystem;
}

/**
 * \brief gibt das aktuelle ClothSystem zurück
 *
 * \return mClothSystem aktuelles ClothSystem
 */
ClothSystemPtr SimonState::getClothSystem(){
	return mClothSystem;
}

/**
* \brief exemplar; Funktion für Singleton-Ansatz
*/
SimonState* SimonState::exemplar(){
	if (instance == 0)
		instance = new SimonState();
	return instance;
}

/**
 * \brief liefert einen boolean, je nachdem ob Viscosity ein oder ausgeschaltet ist
 * \return mUseViscosity
 */
bool SimonState::getViscosityFlag(){
	return mUseViscosity;
}

/**
 * \brief Schaltet die Viscosiät ein.
 */
void SimonState::enableViscosity(){
	mUseViscosity = true;
}

/**
 * \brief get the slowdown factor used in viscosity
 */
float SimonState::getViscositySlowdownLinear(){
	return mViscositySlowdownLinear;
}

/**
 * \brief set the slowdown factor used in viscosity (0 < slowdown < 1 makes sense)
 */
void SimonState::setViscositySlowdownLinear(float slowdown_factor){
	mViscositySlowdownLinear = slowdown_factor;
}

/**
 * \brief get the slowdown factor used in viscosity
 */
float SimonState::getViscositySlowdownAngular(){
	return mViscositySlowdownAngular;
}

/**
 * \brief set the slowdown factor used in viscosity (0 < slowdown < 1 makes sense)
 */
void SimonState::setViscositySlowdownAngular(float slowdown_factor){
	mViscositySlowdownAngular = slowdown_factor;
}

/**
 * \brief Schaltet die Viscosity aus.
 */
void SimonState::disableViscosity(){
	mUseViscosity = false;
}



/**
* \brief getGravity; liefert Gravitation zurück
*/
float const SimonState::getGravity(){
	return mGravityVector.length();
}

/**
* \brief setGravity; setzt die Gravitation
*/
void SimonState::setGravity(float grav){
	mGravityVector.normalize();
	mGravityVector *= grav;
}

/**
* \brief getGravityVector; liefert Vector der Gravitation
*/
Vector3<float> const SimonState::getGravityVector(){
	return mGravityVector;
}

/**
* \brief setGravityVector; setzt Vector der Gravitation
*/
void SimonState::setGravityVector(Vector3<float> gravVec){
	mGravityVector = gravVec;
}

/**
* \brief getGravityVector
* Liefert den normalisierten Gravitationsvektor
*/
Vector3<float> const SimonState::getGravityDirection(){
	Vector3<float> gravDir;
	gravDir = mGravityVector;
	gravDir.normalize();
	return gravDir;
}


/**
* \brief Alle Messages auf "cout" ausgeben.
* Gibt alle in messages gesammelten Meldungen aus.
*/
void SimonState::printMessages(){
	if (messages.str() != ""){
		std::cout <<  messages.str() << std::endl;
		messages.str("");
	}
}


/**
* \brief Alle Fehler auf "cout" ausgeben.
* Gibt alle in error gesammelten Fehlermeldungen aus.
*/
void SimonState::printErrors(){

	if (errors.str() != ""){
		std::cout << errors.str() << std::endl;
		errors.str("");
	}
}


/**
* \brief Alle Warnings auf "cout" ausgeben.
* Gibt alle in error gesammelten Fehlermeldungen aus.
*/
void SimonState::printWarnings(){

	if (warnings.str() != ""){
		std::cout << warnings.str() << std::endl;
		warnings.str("");
	}
}

/**
* \brief gefilterte Fehler auf "cout" ausgeben.
*
* \param std::string keyword Typ der Fehlermeldungen, die ausgegeben werden sollen.
*
* Gibt die in error gesammelten Fehlermeldungen zu einer bestimmtent Gruppe 
* aus. Die Gruppenbezeichnung wird mittels <i>keyword</i> übergeben, und muss bei den
* gespeicherten Fehlermeldungen am Anfang auftauchen.
*/
void SimonState::printErrors(std::string keyword){
	std::string errorLog = errors.str();
	std::string output = "";

	while (errorLog.find(keyword) != std::string::npos) {
		output += errorLog.substr(errorLog.find(keyword),errorLog.find(endm));
		output += "\n";
		errorLog.erase(0, errorLog.find(keyword));
		errorLog.erase(0,errorLog.find(endm) + endm.size());
		
	}
	std::cout << output << std::endl;
}
void SimonState::printWarnings(std::string keyword){

	std::string warningsLog = warnings.str();
	std::string output = "";

	while (warningsLog.find(keyword) != std::string::npos) {
		output += warningsLog.substr(warningsLog.find(keyword),warningsLog.find(endm));
		output += "\n";
		warningsLog.erase(0, warningsLog.find(keyword));
		warningsLog.erase(0,warningsLog.find(endm) + endm.size());
		
	}
	std::cout << output << std::endl;
}


/**
* \brief gefilterte Meldungen auf "cout" ausgeben.
*
* \param std::string keyword Typ der Meldungen, die ausgegeben werden sollen.
*
* Gibt die in messages gesammelten Meldungen zu einer bestimmtent Gruppe 
* aus. Die Gruppenbezeichnung wird mittels <i>keyword</i> übergeben, und muss bei den
* gespeicherten Meldungen am Anfang auftauchen.
*/
void SimonState::printMessages(std::string keyword){

	std::string messageLog = messages.str();
	std::string output = "";

	while (messageLog.find(keyword) != std::string::npos) {
		output += messageLog.substr(messageLog.find(keyword),messageLog.find(endm));
		output += "\n";
		messageLog.erase(0, messageLog.find(keyword));
		messageLog.erase(0,messageLog.find(endm) + endm.size());		
	}
	std::cout << output << std::endl;
}

/**
* \brief alle Meldungen löschen
*/
void SimonState::clearMessages()
{
  messages.str().erase();
}

/**
* \brief alle Warnings löschen
*/
void SimonState::clearWarnings()
{
  warnings.str().erase();
}

/**
* \brief alle Errors löschen
*/
void SimonState::clearErrors()
{
  errors.str().erase();
}

/**
* \brief gefilterte Meldungen löschen
*
* \param std::string keyword Typ der Meldungen, die gelöscht werden sollen.
*
* Gibt die in messages gesammelten Meldungen zu einer bestimmtent Gruppe 
* aus. Die Gruppenbezeichnung wird mittels <i>keyword</i> übergeben, und muss bei den
* gespeicherten Meldungen am Anfang auftauchen.
*/
void SimonState::clearMessages(std::string keyword)
{
	std::string messagesLog = messages.str();

	while (messagesLog.find(keyword) != std::string::npos) {
		messagesLog.erase(0, messagesLog.find(keyword));
		messagesLog.erase(0, messagesLog.find(endm) + endm.size());		
	}
}
void SimonState::clearWarnings(std::string keyword)
{
	std::string warningsLog = warnings.str();

	while (warningsLog.find(keyword) != std::string::npos) {
		warningsLog.erase(0, warningsLog.find(keyword));
		warningsLog.erase(0, warningsLog.find(endm) + endm.size());		
	}
}

void SimonState::clearErrors(std::string keyword)
{
	std::string errorsLog = errors.str();

	while (errorsLog.find(keyword) != std::string::npos) {
		errorsLog.erase(0, errorsLog.find(keyword));
		errorsLog.erase(0, errorsLog.find(endm) + endm.size());		
	}
}

ContactInformationContainerPtr SimonState::getContactInformationContainer() {
	return mContactInformationContainer;
}

void SimonState::setContactInformationContainer(ContactInformationContainerPtr contactInfos) {
	mContactInformationContainer = contactInfos;
}

// Erneuert alle mVariablen, löscht somit die Alten
void SimonState::clearSimulationSystems (){

	mBodySystem = RigidBodySystemPtr (new RigidBodySystem);
	mGeoSystem = GeometrySystemPtr (new GeometrySystem);
	mConstraintSystem = ConstraintSystemPtr (new ConstraintSystem);
	mParticleSystem = ParticleSystemPtr (new ParticleSystem);
	mClothSystem = ClothSystemPtr (new ClothSystem);

	mContactInformationContainer = ContactInformationContainerPtr(new ContactInformationContainer);
}

void SimonState::setIntegrator(int integrator)
{
   mWhichIntegrator=integrator;
}

int SimonState::getIntegrator() const
{
    return mWhichIntegrator;
}

void SimonState::setAirDensityMultiplyer(float multiplyer)
{
	mAirDensityMultiplyer = multiplyer;
}

float SimonState::getAirDensityMultiplyer() const
{
	return mAirDensityMultiplyer;
}

bool SimonState::getProcessSimulationFlag() {
	return mProcessSimulation;
}

void SimonState::setProcessSimulationFlag(bool flag) {
	mProcessSimulation = flag;
}

void SimonState::setContactInformationThreshold(float treshold){
	mContactInformationThreshold = treshold;
}

float SimonState::getContactInformationThreshold(){
	return mContactInformationThreshold;
}

void SimonState::setIntegrationIntervall(float intervall){
	mIntegrationIntervall = intervall;
}

float SimonState::getIntegrationIntervall() const{
	return mIntegrationIntervall;
}
