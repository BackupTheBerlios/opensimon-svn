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
 * \file SimonState.h
 * \class SimonState
 * 
 */
//------------------------------------------------------------------------------

#ifndef SIMON_STATE_H
#define SIMON_STATE_H



#include <simon/SmartPointer.h>
#include <simon/Vector3.h>
#include <simon/ContactInformationContainer.h>
#include <sstream>

#pragma warning( push )
// Kein Copy-Construktor möglich
//! \todo Copy-Konstruktor bauen, oder als Singleton, was es eigentlich eh ist?
#pragma warning( disable : 4511 )
#pragma warning( disable : 4512 )

// forward declarations

class SimonState {

public:

enum IntegrationMethod {INTEGRATE_EULER, INTEGRATE_RUNGEKUTTA};	

	//! ostringsteeam um error-Nachrichten zu speichern
	//! \todo ein kleines kommetar mit hinweis auf endm hinzufügen
	std::ostringstream errors;
	std::ostringstream warnings;
	std::ostringstream messages;

	//! Qualifier, der angibt, wo eine Message zu ende ist.
	//! \todo sollte const sein. Geht immom. nicht, da wir erst im Konstruktor einen wert zuweisen.
	static const std::string endm;
	
	static SimonState* exemplar();

	void printErrors();
	void clearErrors();
	void clearErrors(std::string keyword);
	void printErrors(std::string keyword);

	void printWarnings();
	void clearWarnings();
	void clearWarnings(std::string keyword);
	void printWarnings(std::string keyword);

	void printMessages();
	void clearMessages();
	void clearMessages(std::string keyword);	
	void printMessages(std::string keyword);


	~SimonState();

	void setGravity(float);
	float const getGravity();

	void setGravityVector(Vec3);
	Vec3 const getGravityVector();
	void setGravityDirection(Vec3);
	Vec3 const getGravityDirection();

	GeometrySystemPtr getGeometrySystem();
	RigidBodySystemPtr getBodySystem();
	ConstraintSystemPtr getConstraintSystem();
	ParticleSystemPtr getParticleSystem();
	ClothSystemPtr getClothSystem();

	//! \todo set's für cloth und Particle
	//! \todo noch implementieren
	void setGeometrySystem(GeometrySystemPtr system) {};
	//! \todo noch implementieren
	void setBodySystem(RigidBodySystemPtr system) {};
	//! \todo noch implementieren
	void setConstraintSystem(ConstraintSystemPtr system) {};

	ContactInformationContainerPtr getContactInformationContainer();
	void setContactInformationContainer(ContactInformationContainerPtr contactInfos);

	void setContactInformationThreshold(float threshold);
	float getContactInformationThreshold();

	void clearSimulationSystems();

	bool getProcessSimulationFlag();
	void setProcessSimulationFlag(bool flag);

	bool getViscosityFlag();
	void enableViscosity();
	void disableViscosity();

	//get and set the factor used for viscosity slowdown (0<x<1 makes sense)
	float getViscositySlowdownAngular();
	void setViscositySlowdownAngular(float);
	float getViscositySlowdownLinear();
	void setViscositySlowdownLinear(float);

    void setIntegrator(int integrator);  
    int getIntegrator() const;

	void setIntegrationIntervall(float);
	float getIntegrationIntervall() const;
		
	//get und set fuer den luftdichtenmultiplikator
	float getAirDensityMultiplyer() const;
	void  setAirDensityMultiplyer(float);
    
private:

	SimonState(); //< private cause of the singleton pattern
	static SimonState *instance;

	Vec3 mGravityVector;

	//values for viscosity
	bool mUseViscosity;
	float mViscositySlowdownAngular;
	float mViscositySlowdownLinear;

	//! welcher integrator (enum INTEGRATE_EULER, INTEGRATE_RUNGEKUTTA)
	int mWhichIntegrator;

	float mIntegrationIntervall;

	//! luftdichtenmultplikator 
	//! \see CollisionSystem
	float mAirDensityMultiplyer;

	// Weather or not process the simulation
	bool mProcessSimulation;

	// systeme definieren
	RigidBodySystemPtr mBodySystem;
	GeometrySystemPtr mGeoSystem;
	ConstraintSystemPtr mConstraintSystem;
	ParticleSystemPtr mParticleSystem;
	ClothSystemPtr mClothSystem;
	
	ContactInformationContainerPtr mContactInformationContainer;

	// how high the impulseMagnitude has to be before contact info is stored
	float mContactInformationThreshold;
};

#pragma warning( pop )

#endif

