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


#include <simon/ParticleSystem.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <map>


using namespace std;

//------------------------------------------------------------------------------
/**
 * \class ParticleSystem
 * \brief Das ParticleSystem ist die Klasse, mit der Partikel verwaltet werden. 
 *
 *
 * 
 */
//------------------------------------------------------------------------------

// Delta T für Rechenschritte für alte Integratoren
//const float dt = 0.01f; 


ParticleSystem::ParticleSystem() { 
	// nix
}

/**
 * \brief Liefert ein Pointer auf einen neuen Partikel zurück
 * \return RigidBody*
 */
ParticlePtr  ParticleSystem::create(Id id) {
	ParticlePtr particle(new Particle());
	particle->setId(id);
	mParticleList[id] = particle;

	return particle;
}

/**
 * \brief Liefert einen Pointer auf den Partikel an der Stelle des übergebenen Indexes, falls es vorhanden ist, sonst einen null-Pointer
 * \return RigidBody*
 */
ParticlePtr  ParticleSystem::getParticle(int index) {
	Id id;
	id.setNumber(index);
	id.setType(Id::typeParticle);
	if (mParticleList[id])
		return mParticleList[id];
	else {
		ParticlePtr particle;
		return particle;
	}
}

/**
 * \brief Liefert ein Pointer auf den Partikel mit der Id des übergebenen Id-Objekts zurück, falls es vorhanden ist, sonst einen null-Pointer
 * \return Particle*
 */
ParticlePtr  ParticleSystem::getParticle(Id id) {
	if (mParticleList[id])
		return mParticleList[id];
	else {
		ParticlePtr particle;
		return particle;
	}
}

/**
 * \brief Liefert die Anzahl der gespeicherten Partikel
 * \return int
 */
int ParticleSystem::getNumberOfParticles() {
	return (int)mParticleList.size();
}

/**
 * \brief Löscht den Partikel mit der Id des übergebenen Id-Objekts, falls vorhanden
 */
void ParticleSystem::deleteParticle(Id id) {
	std::cout << "Die Löschenfunktion vom ParticleSystem wurde noch nie getestet. Viel Spass!" << std::endl;
	cout << mParticleList.count(id) << endl;
	if (!mParticleList.count(id))
		mParticleList.erase(id);
	else 
		printf("Zu löschendes Objekt (%d) existiert nicht.", id.getNumber());
}

/**
 * \brief Addiert die Schwerkraft auf alle Partikel auf
 */
void ParticleSystem::addGravity() {
	if (mParticleList.empty())
		return;
	for (std::map<Id, ParticlePtr >::iterator particle = mParticleList.begin(); particle != mParticleList.end(); ++particle) {
		//Gravitation aufaddieren F = m * g
		particle->second->setForce(SimonState::exemplar()->getGravityVector()* particle->second->getMass());
	}
}

/**
 * \brief fragt ab ob Partikel kollidiert 
 */
bool ParticleSystem::isColliding(ParticlePtr particle){
	return particle->getIsColliding();
}
/**
 * \brief fragt ab WO ein Particle kollidiert.(unterhalb oder oberhalb des RB)
 */
int ParticleSystem::isCollidingWhere(ParticlePtr particle){
	return particle->getIsCollidingWhere();
}

/**
 * \brief setzt das zu kollidierende Objekt
 */
void ParticleSystem::setCollidingBody(RigidBodyPtr body){
	mCollidingBody = body;
}


/**
 * \brief fragt das zu kollidierende Objekt ab
 */
RigidBodyPtr ParticleSystem::getCollidingBody(){
	return mCollidingBody;
}

/**
 * Diese Funktion iteriert über alle Partikel und ruft die übergebene Funktion 
 * mit dem aktuellen Partikel als Parameter auf. Sorry, aber es ist so!
 *
 * \param givenFunction Ein Funktionspointer auf eine Funktion die aufgerufen werden soll.
 */
void ParticleSystem::forEveryParticleCall(void (*givenFunction)(ParticlePtr)) {
	 
	for(
		ParticleList::iterator particle = mParticleList.begin();
		particle != mParticleList.end();
		particle++) {
		
		givenFunction((particle->second));
	}		
}

vector<Vec3> ParticleSystem::getPositionsOfParticles() {

	vector<Vec3> positions;
	
	for (ParticleList::iterator particle = mParticleList.begin();
		 particle != mParticleList.end();
		 ++particle) {
		
		positions.push_back((particle->second)->getPosition());
	}
	return positions;
}


vector<Id> ParticleSystem::getIdsOfParticles() {

	vector<Id> ids;
	
	for (ParticleList::iterator particle = mParticleList.begin();
		 particle != mParticleList.end();
		 ++particle) {
		
		ids.push_back((particle->second)->getId());
	}
	return ids;
}

vector<pair<Id,Vec3> > ParticleSystem::getIdsAndPosOfParticles() {

	vector<pair<Id,Vec3> > pairs;
	
	for (ParticleList::iterator particle = mParticleList.begin();
		 particle != mParticleList.end();
		 ++particle) {
		
		pairs.push_back(make_pair((particle->second)->getId(), 
								(particle->second)->getPosition()));
	}
	return pairs;
}

//------------------------neue-Integratoren-----------------------------------------------------------------------

/**
 * \brief Integration mit Runge-Kutta Methode
 * \Integrator durchläuft alle RigidBodies, die im System gespeichert sind
 * \param Interval in Millisekunden
 * \todo Achtunge! orientierung wird noch nicht richtig berechnet. erstmal nur euler verwenden
 */

void ParticleSystem::integrateRungeKutta(float interval) {
	float halfInterval = interval / 2;
	float sixthInterval = interval / 6;
	
	std::map<Id, ParticlePtr>::iterator endOfItr = mParticleList.end();
	for (std::map<Id, ParticlePtr>::iterator particle = mParticleList.begin(); particle != endOfItr; ++particle) {
		
		//! Überspringe den Partikel, wenn die isDynamic Flag false ist
		if (!(particle->second->getIsDynamicFlag()))
			continue;

		//lineare Variablen
		Vector3<float> position = particle->second->getPosition();		//Position
		Vector3<float> velocity = particle->second->getVelocity();		//Geschwindigkeit
		Vector3<float> accel = particle->second->getAcceleration();		//Beschleunigung

		Vector3<float> s1pos(0,0,0);		//1. Stützpunkt mit position
		Vector3<float> s1vel(0,0,0);		//1. Stützpunkt mit geschwindigkeit
		Vector3<float> s2pos(0,0,0);		//2. Stützpunkt mit position
		Vector3<float> s2vel(0,0,0);		//2. Stützpunkt mit geschwindigkeit
		Vector3<float> s3pos(0,0,0);		//3. Stützpunkt mit position
		Vector3<float> s3vel(0,0,0);		//3. Stützpunkt mit geschwindigkeit
		Vector3<float> s4pos(0,0,0);		//4. Stützpunkt mit position
		Vector3<float> s4vel(0,0,0);		//4. Stützpunkt mit geschwindigkeit

		
		//berechne Stützpunkt 1
		//linear
		s1pos = velocity;
		s1vel = accel;
	
	
		//berechne Stützpunkt 2
		//linear
		s2pos = velocity + accel * halfInterval;
		s2vel = accel;
		
			
		//berechne Stützpunkt 3
		//linear
		s3pos = velocity + accel * halfInterval;
		s3vel = accel;
	
		
		//berechne Stützpunkt 4
		//linear
		s4pos = velocity + accel * interval;
		s4vel = accel;
	
	
		//berechne neue Position bzw. Geschwindigkeit
		position = position + (s1pos + 2 * s2pos + 2 * s3pos + s4pos) * sixthInterval;
		velocity = velocity + (s1vel + 2 * s2vel + 2 * s3vel + s4vel) * sixthInterval;
				
		
		//neue Werte setzen
		particle->second->setPrevPosition(particle->second->getPosition());
		particle->second->setPosition(position);
		particle->second->setVelocity(velocity);
		//cout << "Velocity " << particle->second->getVelocity() << endl;
		//cout << "Acc " << particle->second->getAcceleration() << endl;
		
		//! Verlangsamung der Bewegung
		//! \todo keine ahnung, ob das überhaupt sinnvoll umgesetzt ist
		if (SimonState::exemplar()->getViscosityFlag()) {
			Vector3<float> vel = particle->second->getVelocity() *
				(SimonState::exemplar()->getViscositySlowdownLinear() - 0.3);
			particle->second->setVelocity(vel);
		}

		//Setzen von Kraft und Drehmoment auf null
		particle->second->setForce(Vector3<float>(0.0, 0.0, 0.0));
	}
}


//--------------------------alte-Integratoren---------------------------------------------------------------------
/**
 * \brief Euler alt 
 */

//void ParticleSystem::integrateEuler(float interval) {
//	
//	int cnt = (int)(interval/(dt*1000));
//
//	for (std::map<Id, ParticlePtr>::iterator particle = mParticleList.begin(); particle != mParticleList.end(); ++particle) {
//		if (particle->second->getIsDynamicFlag() == true){
//		// lineare Beschleunigung 
//		Vector3<float> acc = particle->second->getForce() / particle->second->getMass();	
//
//		for (int i=0; i<cnt; i++) {
//			
//			// Velocity berechnen
//			Vector3<float> oldVelo = particle->second->getVelocity();
//			Vector3<float> velo = oldVelo + acc * dt;
//			particle->second->setVelocity(velo);
//
//			// Position berechnen
//			particle->second->setPosition(particle->second->getPosition() + oldVelo * dt);
//		}
//		//Verlangsamung der Bewegung
//		//bei Tests aufpassen, gegebenenfalls auskommentieren oder Faktor anpassen
//		Vector3<float> vel = particle->second->getVelocity()*0.98f;
//		particle->second->setVelocity(vel);
//	}
//		//Setzen von Kraft auf null
//	    particle->second->setForce(Vector3<float>(0.0f, 0.0f, 0.0f));
//	}
//}

/**
 * \brief Runge Kutta alt
 */
/*
void ParticleSystem::integrateRungeKutta(float interval) {
	
	int cnt = (int)(interval/(dt*1000));
		
	for (std::map<Id, ParticlePtr>::iterator particle = mParticleList.begin(); particle != mParticleList.end(); ++particle) {
		if (particle->second->getIsDynamicFlag() == true){
		//lineare Variablen
		std::vector< Vector3<float> > vars(3);	//speichert Position, Geschwindigkeit, Beschleunigung
		std::vector< Vector3<float> > s1(2);		//1. Stützpunkt mit position und geschwindigkeit
		std::vector< Vector3<float> > s2(2);		//2. Stützpunkt mit position und geschwindigkeit
		std::vector< Vector3<float> > s3(2);		//3. Stützpunkt mit position und geschwindigkeit
		std::vector< Vector3<float> > s4(2);		//4. Stützpunkt mit position und geschwindigkeit
		
		
		vars[0] = particle->second->getPosition();
		vars[1] = particle->second->getVelocity();
		vars[2] = particle->second->getAcceleration();
	
		for (int i = 0; i < cnt; i++) {
			//berechne Stützpunkt 1
			for (int j = 0; j < 2; j++)
				s1[j] = vars[j+1];
			//berechne Stützpunkt 2
			for (int j = 0; j < 2; j++)
				s2[j] = vars[j+1] + s1[j] * dt / 2;
			//berechne Stützpunkt 3
			for (int j = 0; j < 2; j++)
				s3[j] = vars[j+1] + s2[j] * dt / 2;
			//berechne Stützpunkt 4
			for (int j = 0; j < 2; j++)
				s4[j] = vars[j+1] + s3[j] * dt;
			//berechne neue Position bzw. Geschwindigkeit
			for (int j = 0; j < 2; j++)
				vars[j] = vars[j]+(s1[j]+2*s2[j]+2*s3[j]+s4[j])*dt/6;

			particle->second->setPosition(vars[0]);
			particle->second->setVelocity(vars[1]);
		}	     
		//Verlangsamung
		//bei Tests aufpassen, gegebenenfalls auskommentieren oder Faktor anpassen
		
        Vector3<float> vel = particle->second->getVelocity() * 0.97f;
		particle->second->setVelocity(vel);
	}
		particle->second->setForce(Vector3<float>(0.0f, 0.0f, 0.0f));
	}
}
*/
//----------------------------------------------------------------------------------------------------------------


// Kollisionserkennung und -behandlung


/**
* \brief Vektoren für in Frage kommende Kollisionsobjekte
* param Pointer auf Objekte
*/
void ParticleSystem::lookForCollisionsWith(CapsulePtr caps)
{
	mCaps.push_back(caps);
}


void ParticleSystem::lookForCollisionsWith(SpherePtr sphere)
{
	mSphere.push_back(sphere);
}


void ParticleSystem::lookForCollisionsWith(PlanePtr plane)
{
	mPlane.push_back(plane);
}

void ParticleSystem::lookForCollisionsWith(BoxPtr box)
{
	mBox.push_back(box);
}
/**
* \brief Startet die Kollisionserkennung
* Ruft Brute-force die collide() Funktionen für alle Objekte gegeneinander auf.
*/
void ParticleSystem::checkCollisions() {

	std::map<Id, ParticlePtr>::iterator	endOfItr = mParticleList.end();
	for (std::map<Id, ParticlePtr>::iterator particle = mParticleList.begin(); particle != endOfItr; ++particle) {
		std::vector<CapsulePtr>::iterator endOfItr2 = mCaps.end();
		for(std::vector<CapsulePtr>::iterator cap = mCaps.begin(); cap != endOfItr2; ++cap) // Schleifendurchlauf über kollisionsrelevante Objekte in einem Vektor
        {
			ParticlePtr obj1 = particle->second;
			CapsulePtr obj2 = *cap;
			collide(obj1,obj2);
        }
    }

	for (std::map<Id, ParticlePtr>::iterator particle = mParticleList.begin(); particle != mParticleList.end(); ++particle) {	
		for(std::vector<SpherePtr>::iterator sphere = mSphere.begin(); sphere != mSphere.end(); ++sphere) // Schleifendurchlauf über kollisionsrelevante Objekte in einem Vektor
        {
			ParticlePtr obj1 = particle->second;
			SpherePtr obj2 = *sphere;
			collide(obj1,obj2);
        }
    }

	for (std::map<Id, ParticlePtr>::iterator particle = mParticleList.begin(); particle != mParticleList.end(); ++particle){
		for(std::vector<PlanePtr>::iterator plane = mPlane.begin(); plane != mPlane.end(); ++plane)
		{
			ParticlePtr obj1 = particle->second;
			PlanePtr obj2 = *plane;
			collide(obj1,obj2);
		}
	}
	for (std::map<Id, ParticlePtr>::iterator particle = mParticleList.begin(); particle != mParticleList.end(); ++particle){
		for(std::vector<BoxPtr>::iterator box = mBox.begin(); box != mBox.end(); ++box)
		{
			ParticlePtr obj1 = particle->second;
			BoxPtr obj2 = *box;
			collide(obj1,obj2);
		}
	}
}

void ParticleSystem::collide(ParticlePtr part, CapsulePtr caps)
{
    //part->setIsColliding(false);
	Vector3<float> unitV(0,1,0);
	Vector3<float> rV(qRotate(unitV, caps->getRigidBody()->getOrientation())); 

	Vector3<float> A1 = caps->getRigidBody()->getPosition()+(rV*(caps->getHeight()/2));
	Vector3<float> A2 = caps->getRigidBody()->getPosition()-(rV*(caps->getHeight()/2));
	Vec3 nullVector = Vec3(0.0,0.0,0.0);
	
	// teile des codes Copyright 2001, softSurfer (www.softsurfer.com)

	// alles geschummelt. Kugelzentrum als infinitisimal kleines Segment interpretiert.

	Vector3<float> smallNumberVector(0.1f,0.1f,0.1f);

	Vector3<float> B1 = part->getPosition() + smallNumberVector;
	Vector3<float> B2 = part->getPosition() - smallNumberVector;

	Vector3<float>   u = A2 - A1;
	Vector3<float>   v = B2 - B1;
	Vector3<float>   w = A1 - B1;
	float    a = dot(u,u);        // always >= 0
	float    b = dot(u,v);
	float    c = dot(v,v);        // always >= 0
	float    d = dot(u,w);
	float    e = dot(v,w);
	float    D = a*c - b*b;       // always >= 0
	float    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
	float    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (D < 0.0000000001) { // the lines are almost parallel
		sN = 0.0;        // force using point P0 on segment S1
		sD = 1.0;        // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	else {                // get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}
	if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d + b);
			sD = a;
		}
	}
	// finally do the division to get sc and tc
	if(fabs(sN) < 0.000000001)
		sc = 0.0;
	else
		sc = sN/sD;

	if(fabs(tN) < 0.0000000001)
		tc = 0.0;
	else
		tc = tN/tD;

	// get the difference of the two closest points
	Vector3<float>  dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

	float dist = dP.length();   // return the closest distance 
	float radParticle = 3.0f;	// 3.0f für Partikelradius
	//collision-behandlung
	if (dist < (caps->getRadius()) + radParticle) 
	{
		assert(dP.length() != 0.0);
		if (dP.length() != 0.0)
			dP.normalize();

		Vector3<float> normalB = dP ;
		Vector3<float> contactPoint = part->getPosition() + (radParticle * dP);

        Vector3<float> posOldToContact = contactPoint - (part->getPosition());

        //überprüfung der normalen, falls die capsule weiter als radius eingedrungen ist, umdrehen
        if (dot(posOldToContact, normalB)<0)
        {
            //cout << "Normale " << normalB << " von B=" << caps->getId() << " zeigt in die falsche Richtung !\n";
			normalB = -normalB;
        }
        else
        {
			// cout << "Normale " << normalB << " von B=" << caps->getId() << " zeigt in die richtige Richtung !\n";          
        }


		const Vector3<float>& rA = contactPoint - caps->getRigidBody()->getPosition();
		const Vector3<float>& aVelocityA = caps->getRigidBody()->getAngularVelocity();


		Vector3<float> velA = part->getVelocity();
		Vector3<float> forceA = part->getForce();
		

		const Vector3<float>& velocityB = part->getVelocity();
		const Vector3<float>& velocityA = caps->getRigidBody()->getVelocity() + cross(aVelocityA, rA);

		const Vector3<float>& vR = velocityA - velocityB;
		
		float vN = dot(vR, normalB);    

		//test auf colliding contact
		if (vN <= 0.03)	// Wert: 0.5
		{

			//Vector3<float> normale = -1 * part->getPosition() + caps->getPosition();
			//normalB.normalize();




//-----------Marke Eigenbau--------------------------------------------------------------------------------		
			//float vDoN = dot(velA, normalB);
			//Vector3<float> vS = normalB * vDoN; 
		
			//float fDoN = dot(forceA, normalB);
			//Vector3<float> fS = normalB * fDoN;
			//Vector3<float> force = part->getForce();
			////force.normalize();
			//if (dot (force, normalB) >= 0.0 ){
			//	part->setVelocity(part->getVelocity() - vS);
			//	part->setVelocity(part->getVelocity() * 0.75);	// Wert: 0.75
			//	part->addForce((-1) * fS);
			//	part->setForce(part->getForce() * 0.75);	// Wert: 0.75
			//	part->setIsColliding(true);
			//	}
			//
			//part->setVelocity(part->getVelocity() * vDoN);
			//part->setForce(part->getForce() * fDoN);


//------------Kopie aus Sphere--------------------------------------------------

		float vDoN = dot(velA, normalB);
		Vector3<float> vS = normalB * vDoN; 
		
		float fDoN = dot(forceA, normalB);
		Vector3<float> fS = normalB * fDoN;
		Vector3<float> force = part->getForce();
		if (force.length() != 0.0)
			force.normalize();
		if (dot (force, normalB) > 0 ){
			
			part->setVelocity(part->getVelocity() - vS);
			part->setVelocity(part->getVelocity() * 0.9f);	// Wert: 0.75
			part->setVelocity(part->getVelocity() + caps->getRigidBody()->getVelocity());
			part->addForce((-1.f) * fS);
			part->setForce(part->getForce() * 0.9f);	// Wert: 0.75
			//part->addForce(caps->getRigidBody()->getForce());
			part->setIsColliding(true);
			part->setIsCollidingWhere(1);
			
			
		}
		else if (dot (force, normalB) < 0){
			part->setVelocity(part->getVelocity() + vS);
			part->setVelocity(part->getVelocity() * 0.9f);	// Wert: 0.75
			part->setVelocity(part->getVelocity() + caps->getRigidBody()->getVelocity());
			part->addForce((1.f) * fS);
			part->setForce(part->getForce() * 0.9f);	// Wert: 0.75
			//part->addForce(caps->getRigidBody()->getForce());
			part->setIsColliding(true);
			part->setIsCollidingWhere(-1);
			}
//--------------------------neue Kollisionsbehandlung-------------------------------------------------------------
			/*	float damping = 1.0f; 
			//float damping = (0.1f + caps->getBounciness()) / 2.0f;
			
			float invMassA = caps->getInvMass();			
			float invMassB = part->getInvMass();//1.0f / part->getMass();
			Matrix<float> invTensorA = caps->getInvWorldInertiaTensor();
			
			
			const Vector3<float>& crossA = cross(rA,normalB);
			
			
			const Vector3<float>& uA = invTensorA * crossA;
			

 
//		fNum = damping *( -(1+(-0.5f)) * ....
			
			float fNum = damping*(-(1+0.0f)*(dot(normalB,(velocityA - velocityB)) + dot(aVelocityA,crossA)));
			float fDenom = invMassA + invMassB + dot(crossA,uA);
			float impulseMagnitude = fNum/fDenom;

//			if (impulseMagnitude > 0)
			{
			Vector3<float> impulseForce = impulseMagnitude * normalB; //impulseMagnitude * 2.0 * normalB
			
			if (part->getInvMass()!=0)
			// Velocity setzen
			part->setVelocity(part->getVelocity() - (impulseForce * invMassB));
			}*/
		}
//--------------------keine Ahnung was ist----------------------------------------------------
		/*
		if (vN > -1.5 && vN < 1.5){
				//Vector3<float> normale = -1 * part->getPosition() + caps->getPosition();
			normalB.normalize();
		
			float vDoN = dot(velA, normalB);
			Vector3<float> vS = normalB * vDoN; 
		
			float fDoN = dot(forceA, normalB);
			Vector3<float> fS = normalB * fDoN;
			Vector3<float> force = part->getForce();
			//force.normalize();
			if (dot (force, normalB) >= -0.5 ){
				part->setVelocity(part->getVelocity() - (vS * 1.2));
				//part->setVelocity(Vector3<float>(0.0,0.0,0.0));
				//part->setForce(Vector3<float>(0.0,0.01,0.0));
				
				part->addForce((-1.2) * fS);
				}*/
//---------------------------------------------------------------------------------------------------------
			/*vDoN *= 50;
			  fDoN *= 50;*/
			//part->setVelocity(part->getVelocity() * vDoN);
			//part->setForce(part->getForce() * fDoN);
		}
	}
	


/**
* \brief Collision mit einer anderen Sphere
* \param sphere zu testende Sphere
*/

void ParticleSystem::collide(ParticlePtr part, SpherePtr sphere)
{
	
	Vector3<float> distance = part->getPosition() - sphere->getRigidBody()->getPosition();
    float radiusSum = sphere->getRadius();
    
    //test auf collision
	if (distance.length() <= radiusSum)	// sonst nur radiusSum
    {
        //berechnung normalB und contactPoint
		distance.normalize();
        const Vector3<float>& normalB = distance;
        //const Vector3<float>& contactPoint = part->getPosition(); //+ 0.01f * normalB;
      
		// folgender Abschnitt aus: 
		// CollisionSystem::collisionResponse(part, sphere, contactPoint, normalB);

		//berechnung der relativen geschwindigkeit in richtung normalB
		const Vector3<float>& velocityA = part->getVelocity();
		const Vector3<float>& velocityB = sphere->getRigidBody()->getVelocity();
		Vector3<float> velA = part->getVelocity();
		Vector3<float> forceA = part->getForce();
		Vector3<float> velB = sphere->getRigidBody()->getVelocity();
		Vector3<float> forceB = sphere->getRigidBody()->getForce();
		
		const Vector3<float>& vR = velocityA - velocityB;
		float vN = dot(vR, normalB);		

		//behandlung für colliding contact
		if (vN <= 0.5)
		{
			//Kraftberechnung für Resting Contact
//Todo
//--------------bisherige Behandlung----------------------------------------------
			
			Vector3<float> normale = -1 * part->getPosition() + 
				sphere->getRigidBody()->getPosition();
			normale.normalize();
		
			float vADoN = dot(velA, normale);
			Vector3<float> vS = normale * vADoN; 
			
			float fADoN = dot(forceA, normale);
			Vector3<float> fS = normale * fADoN;

			float vBDoN = dot (velA, normale);
			Vector3<float> vS2 = normale * vBDoN;
				
			float fBDoN = dot (forceB, normale);
			Vector3<float> fS2 = normale * fBDoN;

			Vec3 nullvector = Vec3(0.0,0.0,0.0);
			Vector3<float> force = part->getForce();
			force.normalize();
			if (dot (force, normale) > 0 ){
				part->setVelocity(part->getVelocity() - vS);
				part->setVelocity(part->getVelocity() * 0.9f);	// Wert: 0.75
				part->setVelocity(part->getVelocity() + sphere->getRigidBody()->getVelocity());
				part->addForce((-1.f) * fS);
				part->setForce(part->getForce() * 0.9f);	// Wert: 0.75
				part->addForce(sphere->getRigidBody()->getForce());
				
				if (!(sphere->getRigidBody()->getVelocity() == Vec3(0.0,0.0,0.0))){
					sphere->getRigidBody()->addForce((-0.02f) * fS2);
					sphere->getRigidBody()->
						setVelocity(sphere->getRigidBody()->getVelocity() - 
									(vS2 * 0.02f));
				}
				
				part->setIsColliding(true);
			}else if (dot (force, normale) < 0){
				part->setVelocity(part->getVelocity() + vS);
				part->setVelocity(part->getVelocity() * 0.9f);	// Wert: 0.75
				part->setVelocity(part->getVelocity() + 
								  sphere->getRigidBody()->getVelocity());
				part->addForce((1.0) * fS);
				part->setForce(part->getForce() * 0.9f);	// Wert: 0.75
				part->addForce(sphere->getRigidBody()->getForce());
				
				if (!(sphere->getRigidBody()->getVelocity() == Vec3(0.0,0.0,0.0))){
					sphere->getRigidBody()->addForce((-0.02f) * fS2);
					sphere->getRigidBody()
						->setVelocity(sphere->getRigidBody()->getVelocity() - 
									  (vS2 * 0.02f));
				}
				
			}

		
//----------------alte Behandlung-------------------------------------------------
			//berechnung der impuls-kraft in richtung normalB
/*		float damping = 0.5f; //(0.6 + sphere->getBounciness()) / 2.0;
  float invMassA, invMassB = 0.0;
  Matrix<float> invTensorA(3,3);
  Matrix<float> invTensorB(3,3);
  Vec3 rA = part->getForce();
  Vec3 rB = sphere->getRigidBody()->getForce();
  //test ob dynamisch
  if(part->getIsDynamicFlag())
  {	
  invMassA = 1.0f / part->getMass();
  }
  else 
  invMassA = 0.0;

  if(sphere->getRigidBody()->getIsDynamicFlag())
  {	
  invMassB = 1.0f / sphere->getMass();
  invTensorB = sphere->getRigidBody()->getInvWorldInertiaTensor();
  }
  else 
  invMassB = 0.0;
		
  const Vector3<float>& crossA = cross(rA,normalB);
  const Vector3<float>& crossB = cross(rB,normalB);

  const Vector3<float>& uA = invTensorA * crossA;
  const Vector3<float>& uB = invTensorB * crossB;

  float fNum = -(1+damping)*(dot(normalB,(velocityA - velocityB)));
  float fDenom = invMassA + invMassB + dot(crossA,uA)+dot(crossB,uB);

  float f = fNum / fDenom;
		
  Vector3<float> impulseForce = 9.5 * f * normalB;
		
  //jetzt:
  part->setVelocity(part->getVelocity() + impulseForce/part->getMass());
*/
//----------neue Behandlung-------------------------------------------------------		 	
		
			////berechnung der impuls-kraft in richtung normalB
			//float damping = 0.1f;
			//float invMassA = part->getInvMass();
			//	float invMassB = sphere->getInvMass();
			// 
			//    // calculate impulse
			//float fNum = -(/*1+*/damping)*(dot(normalB,(velocityB - velocityA)));
			//float fDenom = invMassA + invMassB;
			//
			//float impulseMagnitude = fNum / fDenom;

			//if (impulseMagnitude < 0){

			//Vector3<float> impulseForce = impulseMagnitude * normalB;
			//                                   
			//if (part->getInvMass()!=0)
			//	part->setVelocity(part->getVelocity() - (impulseForce/invMassA));
			//
			//if (sphere->getInvMass()!=0)
			//	sphere->setVelocity(sphere->getVelocity() - (impulseForce * invMassB));
			//}
//--------------------------------------------------------------------------------
		}
	}
}

						
/**
* \brief Collision mit einem Plane.
* \param plane zu testende Ebene
*/


void ParticleSystem::collide(ParticlePtr part, PlanePtr plane) {
	//SimonState::exemplar()->messages << "Cloth: Kollision Kugel vs. Ebene gestartet" << SimonState::endm;

	float distance = dot(part->getPosition() - 
						 plane->getRigidBody()->getPosition(), 
						 plane->getNormal());
	Vector3<float> n = plane->getNormal();
    Vector3<float> v = part->getVelocity(); 
	
	float vDotN = dot (v, n);
    //Vector3<float> f = getRigidBody()->getForce(); 	
    
    // no resting contact. check collision
    if ( distance <= 3.0)
    {   
        if (vDotN<=0.5f)
        {
						 
			// normal colliding contact:
			
			Vector3 <float> velPart = part->getVelocity();
			
			Vector3 <float> normPlane = plane->getNormal();
			normPlane.normalize();
			float vDoN = dot(velPart,  normPlane);
			normPlane *= vDoN;
			part->setVelocity(part->getVelocity() - normPlane);
			//cout << "Vel aus col " << part->getVelocity()<< endl;
			normPlane = plane->getNormal();
			normPlane.normalize();
			Vector3 <float> forcePart = part->getForce();
			float fDotN = dot(forcePart, normPlane);
			normPlane *= fDotN;
			part->addForce((-1) * normPlane);
			//cout << "Acc aus Col " << part->getAcceleration() << endl;
			// factor in elasticity
			// geschummelt (eigentlich getBounciness)
			//vDotN *= 1 + 2.5f;
			// reflect veloctiy along normal
			//part->setVelocity( Vector3<float>(v[X] - vDotN*n[X], v[Y] - vDotN*n[Y], v[Z] - vDotN*n[Z]));
			SimonState::exemplar()->messages << "Cloth: Kollision Particle vs. Ebene beendet" << SimonState::endm;
        }
    }
}


/**
* \brief Collision Partikel mit einer Box.
* \param box zu testende Box
*/
void ParticleSystem::collide(ParticlePtr part, BoxPtr box)
{ 
    //sphere in das lokale koordinatensystem der box bringen
	Vector3<float> particlePos = part->getPosition() - box->getRigidBody()->getPosition();
	Quaternion boxOri = box->getRigidBody()->getOrientation();
	const Quaternion& boxInvOri = boxOri.inverse();    
    particlePos = qRotate(particlePos, boxInvOri);
    
    //test auf collision mit den 6 seiten der box
    unsigned int cnt = 0;
	float rad = 3.0f;

	float right = dot(particlePos, Vector3<float>(1,0,0)) + rad;
	float left = dot(particlePos, Vector3<float>(-1,0,0)) - rad;
	float front = dot(particlePos, Vector3<float>(0,0,1)) + rad;
	float back = dot(particlePos, Vector3<float>(0,0,-1)) - rad;
	float top = dot(particlePos, Vector3<float>(0,1,0)) + rad;
	float bottom = dot(particlePos, Vector3<float>(0,-1,0)) - rad;
	
	
    float dR  = box->getWidth()/2 - right + rad;
    float dL  = box->getWidth()/2 - left + rad;
    float dF  = box->getDepth()/2 - front + rad;
    float dBa = box->getDepth()/2 - back + rad;
    float dT  = box->getHeight()/2 - top + rad; 
    float dBo = box->getHeight()/2 - bottom + rad; 
    
    if (right  >= 0 && dR >= 0)  
	{	
		cnt ++;
	    if (right >  box->getWidth()/2)
		  particlePos[X] = box->getWidth()/2;	
		else  		
		  particlePos[X] = right ;	
	}
	if (left   >  0 && dL >= 0)  
	{
		cnt ++;	
		if(left > box->getWidth()/2)
		  particlePos[X] = -box->getWidth()/2;
		else
		  particlePos[X] = -left ;
	}
	if (front  >= 0 && dF >= 0)  
	{
		cnt ++;
		if (front > box->getDepth()/2)  
		  particlePos[Z] = box->getDepth()/2;
		else
		  particlePos[Z] = front;	  
 	}
	if (back   >  0 && dBa >= 0)  
	{
		cnt ++;
		if (back > box->getDepth()/2)  
		  particlePos[Z] = -box->getDepth()/2;
		else
		  particlePos[Z] = -back;  	
 	}
	if (top    >= 0 && dT >= 0) 
	{
		cnt ++;
		if (top > box->getHeight()/2)  
		  particlePos[Y] = box->getHeight()/2;
		else
		  particlePos[Y] = top;	  
 	}
	if (bottom >  0 && dBo >= 0) 
	{
		cnt ++;
		if (bottom > box->getHeight()/2)  
		  particlePos[Y] = -box->getHeight()/2;
		else
		  particlePos[Y] = -bottom  ;
	}
    
	//test auf collision
    if (cnt >= 3)
	{
		
		//berechnung normalB und contactPoint
		const Vector3<float>& contactPoint 
			= qRotate(particlePos, boxOri) + box->getRigidBody()->getPosition();
		//cout << "C: " << contactPoint << endl;
		//cout << "P: " << part->getPosition() << endl;
		Vector3<float> normalB = part->getPosition() - contactPoint;
		//cout << normalB.length() <<"\n";

		//if (normalB.length() != 0)
		normalB.normalize();
		
		// folgender Abschnitt aus: 
		// CollisionSystem::collisionResponse(part, sphere, contactPoint, normalB);

		//berechnung der relativen geschwindigkeit in richtung normalB
		const Vector3<float>& velocityA = part->getVelocity();
		const Vector3<float>& velocityB = box->getRigidBody()->getVelocity();
		Vector3<float> velA = part->getVelocity();
		Vector3<float> forceA = part->getForce();


//		const Vector3<float>& rA = contactPoint - part->getPosition(); // vorher: particlePos!
//		const Vector3<float>& rB = contactPoint - box->getPosition();
		
		const Vector3<float>& vA = velocityA;
		const Vector3<float>& vB = velocityB;
		const Vector3<float>& vR = vA - vB;
		
		float vN = dot(vR, normalB);
		
//------------------Behandlung für colliding contact---------------------------------------
		if (vN <= 0.5)
		{	

//--------------------neue Behandlung aus Sphere kopiert-------------------------
			float vDoN = dot(velA, normalB);
			Vector3<float> vS = normalB * vDoN; 
		
			float fDoN = dot(forceA, normalB);
			Vector3<float> fS = normalB * fDoN;
			Vector3<float> force = part->getForce();
			force.normalize();
			if (dot (force, normalB) > 0 ){
				part->setVelocity(part->getVelocity() - vS);
				part->setVelocity(part->getVelocity() * 0.75);	// Wert: 0.75
				part->addForce((-1.0) * fS);
				part->setForce(part->getForce() * 0.75);	// Wert: 0.75
				part->setIsColliding(true);
			}
//------------------------------------------------------------------------------
			////berechnung der impuls-kraft in richtung normalB
			////float damping = (0.01f + box->getBounciness()) / 2.0f;
			//float damping = 0.12f;
			//float invMassA, invMassB = 0.0f;
			////Matrix<float> invTensorA(3,3);

			////Matrix<float> invTensorB = box->getInvWorldInertiaTensor();
		
			////test ob dynamisch
			//if(part->getIsDynamicFlag())
			//{	
			//	invMassA = part->getInvMass();
			//}
			//else 
			//	invMassA = 0.0;
			//
			//if(box->getRigidBody()->getIsDynamicFlag())
			//{	
			//	invMassB = box->getInvMass();
			//	//invTensorB = box->getRigidBody()->getInvWorldInertiaTensor();
			//}
			//else 
			//	invMassB = 0.0;
		
			//float fNum = -(/*1+*/damping)*(dot(normalB,(velocityA - velocityB)));
			//float fDenom = invMassA + invMassB;// + dot(crossA,uA)+dot(crossB,uB);

			//float impulseMagnitude = fNum / fDenom;

			//if (impulseMagnitude < 0){
			//Vector3<float> impulseForce = impulseMagnitude * normalB;

			///*impulseForce.normalize();
			//float vDoI = dot(velA, impulseForce);
			//float fDoI = dot(forceA, impulseForce);
			//Vec3 vS = impulseForce * vDoI;
			//Vec3 fS = impulseForce * fDoI;*/

			////part->setVelocity(part->getVelocity()[X], 0.0f, part->getVelocity()[Z]);
			////part->setVelocity(part->getVelocity() - vS);
			////part->addForce(fS);
			////setzen der linaren momente
			//if (part->getInvMass()!=0)
			//{
			//part->setVelocity(part->getVelocity() - impulseForce/invMassA);
			//part->setIsColliding(true);
			//}
			////part->setVelocity(part->getVelocity() + (impulseForce / invMassB));        
			//
			//}
		}
	}
	
}

//-----------------------------------------------------------------------------
//---------Test Verlet----------------
void ParticleSystem::integrateVerletBaltman (double interval)
{

	// Iteratoren
	std::map<Id,  ParticlePtr >::iterator particleIterator;
	std::map<Id,  ParticlePtr >::iterator particleEnd;

	// aktuelle Werte
	ParticlePtr particle; // Partikel
	Vec3 position;                 // lineare Position (Verschiebung)
	Quaternion orientation;        // angulaere Position (Rotation als Quat)
	Vec3 velocity;                 // lineare Geschwindigkeit
	Vec3 velocityAngular;          // Winkelgeschwindigkeit
	Vec3 acceleration;             // lineare Beschleunigung
	Vec3 accelerationAngular;      // Winkelbeschleunigung
	Quaternion orientationDot;     // Ableitung der Orientierung nach der Zeit
	Quaternion orientationDotDot;  // 2. Ableitung der Orientierung nach der Zeit
	double dt = interval;          // Delta time
	double dt2 = interval * interval;  // Delta time squared
	particleEnd =  mParticleList.end ();

	// for all Particles
	for (	particleIterator = mParticleList.begin(); particleIterator != particleEnd; ++particleIterator) {

		particle = particleIterator->second;

		// Ueberspringe den RigidBody, wenn die isDynamic Flag false ist
		if (!(particle->getIsDynamicFlag())) {

			// Setze Kraft und Drehmoment Null
			//particle->setForce(Vector3<float>(0, 0, 0));
			//particle->setTorque(Vector3<float>(0, 0, 0));
			continue;
		}

		// Werte lesen
		position = particle->getPosition ();
		//orientation = particle->getOrientation ();
		velocity = particle->getVelocity ();
		//velocityAngular = particle->getAngularVelocity (); //Winkelgeschwindigkeit nicht benötigt bei Particle
		acceleration = particle->getAcceleration ();
		//accelerationAngular = particle->getAngularAcceleration ();//nicht benötigt bei Particle

		// Ableitungen der Orientierung
/*    orientationDot = 0.5 * Quaternion (velocityAngular) * orientation;
	  orientationDotDot = (0.5 * Quaternion(0.0, accelerationAngular [0],
	  accelerationAngular [1],
	  accelerationAngular [2]) *
	  orientation +
	  0.5 * Quaternion (0.0, velocityAngular [0],
	  velocityAngular [1],
	  velocityAngular [2]) *
	  orientationDot);

*/

/*
// Alternative
orientationDotDot = (0.5) *
Quaternion (-2.0 * dot (orientationDot, orientationDot),
accelerationAngular [0],
accelerationAngular [1],
accelerationAngular [2]) *
orientation;
*/
				
		// Linear Update
		position += ((float)dt * velocity + 0.5f * (float)dt2 * acceleration);
		velocity += ((float)dt * acceleration);

		// Angular Update
		//orientation += (dt * orientationDot + 0.5 * dt2 * orientationDotDot);
		//velocityAngular += (dt * accelerationAngular);
		//orientation.normalize();

		if (SimonState::exemplar()->getViscosityFlag()) {
			velocity *= SimonState::exemplar()->getViscositySlowdownLinear();
			//velocityAngular *= mViscositySlowdownAngular;
		}
		
		// Zurueckschreiben der Werte
		particle->setPosition (position);
		//particle->setOrientation (orientation);
		particle->setVelocity (velocity);
		//particle->setAngularVelocity (velocityAngular);

		// Setzen von Kraft und Drehmoment auf Null
		particle->setForce(Vector3<float>(0, 0, 0));
		//particle->setTorque(Vector3<float>(0, 0, 0));//nicht nötig bei Particle

	}
}
  
