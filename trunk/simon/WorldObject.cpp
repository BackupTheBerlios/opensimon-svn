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


/**
*  \class WorldObject
*/

#include <simon/WorldObject.h>
#include <simon/config.h>
#include <float.h> //für ISNAN/ISINF

#include <assert.h>
#include <math.h>



/**
 * \brief Default-Konstruktor
 *
 * Setzt Geschwindigkeit, Beschleunigung, Kraft und Position auf (0, 0, 0)
 * Masse gleich 0 und die "darfSichBewegen-Flag" mIsDynamic auf true.
 */
WorldObject::WorldObject() {
	mInvMass = 1.0;
	mVelocity = Vec3(0.0, 0.0, 0.0);
	mAcceleration = Vec3(0.0, 0.0, 0.0);
	mForce = Vec3(0.0, 0.0, 0.0);
	mPosition = Vec3(0.0, 0.0, 0.0);
	
	// mPrevPosition muss auf mPosition gesetzt werden
	mPrevPosition = mPosition;
	
	//! Standardmässig dürfen sich Objekte bewegen und sind nicht eingefroren
	mIsDynamic = true;
	mIsFrozen = false;	
	// Standardmässig verlangsamen sich Objekte nicht.
	

	
}

/**
 * \brief Destruktor
 */
WorldObject::~WorldObject() {

}



// *********************************
// S E T T E R
// *********************************



/**
* \brief Setzt die ID
*/
void WorldObject::setId(Id id)
{
	mId = id;
}



/**
* \brief Setzt die Masse in Gramm
* \param mass
*/
void WorldObject::setMass(float mass)
{
	assert(mass);
	mInvMass = 1.0f / mass;
}

/**
* \brief Setzt die inverse Masse in Gramm
* \param mass
*/
void WorldObject::setInvMass(float invMass)
{
	mInvMass = invMass;
}



/**
* \brief Setzt die Kraft als Vector
*/
void WorldObject::setForce(Vec3 force)
{
	mForce = force;
	mAcceleration = mForce * getInvMass();
}


/**
* \brief Setzt die Beschleunigung als Vector
*/
void WorldObject::setAcceleration(Vec3 acceleration)
{
	assert(!ISNAN(acceleration[0]) || !ISNAN(acceleration[1]) || !ISNAN(acceleration[2]));
	assert(!ISINF(acceleration[0]) || !ISINF(acceleration[1]) || !ISINF(acceleration[2]));

	mAcceleration = acceleration;
	mForce = mAcceleration * getMass();
}

/**
* \brief Setzt die vorige Beschleunigung als Vector
*/
void WorldObject::setPrevAcceleration(Vec3 acceleration)
{
        mPrevAcceleration = acceleration;
        mPrevForce = mPrevAcceleration * getMass();
}


/**
* \brief Setzt die Geschwindigkeit als Vektor
*/
void WorldObject::setVelocity(Vec3 velocity)
{
	assert(!ISNAN(velocity[0]) || !ISNAN(velocity[1]) || !ISNAN(velocity[2]));
	assert(!ISINF(velocity[0]) || !ISINF(velocity[1]) || !ISINF(velocity[2]));

	mPrevVelocity =  mVelocity;
	mPrevLinearMomentum = mLinearMomentum;
	mVelocity = velocity;
	mLinearMomentum = getMass() * velocity;
}

/**
* \brief Setzt die vorherige Geschwindigkeit als Vektor
*/
void WorldObject::setPrevVelocity(Vec3 prevVelocity){
	mPrevVelocity = prevVelocity;
	mPrevLinearMomentum = getMass() * prevVelocity;
}

/**
* \brief Macht den letzten aufruf von setVelocity rückgängig.
*/
void WorldObject::undoSetVelocity(){
	mVelocity = mPrevVelocity;
	mLinearMomentum = mPrevLinearMomentum;
}




/**
* \brief Setzt das lineare Moment als Vektor
*/
void WorldObject::setLinearMomentum(Vec3 linearMomentum){
	assert(!ISNAN(linearMomentum[0]) || !ISNAN(linearMomentum[1]) || !ISNAN(linearMomentum[2]));
	assert(!ISINF(linearMomentum[0]) || !ISINF(linearMomentum[1]) || !ISINF(linearMomentum[2]));

	mPrevLinearMomentum =  mLinearMomentum;
	mPrevVelocity = mVelocity;
	mLinearMomentum = linearMomentum;	
	mVelocity = mInvMass * linearMomentum;
}

/**
* \brief Setzt das vorherige lineare Moment als Vektor
*/
void WorldObject::setPrevLinearMomentum(Vec3 prevLinearMomentum)
{
	mPrevLinearMomentum = prevLinearMomentum;
	mPrevVelocity = mInvMass * prevLinearMomentum;
}

/**
* \brief Macht den letzten aufruf von setLinearMomentum rückgängig.
*/
void WorldObject::undoSetLinearMomentum()
{
	mLinearMomentum = mPrevLinearMomentum;
	mVelocity = mPrevVelocity;
}

/**
* \brief Setzt die Position als Vektor
*/
void WorldObject::setPosition(Vec3 position)
{
	assert(!ISNAN(position[0]) || !ISNAN(position[1]) || !ISNAN(position[2]));
	assert(!ISINF(position[0]) || !ISINF(position[1]) || !ISINF(position[2]));
	mPrevPosition = mPosition;
	mPosition = position;	
}

/**
* \brief Macht den letzten aufruf von setPosition rückgängig.
*/
void WorldObject::undoSetPosition()
{
	mPosition = mPrevPosition;
}
	
/**
* \brief Setzt die verherige Position als Vector
*/
void WorldObject::setPrevPosition(Vec3 prevPosition)
{
	mPrevPosition = prevPosition;
}


/**
 * \brief Setzt die "DarfSichBewegen-Flag".
 *
 * \see RigidBodySystem 
 *
 * \param bool Flag, die angibt, ob sich das Objekt bewegen darf, oder nicht. 
 *
 * Diese Flag wird vom Integrator im RigidBodySystem ausgewertet.
*/ 
void WorldObject::setIsDynamicFlag(bool flag){
    mIsDynamic = flag;
}




// *********************************
// G E T T E R
// *********************************




/**
* \brief Gibt die ID zurück
* \return Id
*/
Id const WorldObject::getId()
{
	return mId;
}


/**
* \brief Gibt die Masse zurück
* \return float
*/
float const WorldObject::getMass()
{
	if (mInvMass == 0) return 999999;
	return 1.0f / mInvMass;
}


/**
* \brief Gibt 0 wenn das objekt frozen ist, sonst die inverse Masse zurück.
* \see freeze() &  unfreeze()
* \return float
*/
float const WorldObject::getInvMass()
{
	if (mIsFrozen || !mIsDynamic)
		return 0;
	else
		return mInvMass;
}


/**
* \brief Gibt die Kraft als Vektor zurück
* \return Vec3
*/
Vec3 const WorldObject::getForce() const {
	return mForce;
}

/**
* \brief Gibt die Kraft als Vektor zurück
* \param force Rückgabe (optimiert)
*/
void WorldObject::getForce (Vec3 &force) const {
	force = mForce;
}

/**
* \brief Gibt die Beschleunigung als Vektor zurück
* \return Vec3
*/
Vec3 const WorldObject::getAcceleration() const
{
	return mAcceleration;
}

/**
* \brief Gibt die vorige Beschleunigung als Vektor zurück
* \return Vec3
*/
Vec3 const WorldObject::getPrevAcceleration() const
{
        return mPrevAcceleration;
}


/**
* \brief Gibt die Geschwindigkeit als Vektor zurück
* \return Vector3
*/
Vec3 const WorldObject::getVelocity() const
{
	return mVelocity;
}


/**
* \brief Gibt die Geschwindigkeit als Vektor zurück
* \param velo schreibziel für die velocity
*/
void WorldObject::getVelocity(Vec3 &velo) const {
	velo = mVelocity;
}

/**
* \brief Gibt die vorherige Geschwindigkeit als Vektor zurück
* \return Vector3
*/
Vec3 const WorldObject::getPrevVelocity() const
{
	return mPrevVelocity;
}

/**
* \brief Gibt das lineare Moment als Vektor zurück
* \return linearMomentum
*/
Vec3 const WorldObject::getLinearMomentum() const
{
	return mLinearMomentum;
}


/**
* \brief Gibt das lineare Moment als Vektor zurück
* \param linearMomentum
*/
void WorldObject::getLinearMomentum(Vec3 &linearMomentum) const
{
	linearMomentum = mLinearMomentum;
}

/**
* \brief Gibt das vorherige lineare Moment als Vektor zurück
* \return Vector3
*/
Vec3 const WorldObject::getPrevLinearMomentum() const
{
	return mPrevLinearMomentum;
}

/**
* \brief Gibt die Position als Vektor zurück
* \return position in Worldspace
*/
Vec3 const WorldObject::getPosition() const
{
	return mPosition;
}


/**
* \brief Gibt die Position als Vektor zurück
* \param where position in worldspace should be written
*/
void WorldObject::getPosition(Vec3 &pos) const
{
	pos = mPosition;
}


/**
* \brief Setzt die verherige Position als Vector
*/
Vec3 const WorldObject::getPrevPosition() const {
	return mPrevPosition;
}



/**
 * \brief Liefert den Status der "DarfSichBewegen-Flag".
 *
 * \see RigidBodySystem 
 *
 * \return bool Status, ob sich das Objekt bewegen darf, oder nicht. 
 *
*/     
bool const WorldObject::getIsDynamicFlag() const{
    return mIsDynamic;
}     



// *********************************
// O T H E R 
// *********************************

/**
* \brief vorübergehendes einfrieren, wird für Shock Propagation gebraucht
*/
void WorldObject::freeze()
{
	mIsFrozen = true;
}

/**
* \brief vorübergehendes wieder ausfrieren., wird für Shock Propagation gebraucht
*/
void WorldObject::unfreeze()
{
	mIsFrozen = false;
}

/**
*	\brief Abfrage ob eingefroren
*/
bool const WorldObject::getIsFrozen()
{
	return mIsFrozen;
}


/**
* \brief Fügt eine Kraft hinzu
*/
void WorldObject::addForce(Vec3 force)
{
	mForce += force;
	mAcceleration = mForce * getInvMass();
	
	/*for (int i = 0; i < 3; i++) {
		if (fabs(mForce[i]) <= 1.0e-09) {
			mForce[i]=0;
		}
		if (fabs(mAcceleration[i]) <= 1.0e-09) {
			mAcceleration[i]=0;
		}
	}*/
}

