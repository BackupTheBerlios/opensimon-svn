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
 * \file SmartPointer.h
 * \class SmartPointer
 *
 * \brief Copyable SmartPointer 
 * The SmartPointer template encapsulates a Pointer of a given type.
 * It maintains a double linked list to keep track of its copies
 * When the last copy gets destructed, the encapsulated pointer will be deleted.
 *
 * This is done by the boost::shared_ptr class.
 *
 * There are a lot of type defintions for easy use of smartpointerstuff.
 */
//------------------------------------------------------------------------------
#ifndef SMART_POINTER_H
#define SMART_POINTER_H


//! <b> ACHTUNG <b> Weiter unten gibt es sehr praktische Typedefs!!!

#include <boost/shared_ptr.hpp>

#define SmartPointer boost::shared_ptr


//! forward declarations for typedefs
class WorldObject;
class Particle;
class Box;
class Sphere;
class Plane;
class Capsule;
class Geometry;

class RigidBody;

class Connection;

class PrimaryConstraint;
class BallAndSocketConstraint;
class HingeConstraint;
class FixedConstraint;
class ThreadConstraint;
class ConstraintMatrixNode;

class SpringConnection;

class ParticleSystem;
class RigidBodySystem;
class ConstraintSystem;
class GeometrySystem;
class ClothSystem;

class BaseEvent;

class ToSimonGeoBoxEvent;
class ToSimonGeoPlaneEvent;
class ToSimonGeoSphereEvent;
class ToSimonGeoCapsuleEvent;

class ToSimonParticleEvent;

class ClothChangeEvent;
class PositionChangedEvent;
class ToSimonCommandEvent;
class ToSimonBeamEvent;
class TransformationChangeEvent;

class ToSimonConstraintBalljointEvent;
class ToSimonConstraintHingejointEvent;
class ToSimonConstraintThreadEvent;

class ToSimonClothEvent;

class ContactInformationContainer;

class Interference;

typedef SmartPointer<WorldObject> WorldObjectPtr;
typedef SmartPointer<Particle> ParticlePtr;
typedef SmartPointer<Box> BoxPtr;
typedef SmartPointer<Sphere> SpherePtr;
typedef SmartPointer<Plane> PlanePtr;
typedef SmartPointer<Capsule> CapsulePtr;
typedef SmartPointer<Geometry> GeometryPtr;

typedef SmartPointer<RigidBody> RigidBodyPtr;

typedef SmartPointer<PrimaryConstraint> PrimaryConstraintPtr;
typedef SmartPointer<BallAndSocketConstraint> BallAndSocketConstraintPtr;
typedef SmartPointer<HingeConstraint> HingeConstraintPtr;
typedef SmartPointer<FixedConstraint> FixedConstraintPtr;

typedef SmartPointer<ThreadConstraint> ThreadPtr;
typedef SmartPointer<ConstraintMatrixNode> ConstraintMatrixNodePtr;

typedef SmartPointer<Connection> ConnectionPtr;

typedef SmartPointer<SpringConnection> SpringConnectionPtr;

typedef SmartPointer<ParticleSystem> ParticleSystemPtr;
typedef SmartPointer<RigidBodySystem> RigidBodySystemPtr;
typedef SmartPointer<GeometrySystem> GeometrySystemPtr;
typedef SmartPointer<ConstraintSystem> ConstraintSystemPtr;
typedef SmartPointer<ClothSystem> ClothSystemPtr;

typedef SmartPointer<BaseEvent> BaseEventPtr;

typedef SmartPointer<ToSimonGeoBoxEvent> ToSimonGeoBoxEventPtr;
typedef SmartPointer<ToSimonGeoSphereEvent> ToSimonGeoSphereEventPtr;
typedef SmartPointer<ToSimonGeoCapsuleEvent> ToSimonGeoCapsuleEventPtr;
typedef SmartPointer<ToSimonGeoPlaneEvent> ToSimonGeoPlaneEventPtr;

typedef SmartPointer<ToSimonParticleEvent> ToSimonParticleEventPtr;

typedef SmartPointer<ClothChangeEvent> ClothChangeEventPtr;
typedef SmartPointer<PositionChangedEvent> PositionChangedEventPtr;
typedef SmartPointer<ToSimonCommandEvent> ToSimonCommandEventPtr;
typedef SmartPointer<ToSimonBeamEvent> ToSimonBeamEventPtr; 
typedef SmartPointer<TransformationChangeEvent> TransformationChangeEventPtr;

typedef SmartPointer<ToSimonConstraintBalljointEvent> ToSimonConstraintBalljointEventPtr;
typedef SmartPointer<ToSimonConstraintHingejointEvent> ToSimonConstraintHingejointEventPtr;
typedef SmartPointer<ToSimonConstraintThreadEvent> ToSimonConstraintThreadEventPtr;

typedef SmartPointer<ToSimonClothEvent> ToSimonClothEventPtr;

typedef SmartPointer<ContactInformationContainer> ContactInformationContainerPtr;

typedef SmartPointer<Interference> InterferencePtr;

#endif // !SMART_POINTER_H
