#ifndef OBJECT_DESCRIPTION_H
#define OBJECT_DESCRIPTION_H

#include <simon/simon.h>
#include "GraphicsBase.h"

class ObjectDescription{

public:

	ObjectDescription();
	virtual ~ObjectDescription(){};

 	virtual void addSteadyForce(Vec3);
 	virtual void clearSteadyForce(); 
	Vec3 getSteadyForce();

	virtual void setGeometry(GeometryPtr);
	virtual GeometryPtr getGeometry();

	virtual RigidBodyPtr getRigidBody();

	virtual void draw(GraphicsBase*) = 0;

private:
	
	Vec3 mSteadyFoce;
	GeometryPtr mGeometry;
};

#endif  // OBJECT_DESCRIPTION_H
