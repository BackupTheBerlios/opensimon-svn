
#include "ObjectDescription.h"

ObjectDescription::ObjectDescription():
	mSteadyFoce(0,0,0) {

}

void ObjectDescription::addSteadyForce(Vec3 force){
	mSteadyFoce += force;
}

void ObjectDescription::clearSteadyForce(){
	mSteadyFoce = Vec3(0,0,0);
}

Vec3 ObjectDescription::getSteadyForce(){
	return mSteadyFoce;
}

void ObjectDescription::setGeometry(GeometryPtr geometry){
	mGeometry = geometry;
}

GeometryPtr ObjectDescription::getGeometry(){
	return mGeometry;
}

RigidBodyPtr ObjectDescription::getRigidBody(){
	return mGeometry->getRigidBody();
}


void ObjectDescription::draw(GraphicsBase* context){
	// draw nothing
}
