#include "SphereDescription.h"

SphereDescription::SphereDescription(){

}

void SphereDescription::draw(GraphicsBase* context){
 	context->drawSphere(getRigidBody()->getPosition(), 
						boost::static_pointer_cast<Sphere>(getGeometry())->getRadius(),
						getRigidBody()->getOrientation(), 
 					   GraphicsBase::materialStone);
}
