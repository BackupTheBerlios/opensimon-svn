#include "CapsuleDescription.h"

CapsuleDescription::CapsuleDescription(){

}

void CapsuleDescription::draw(GraphicsBase* context){
 	context->drawCapsule(getRigidBody()->getPosition(),
						boost::static_pointer_cast<Capsule>(getGeometry())->getRadius(),
						boost::static_pointer_cast<Capsule>(getGeometry())->getHeight(),
						 getRigidBody()->getOrientation(), 
 					   GraphicsBase::materialStone);
}
