#include "BoxDescription.h"

BoxDescription::BoxDescription(){

}

void BoxDescription::draw(GraphicsBase* context){
 	context->drawBox(getRigidBody()->getPosition(),
					 boost::static_pointer_cast<Box>(getGeometry())->getScale(),
					 getRigidBody()->getOrientation(), 
					 GraphicsBase::materialStone);
}
