#include "PlaneDescription.h"

#include <iostream>

using namespace std;

PlaneDescription::PlaneDescription(){

}

void PlaneDescription::draw(GraphicsBase* context){
 	context->drawPlane(getRigidBody()->getPosition(),
					   getRigidBody()->getOrientation(),
 					   GraphicsBase::materialStone);
}
