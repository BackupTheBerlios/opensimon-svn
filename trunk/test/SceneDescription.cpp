/**
 * \class SceneDescripton
 */

#include "SceneDescription.h"
#include <iostream>

using namespace std;

/**
 *
 * Setting:
 *  - mTestCollisions = true
 */
SceneDescription::SceneDescription(){

	mTestCollisions = true;
	mFramerate = 30;
	mIntegrationsPerFrame = 2;
}

void SceneDescription::testCollisions(bool flag){
	mTestCollisions = flag;
}

bool SceneDescription::testCollisions(){
	return mTestCollisions;
}


void SceneDescription::setIntegrationsPerFrame(int x){
	mIntegrationsPerFrame = x;
}

int SceneDescription::getIntegrationsPerFrame(){
	return mIntegrationsPerFrame;
}

void SceneDescription::setFramerate(int x){
	mFramerate = x;
}

int SceneDescription::getFramerate(){
	return mFramerate;
}
