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

	mCamPosition = Vec3(1000, 1000, 0);
	mCenterOfInterest = Vec3(0,0,0);
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


void SceneDescription::setCameraPosition(Vec3 pos){
	mCamPosition = pos;
}

Vec3 SceneDescription::getCameraPosition(){
	return mCamPosition;
}

void SceneDescription::setCenterOfInterest(Vec3 coi){
	mCenterOfInterest = coi;
}

Vec3 SceneDescription::getCenterOfInterest(){
	return mCenterOfInterest;
}
