
#ifndef SCENE_DESCRIPTION_H
#define SCENE_DESCRIPTION_H

#include <simon/simon.h>
#include "SphereDescription.h"
#include "BoxDescription.h"
#include "PlaneDescription.h"
#include "CapsuleDescription.h"

/**
 * Saves information about the scene which will be shown in VisualSimon.
 *
 * \author trappe
 *
 */
class SceneDescription {
	
public: 

	SceneDescription();

	//! enables/disables collision tests
	void testCollisions(bool);

	//! tell's weather to test for collisions or not
	bool testCollisions();

	//! saves how many integration steps should be done per frame
	void setIntegrationsPerFrame(int);
	//! tells how many integration steps should be done per frame
	int getIntegrationsPerFrame();

	//! saves the framerate which should be used
	void setFramerate(int);
	//! tells the framerate which should be used
	int getFramerate();

	//! saves the camera pos
	void setCameraPosition(Vec3);
	//! tells the cam position
	Vec3 getCameraPosition();

	//! saves center of intrest
	void setCenterOfInterest(Vec3);
	//! tells the coi
	Vec3 getCenterOfInterest();

	//! stores all objects in the scene
	vector<ObjectDescription*> objectVector;

private:
	
	//! collision test flag
	bool mTestCollisions;

	int mIntegrationsPerFrame;
	int mFramerate;

	//! camera distance from worldspace (0,0,0)
	Vec3 mCamPosition;

	//! center of interest
	Vec3 mCenterOfInterest;
};

#endif // SCENE_DESCRIPTION_H
