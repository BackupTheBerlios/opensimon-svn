
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

	//! stores all objects in the scene
	vector<ObjectDescription*> objectVector;

private:
	
	//! collision test flag
	bool mTestCollisions;

	int mIntegrationsPerFrame;
	int mFramerate;
};

#endif // SCENE_DESCRIPTION_H
