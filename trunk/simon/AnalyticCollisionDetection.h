#ifndef ANALYTIC_COLLISION_DETECTION_H
#define ANALYTIC_COLLISION_DETECTION_H

#include <simon/Interference.h>
#include <simon/Geometry.h>
#include <simon/Box.h>
#include <simon/Capsule.h>
#include <simon/Sphere.h>
#include <simon/Plane.h>

#include <simon/CollisionDetection.h>

/**
 * \class AnalyticCollisionDetection
 *
 * \brief implementation of anaytical collision detection.
 *
 * This class provides an analytical approach for collision detection.
 *
 * \todo the Box-Box collision needs a bit love ;)
 */
class AnalyticCollisionDetection : public CollisionDetection{

public:

	//! adds geometry
	virtual void add(GeometryPtr);

	//! finds the interferences of all objects
	virtual void findInterferences(std::vector<InterferencePtr>&);

	//! find the interferences of a single object
	virtual void findInterferences(GeometryPtr, 
								   std::vector<InterferencePtr>&);

private:
	
	bool check(GeometryPtr, GeometryPtr, Interference&);

	bool check(SpherePtr, SpherePtr, Interference&);
	bool check(SpherePtr, PlanePtr, Interference&);

	bool check(BoxPtr, BoxPtr, Interference&);
	bool check(BoxPtr, PlanePtr, Interference&);
	bool check(BoxPtr, SpherePtr, Interference&);


	bool check(CapsulePtr, CapsulePtr, Interference&);
	bool check(CapsulePtr, PlanePtr, Interference&);
	bool check(CapsulePtr, SpherePtr, Interference&);
	bool check(CapsulePtr, BoxPtr, Interference&);

	std::vector< GeometryPtr > mGeometry;

	bool collideSphereBox(Vec3, float, Vec3&, Vec3&, float, BoxPtr);

};

#endif  // !ANALYTIC_COLLISION_DETECTION_H
