#ifndef SWIFT_COLLISION_DETECTION_H
#define SWIFT_COLLISION_DETECTION_H

#include <simon/CollisionDetection.h>

#include <vector>
#include <map>

#include <SWIFT.h>

/**
 * \class SwiftCollisionDetection
 *
 * \brief Collision detection with SWIFT.
 *
 */
class SwiftCollisionDetection : public CollisionDetection{

public:
	
	//! default constructor
	SwiftCollisionDetection();

	//! adds geometry
	virtual void add(GeometryPtr);

	//! find the interferences of all objects
	virtual void findInterferences(std::vector<InterferencePtr>&);

	//! find the interferences of a single object
	virtual void findInterferences(GeometryPtr, 
								   std::vector<InterferencePtr>&);

private:

	//! SWIFT scene
	SWIFT_Scene* mScene;

	//! ids for SWIFT objects
	int mHighestSwiftId;
	
	//! stores the mapping from SWIFT Id's to Simon geometries
	std::map<unsigned int, GeometryPtr> mIdConvertMap;

	//! stores all collision geometry of simon
	std::vector<GeometryPtr> mGeometry;

	//! creates and stores a new box into the SWIFT scene
	void create(const BoxPtr);
	//! creates and stores a new plane into the SWIFT scene
	void create(const PlanePtr);

	//! tells SWIFT about the new positions
	void updatePositions();
};

#endif  // !SWIFT_COLLISION_DETECTION_H
