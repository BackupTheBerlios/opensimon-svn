#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <simon/Interference.h>
#include <simon/Geometry.h>

#include <vector>

/**
 * \class CollisionDetection
 *
 * \brief Basic implementation for collision detection
 *
 * You can implement an own technique of collision detection by
 * deriving form this class.
 */
class CollisionDetection{

public:
	
	//! adds geometry
	virtual void add(GeometryPtr) = 0;

	//! find the interferences of all objects
	virtual void findInterferences(std::vector<InterferencePtr>&) = 0;

	//! find the interferences of a single object
	virtual void findInterferences(GeometryPtr, 
								   std::vector<InterferencePtr>&) = 0;

};

#endif  // !COLLISION_DETECTION_H
