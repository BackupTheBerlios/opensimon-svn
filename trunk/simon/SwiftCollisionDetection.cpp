
/**
 * \class SwiftCollisionDetection
 */


#include <simon/SwiftCollisionDetection.h>
#include <simon/Id.h>
#include <simon/Box.h>
#include <simon/Capsule.h>
#include <simon/Sphere.h>
#include <simon/Plane.h>

using namespace std;

/**
 * All objects will be stored in a swift scene with local 
 * bounding box sorting.
 */
SwiftCollisionDetection::SwiftCollisionDetection() :
	mScene(new SWIFT_Scene(true, false)){

	mHighestSwiftId = -1;
}

void SwiftCollisionDetection::add(GeometryPtr geometry){
	assert(geometry);

	if (geometry->getId().getType() == Id::typeBox){
		BoxPtr box = boost::static_pointer_cast<Box>(geometry);
		create(box);
	}else if (geometry->getId().getType() == Id::typePlane){
		PlanePtr plane = boost::static_pointer_cast<Plane>(geometry);
 		create(plane);
	}
}

/**
 * \param scale scaling of the box
 */
void SwiftCollisionDetection::create(const BoxPtr box){


	mHighestSwiftId++;

	mIdConvertMap.insert(
		pair<unsigned int, GeometryPtr>(mHighestSwiftId, box));
	
	Vec3 scale = box->getScale();
	
	SWIFT_Real cubeVertices[8*3] = {-1, -1, -1,
  									1, -1, -1,
									1,  1, -1,
									-1,  1, -1,
									-1, -1,  1,
									1, -1,  1,
									1,  1,  1,
									-1,  1,  1};

	// do the scaleing
	for (int i = 0; i < 8*3; i = i+3){
 		cubeVertices[i] *= scale[X];
 		cubeVertices[i+1] *= scale[Y];
 		cubeVertices[i+2] *= scale[Z];
   	}

	int cubeFaces[6*4] = {0, 4, 7, 3,
						  1, 2, 6, 5,
						  2, 3, 7, 6,
						  0, 1, 5, 4,
						  4, 5, 6, 7,
						  3, 2, 1, 0};

	int cubeFaceValences[6] = {4,4,4,4,4,4};

	if( !mScene->Add_Object(cubeVertices, cubeFaces, 8, 6, mHighestSwiftId, false,
							DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, 
							1, DEFAULT_BOX_SETTING, 
							DEFAULT_BOX_ENLARGE_REL, DEFAULT_BOX_ENLARGE_ABS,
							cubeFaceValences)) {
		cerr << "Adding object failed -- Exiting..." << endl;
	}
}

void SwiftCollisionDetection::create(const PlanePtr plane){
	
}

/**
 * \param interferenceVector The storage place of all
 * interferences. Objects will just be appended.
 */
void SwiftCollisionDetection::findInterferences(
	vector<InterferencePtr> &interferenceVector){

	updatePositions();

	int np;
	int* oids;
	SWIFT_Real* distances;
	SWIFT_Real* nearestPoints;
	SWIFT_Real* normals;
	int* pids;
	int* featureTypes;
	int* featureIDs;

	mScene->Query_Contact_Determination(false, SWIFT_INFINITY, np, &oids,
										&distances, &nearestPoints, &normals,
										&pids, &featureTypes, &featureIDs);

	for(int i = 0; i < np; i++ ) {

		// only if the bodies are colliding
		if (distances[i] != -1)
			continue;

		// only if both geometries are valid
		if (mIdConvertMap.count(oids[2*i]) == 0 || mIdConvertMap.count(oids[2*i+1]) == 0){
			continue;
		}

		InterferencePtr interference 
			= InterferencePtr(new Interference(mIdConvertMap[oids[2*i]], 
											   mIdConvertMap[oids[(2*i)+1]]));

		Vec3 location = 
			Vec3(nearestPoints[6*i],
				 nearestPoints[6*i+1],
				 nearestPoints[6*i+2]) +
			mIdConvertMap[oids[2*i]]->getPosition();

		Vec3 normal = Vec3(normals[3*i],
						   normals[3*i+1],
						   normals[3*i+2]);


// 		cout << "-----" << endl;
// 		cout << "    Object " << oids[2*i] << " vs. Object "
// 			 << oids[(2*i)+1] << " = (" 
// 			 << nearestPoints[6*i] << ","
// 			 << nearestPoints[6*i+1] << ","
// 			 << nearestPoints[6*i+2] << "):(" 
// 			 << Vec3(nearestPoints[6*i],
// 					 nearestPoints[6*i+1],
// 					 nearestPoints[6*i+2]) + geoBox1->getPosition()
// 			 << ") & ("
// 			 << nearestPoints[6*i+3] << ","
// 			 << nearestPoints[6*i+4] << ","
// 			 << nearestPoints[6*i+5] << "):("
// 			 << Vec3(nearestPoints[6*i+3],
// 					 nearestPoints[6*i+4],
// 					 nearestPoints[6*i+5]) + geoBox2->getPosition()
// 			 << ") with normal: ("
// 			 << normals[3*i] << ","
// 			 << normals[3*i+1] << ","
// 			 << normals[3*i+2] << ")"
// 			 << endl;

// 		cout << "    Feature type: " << featureTypes[2*i] << " & " << featureTypes[2*i+1];
// 		cout << endl;

		// handle edge edge
		if (featureTypes[2*i] == 2 && featureTypes[2*i+1] == 2){
			cout << "    Feature ids: " << featureIDs[2*i] << " & " << featureIDs[2*i+1];
			cout << endl;
		}

		// contactPoint, normal, distance
		interference->addContactPoint(location, normal);

		interferenceVector.push_back(interference);

	}
}


/**
 * \param geometry The geometry which should be checked \param
 * interferenceVector The storage place of all interferences. Objects
 * will just be appended
 */
	void SwiftCollisionDetection::findInterferences(
		GeometryPtr geometry, 
		vector<InterferencePtr>& interferenceVector){

	}


void SwiftCollisionDetection::updatePositions(){
		
	Matrix3x3 rotation;
	Vec3 translation;
	RigidBodyPtr rigidBody;

	map<unsigned int, GeometryPtr>::iterator end = mIdConvertMap.end();
	for (map<unsigned int, GeometryPtr>::iterator it = mIdConvertMap.begin(); 
		 it != end; 
		 ++it){
		
		//cout << mIdConvertMap.count(it->first) << endl;

		// if the geometry changes we need to add a new swift object
		// and delete the old one
		if (it->second->hasChanged()){
			mIdConvertMap.erase(it->first);
			create(boost::static_pointer_cast<Box>(it->second));
			it->second->hasChanged(false);
			return;
		}

		rigidBody = it->second->getRigidBody();
		rotation = rigidBody->getOrientation().getRotationMatrix();
		translation = rigidBody->getPosition();

		SWIFT_Real swiftRotation[9] = {rotation [0][0],
									   rotation [0][1],
									   rotation [0][2],
									   rotation [1][0],
									   rotation [1][1],
									   rotation [1][2],
									   rotation [2][0],
									   rotation [2][1],
									   rotation [2][2]};

		SWIFT_Real swiftTranslation[3] = {translation [0],
										  translation [1],
										  translation [2]};

		mScene->Set_Object_Transformation (it->first, 
										   swiftRotation, 
										   swiftTranslation);

	}
}
