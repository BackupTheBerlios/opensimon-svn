/**
 * Simon is the legal property of its developers, whose names are too
 * numerous to list here.  Please refer to the COPYRIGHT file
 * distributed with this source distribution.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */



#include <SWIFT.h>

#include "test-environment.h"
#include <simon/Graphics.h>
#include <simon/GeometrySystem.h>
#include <simon/RigidBodySystem.h>
#include <simon/Interference.h>

// SWIFT scene
SWIFT_Scene* scene;

// ids
int id1 = 0;
int id2 = 1;

RigidBodySystem rbSystem;
GeometrySystem geoSystem;
RigidBodyPtr rigidBodyBox1;
RigidBodyPtr rigidBodyBox2;

GeometryPtr geoBox1;
GeometryPtr geoBox2;

/**
 * \brief Loop-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
void displayLoop() {

	int intervall = 1;

	rbSystem.addGravity();

    // integrate rigid bodies
    rbSystem.integrate(intervall);

    // set new object transformations
    Matrix3x3 rotation1 = rigidBodyBox1->getOrientation
		().getRotationMatrix ();
    Matrix3x3 rotation2 = rigidBodyBox2->getOrientation
		().getRotationMatrix ();
    Vec3 translation1 = rigidBodyBox1->getPosition ();
    Vec3 translation2 = rigidBodyBox2->getPosition ();

    SWIFT_Real swiftRotation1 [9] = {rotation1 [0][0],
                                     rotation1 [0][1],
                                     rotation1 [0][2],
                                     rotation1 [1][0],
                                     rotation1 [1][1],
                                     rotation1 [1][2],
                                     rotation1 [2][0],
                                     rotation1 [2][1],
                                     rotation1 [2][2]};
 
    SWIFT_Real swiftRotation2 [9] = {rotation2 [0][0],
                                     rotation2 [0][1],
                                     rotation2 [0][2],
                                     rotation2 [1][0],
                                     rotation2 [1][1],
                                     rotation2 [1][2],
                                     rotation2 [2][0],
                                     rotation2 [2][1],
                                     rotation2 [2][2]};
 

    SWIFT_Real swiftTranslation1 [3] = {translation1 [0],
                                        translation1 [1],
                                        translation1 [2]};
 
    SWIFT_Real swiftTranslation2 [3] = {translation2 [0],
                                        translation2 [1],
                                        translation2 [2]};

    scene->Set_Object_Transformation (0, swiftRotation1, swiftTranslation1);
    scene->Set_Object_Transformation (1, swiftRotation2, swiftTranslation2);

	// query contact information

	int np;
	int* oids;
	SWIFT_Real* distances;
	SWIFT_Real* nearestPoints;
	SWIFT_Real* normals;
	int* pids;
	int* featureTypes;
	int* featureIDs;
	//scene->Query_Intersection( false, np, &oids);
	//scene->Query_Exact_Distance( false, SWIFT_INFINITY, np, &oids, &dists );
	scene->Query_Contact_Determination(false, SWIFT_INFINITY, np, &oids,
 									   &distances, &nearestPoints, &normals,
									   &pids, &featureTypes, &featureIDs);

	for(int i = 0; i < np; i++ ) {
		Interference interference(geoBox1, geoBox2);

		Vec3 location = Vec3(nearestPoints[6*i],
							 nearestPoints[6*i+1],
							 nearestPoints[6*i+2])+geoBox1->getPosition();
		Vec3 normal = Vec3(normals[3*i],
						   normals[3*i+1],
						   normals[3*i+2]);

		// only if the bodies are colliding
		if (distances[i] != -1)
			break;

		cout << "-----" << endl;
		cout << "    Object " << oids[2*i] << " vs. Object "
			 << oids[(2*i)+1] << " = (" 
			 << nearestPoints[6*i] << ","
			 << nearestPoints[6*i+1] << ","
			 << nearestPoints[6*i+2] << "):(" 
			 << Vec3(nearestPoints[6*i],
					 nearestPoints[6*i+1],
					 nearestPoints[6*i+2]) + geoBox1->getPosition()
			 << ") & ("
			 << nearestPoints[6*i+3] << ","
			 << nearestPoints[6*i+4] << ","
			 << nearestPoints[6*i+5] << "):("
			 << Vec3(nearestPoints[6*i+3],
					 nearestPoints[6*i+4],
					 nearestPoints[6*i+5]) + geoBox2->getPosition()
			 << ") with normal: ("
			 << normals[3*i] << ","
			 << normals[3*i+1] << ","
			 << normals[3*i+2] << ")"
			 << endl;

		cout << "    Feature type: " << featureTypes[2*i] << " & " << featureTypes[2*i+1];
		cout << endl;


		// contactPoint, normal, distance
		interference.addContactPoint(location, normal);

		geoSystem.collisionResponse(interference, true);

		
		Graphics::drawSphere(location, 5, Graphics::red);

	}

	//geoSystem.doContacts(intervall);


    // draw objects
	geoSystem.drawGeometries();

   
}


/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung 
 * aufgerufen. Hier sollte alles reingeschrieben werden, 
 * was für die initialisierung der einzelnen Tests nötig ist.
 */
	void initialize(int /*argc*/, char** /*argv*/) {

		// Create a swift scene with local bounding box sorting
		scene = new SWIFT_Scene( true, false );

	

		// A moving cube

		SWIFT_Real cubeVertices[8*3] = {-1, -1, -1,
										1, -1, -1,
										1, 1, -1,
										-1, 1, -1,
										-1, -1, 1,
										1, -1, 1,
										1, 1, 1,
										-1, 1, 1};
		int cubeFaces[6*4] = {0, 4, 7, 3,
							  1, 2, 6, 5,
							  2, 3, 7, 6,
							  0, 1, 5, 4,
							  4, 5, 6, 7,
							  3, 2, 1, 0};

		int cubeFaceValences[6] = {4,4,4,4,4,4};


		if( !scene->Add_Object( cubeVertices, cubeFaces, 8, 6, id1, false,
								DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, 
								30, DEFAULT_BOX_SETTING, 
								DEFAULT_BOX_ENLARGE_REL, DEFAULT_BOX_ENLARGE_ABS,
								cubeFaceValences)) {
			cerr << "Adding object1 failed -- Exiting..." << endl;
			exit( -1 );
		} else {
			cerr << "Added object1 to scene" << endl;
		}


		// an other one

		//SWIFT_Real orientation[9] = {
		SWIFT_Real translation [3] = {0,120,0};

		if( !scene->Add_Object( cubeVertices, cubeFaces, 8, 6, id2, true,
								DEFAULT_ORIENTATION, DEFAULT_TRANSLATION, 
								30, DEFAULT_BOX_SETTING, 
								DEFAULT_BOX_ENLARGE_REL, DEFAULT_BOX_ENLARGE_ABS,
								cubeFaceValences)) {
			cerr << "Adding object1 failed -- Exiting..." << endl;
			exit( -1 );
		} else {
			cerr << "Added object1 to scene" << endl;
			scene->Set_Object_Transformation (0, DEFAULT_ORIENTATION, translation);
		}

		rigidBodyBox1 = RigidBodyPtr (rbSystem.create(Id(Id::typeBox,0)));
		rigidBodyBox1->setIsDynamicFlag(false);
		rigidBodyBox2 = RigidBodyPtr (rbSystem.create(Id(Id::typeBox,1)));
		rigidBodyBox2->setPosition (Vec3 (0.0, 120.0, 0.0));
//		rigidBodyBox2->setOrientation (Quaternion (M_PI/2, Vec3(1,1,0)));
		geoBox1 = geoSystem.createBox (rigidBodyBox1, Vec3 (30,30,30));
		geoBox2 = geoSystem.createBox (rigidBodyBox2, Vec3 (30,30,30));
	

	}

// Diese Methode verarbeitet keybord events
	void keyHandler(unsigned char /*key*/){

	}
