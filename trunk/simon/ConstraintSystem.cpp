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


/**
 * \class ConstraintSystem
 * \author Palmer, Hornung
 *
 * \brief Klasse, die das ganze Constraint System kontrolliert
 *
 * In dieser Klasse laufen alle Teile der Constraints zusammen. 
 * Sie stellt auf die Schnittstelle zum restlichen Simon-Programm 
 * dar.
 *
 */

// ////////////////////////////////////////////////////////////////////////////

/**
\brief Includes
*/
#include <simon/ConstraintSystem.h>
#include <iostream>
#include <queue>
#include <simon/Clock.h>

using namespace std;


/**
\brief  Standard-Konstruktor
*/
ConstraintSystem::ConstraintSystem() {
	mNumberOfAlgorithm = 4;
}

/**
\brief  Standard-Destruktor
*/
ConstraintSystem::~ConstraintSystem() {
	//sad ... we had such a nice delete code ... now with smart pointers, we do not need it any more
	
	/*std::map<Id, ConstraintMatrixNode*>::iterator rootsIterator;
	std::list<ConstraintMatrixNode*>::iterator listItr;
	std::map<Id, PrimaryConstraint*>::iterator constraintItr;
	ConstraintMatrixNode* root;
	
	//delete all constraint matrix nodes
	for (rootsIterator = mRoots.begin (); rootsIterator != mRoots.end (); rootsIterator++) {
		root = rootsIterator->second;
		//empty mList
		mList.clear();
		ordermatrix(root);
	
		//forward
		for (listItr = mList.begin(); listItr != mList.end(); listItr++) {
			if (*listItr) {
				delete *listItr;
			}
		}
	}
	
	//delete all constraints
	for (constraintItr = mConstraints.begin(); constraintItr != mConstraints.end(); constraintItr++) {
		if (constraintItr->second) {
			delete (constraintItr->second);
		}
	}*/
	
}

/*void ConstraintSystem::enableMaxForceLengthCheck() {
	std::map<Id, PrimaryConstraintPtr>::iterator pcItr;
	for (pcItr = mConstraints.begin(); pcItr != mConstraints.end();++pcItr) {
		pcItr->second->enableMaxForceLengthCheck();
	}
}

void ConstraintSystem::disableMaxForceLengthCheck() {
	std::map<Id, PrimaryConstraintPtr>::iterator pcItr;
	for (pcItr = mConstraints.begin(); pcItr != mConstraints.end();++pcItr) {
		pcItr->second->disableMaxForceLengthCheck();
	}
}*/

void ConstraintSystem::setTau(float tau) {
	std::map<Id, PrimaryConstraintPtr>::iterator pcItr;
	for (pcItr = mConstraints.begin(); pcItr != mConstraints.end();++pcItr) {
		pcItr->second->setTau(tau);
	}
}


/**
\brief Eine neue BallAndSocket-Verbindung anlegen
\param Ident Id-Kennzeichnung des neuen Constraints
\param objectA, erster zu verbindender Körper
\param objectB, zweiter zu verbindender Körper
\param positionA, Punkt des 1. Körpers, der mit der Position des 2. Körpers übereinstimmen soll (in Objektkoordinaten)
\param positionB, Punkt des 2. Körpers, der mit der Position des 1. Körpers übereinstimmen soll (in Objektkoordinaten)

Diese Funktion legt ein neues Objekt vom Typ BallAndSocketConstraint an, speichert einen Pointer darauf in mConstraints.
*/
void ConstraintSystem::createBallAndSocketConstraint(Id ident, 
													 RigidBodyPtr objectA, 
													 RigidBodyPtr objectB,
													 Vec3 positionA, 
													 Vec3 positionB) {

	// transform vectors into matrixes
	Matrix3x1 positionAMatrix (positionA);
	Matrix3x1 positionBMatrix (positionB);

	//create and configure new constraint object
	BallAndSocketConstraintPtr constraint(new BallAndSocketConstraint(objectA, objectB, positionAMatrix, positionBMatrix));
	constraint->setId(ident);
	//remember pointer (twice)
	mConstraints[ident] = constraint;
	mBallAndSocketConstraints[ident] = constraint;
}

/**
\brief Eine neue HingeConstraint-Verbindung anlegen
\param Ident Id-Kennzeichnung des neuen Constraints
\param objectA Erster zu verbindender Körper.
\param objectB Zweiter zu verbindender Körper.
\param positionA Punkt des 1. Körpers, der mit der Position des 2. Körpers übereinstimmen soll (in Objektkoordinaten).
\param positionB Punkt des 2. Körpers, der mit der Position des 1. Körpers übereinstimmen soll (in Objektkoordinaten).
\param positionN1 Punkt in Objektkoordinaten des 1. Körpers, der in der Gelenkebene liegt.
\param positionN2 Punkt in Objektkoordinaten des 2. Körpers, der in der Gelenkebene liegt.
\param specifyNormal Der jeweils zweite Punkt eines Körpers gibt die Normale der Gelenkebene anstelle eines der Punkte der Ebene.

Diese Funktion legt ein neues Objekt vom Typ HingeConstraint an, speichert einen Pointer darauf in mConstraints.
*/
void ConstraintSystem::createHingeConstraint(Id ident, 
                                             RigidBodyPtr objectA, 
                                             RigidBodyPtr objectB,
                                             Vec3 positionA, 
                                             Vec3 positionB,
                                             Vec3 positionN1,
                                             Vec3 positionN2,
                                             bool specifyNormal) {

	// transform vectors into matrixes
	Matrix3x1 positionAMatrix (positionA);
	Matrix3x1 positionBMatrix (positionB);
	Matrix3x1 positionN1Matrix (positionN1);
	Matrix3x1 positionN2Matrix (positionN2);

	//create and configure new constraint object
	HingeConstraintPtr constraint(new HingeConstraint(objectA, objectB, positionAMatrix, positionBMatrix, positionN1Matrix, positionN2Matrix, specifyNormal));
	constraint->setId(ident);
	//remember pointer (twice)
	mConstraints[ident] = constraint;
	mHingeConstraints[ident] = constraint;
}


/**
\brief Smart-Pointer auf BallAndSocketConstraint zurückliefern
\param Ident Id-Kennzeichnung des Constraints, auf den man zugreifen will
*/
BallAndSocketConstraintPtr ConstraintSystem::getBallAndSocketConstraint(Id Ident) {
	return mBallAndSocketConstraints[Ident];
}

/**
\brief Smart-Pointer auf HingeConstraint zurückliefern
\param Ident Id-Kennzeichnung des Constraints, auf den man zugreifen will
*/
HingeConstraintPtr ConstraintSystem::getHingeConstraint(Id Ident) {
	return mHingeConstraints[Ident];
}

/**
\brief Ordnet alle Constraints und betreffende Festkörper in einem/mehreren Graphien

\todo Später diese Methode nur noch private
\todo Grundprämisse warum erklären !

Die Constraints und betreffende Festkörper werden geordnet in einen/mehrere Graphen aufgenommen. 
Grundprämissen: 
a)
Zwischen einem Constraint und den 2 Körpern soll immer eine Vater-Kind-Beziehung der Form
Körper=>Constraint=>Körper gelten.
b)
Zwischen zwei Körpern gibt es nur ein Constraint
 
Hierbei werden folgende Schritte unternommen:

1.
 Erzeuge für alle Constraints und die Festkörper, die sie verbinden, einen ConstraintMatrixNode. 
 Dieser Node bekommt dieselbe Id wie der Constraint/Festkörper.
 
 2. Aufbauen des Graphs / der Graphen
 Das Aufbauen der Graphen hat verschiedene Stufen. 
 Es wird versucht, alle zusammenhängenden Körper/Constraints in einen Graphen zu packen.
 Es soll keine Verbindungen zwischen verschiedenen Graphen geben.
 Nach einem Durchgang sollen alle Knoten, die zusammenhängen, in einen Graphen einsortiert sein. 
 Danach wird mit eventuell verbliebenen Knoten der Vorgang wiederholt.
 
 Schritte dieses Vorgangs: 
 2.1. Root-Knoten suchen
 Ein Root-Knoten soll nur ein Körper-Knoten sein, wegen der Grundprämisse.
 Der Root-Knoten wird sich in mLeafs gemerkt
 
2.2. Graph zu aktuellem Knoten aufbauen Solange in einem
Schleifendurchlauf neue Elemente hinzukommen, füge Knoten ein.  Es
wird zu den aktuellen Blättern, die immer Körperknoten sind,
Constraints gesucht, die den Körper mit einem anderen
verbinden. Dieses Constraint und der neue Körper werden in den Graph
eingefügt, der neue Körper sich in mLeafs gemerkt.
*/
void ConstraintSystem::buildGraphs() {

	if (mConstraints.empty())
		return;
	
	//Lokal benötigte Datenspeicher
	
	//Speicher zum Zwischenspeichern der Nodes/Nodeinformationen
	std::map<Id, ConstraintMatrixNode*> bodyNodes;//zwischenspeicher für alle Körper-Knoten
	std::map<Id, ConstraintMatrixNode*> constraintNodes;//zwischenspeicher für alle Constraint-Knoten
	std::queue<ConstraintMatrixNode*> leafs;//aktuelle Körper-Knoten, die Blätter sind
	std::queue<Id> toDelete;//Zwischenmerkspeicher
	
	//Iteratoren für den Zugriff
	std::map<Id, PrimaryConstraintPtr>::iterator pcItr;
	std::map<Id, ConstraintMatrixNode*>::iterator bodyItr;
	std::map<Id, ConstraintMatrixNode*>::iterator constraintItr;
	
	//pointer
	ConstraintMatrixNode* node;

	
	
	//Schritt 1 - Knoten zu den Constraints/Körpern
	
	//alte Knoten löschen 
	//SmartPointer löschen, Knoten sollten dann mit gelöscht werden
	mBodyNodes.clear();
	mConstraintNodes.clear();
	//Roots löschen (sind noch normale Pointer)
	mRoots.clear();
	
	//Schleife über alle Constraints
	std::map<Id, PrimaryConstraintPtr>::iterator endOfpcItr = mConstraints.end();
	for (pcItr = mConstraints.begin(); pcItr != endOfpcItr; ++pcItr) {
		//Knoten für den Constraint anlegen
		//Annahme: die Index-Id entspricht der Id des Constraints (so wird mConstraints eigentlich angelegt!)
		constraintNodes[pcItr->first] = new ConstraintMatrixNode(pcItr->second);
		//Node persistent speichern, um später löschen zu können
		mConstraintNodes[pcItr->first] = ConstraintMatrixNodePtr(constraintNodes[pcItr->first]);
		//Knoten für die verbundenen Körper erzeugen, falls nicht schon vorhanden
		Id objectAId = pcItr->second->getObjectA()->getId();
		if (bodyNodes.count(objectAId) == 0) {
			bodyNodes[objectAId] = new ConstraintMatrixNode(boost::static_pointer_cast<RigidBody>(pcItr->second->getObjectA())); 
			mBodyNodes[objectAId] = ConstraintMatrixNodePtr(bodyNodes[objectAId]);
		}
		Id objectBId = pcItr->second->getObjectB()->getId();
		if (bodyNodes.count(objectBId) == 0) {
			bodyNodes[objectBId] = new ConstraintMatrixNode(boost::static_pointer_cast<RigidBody>(pcItr->second->getObjectB())); 
			mBodyNodes[objectBId] = ConstraintMatrixNodePtr(bodyNodes[objectBId]);
		}
	}
	
	//Schritt 2.1. - Root-Knoten suchen 
	//nimm den ersten Körper-Knoten in bodyNodes als neue Wurzel
	//dies wird solange durchgeführt, bis keine Knoten mehr zu verteilen sind
	while (! bodyNodes.empty()) {
		bodyItr = bodyNodes.begin();
		
		/*while(nodeItr->second->isConstraint()) {//nicht mehr benötigt
			++nodeItr;
		}*/
		mRoots[bodyItr->first] = bodyItr->second;
		
		//assign node to mLeafs, delete it from bodyNodes
		leafs.push(bodyItr->second);
		bodyNodes.erase(bodyItr);
		
		//Schritt 2.2. - Baum zu aktuellem Knoten aufbauen
		//alle Knoten die indirekt mit der aktuellen Wurzel verbunden sind werden mitgenommen
		while (! leafs.empty()) {
			node = leafs.front();//forderster Körperknoten in der queue
			//füge alle Constraints hinzu, die aktuellen Körperknoten beinhalten
			std::map<Id, ConstraintMatrixNode*>::iterator endOfConstrintItr = constraintNodes.end();
			for(constraintItr = constraintNodes.begin(); constraintItr != endOfConstrintItr;++constraintItr) {
				Id objectAId = constraintItr->second->getConstraint()->getObjectA()->getId();
				Id objectBId = constraintItr->second->getConstraint()->getObjectB()->getId();
				if (objectAId == node->getBody()->getId()) {
					//füge (...node...)=>constraintNode=>ObjektBNode ein
					node->addChild(constraintItr->second);
					constraintItr->second->setParent(node);
					constraintItr->second->addChild(bodyNodes[objectBId]);
					bodyNodes[objectBId]->setParent(constraintItr->second);
					//markiere in ConstraintNode, das die Reihenfolge "nicht vertauscht" ist (B->Constraint->A)
					
					//constraintItr->second->setInverse(false);
					
					//füge neuen Körper der leaf-liste hinzu
					leafs.push(bodyNodes[objectBId]);
					//lösche Körper aus der entsprechenden Liste
					bodyNodes.erase(objectBId);
					//zwischenmerken, das der Constraint noch gelöcht werden muss
					toDelete.push(constraintItr->first);
				} else if (objectBId == node->getBody()->getId()) {
					//füge (... node ...)=>constraintNode=>ObjektANode ein
					node->addChild(constraintItr->second);
					constraintItr->second->setParent(node);
					constraintItr->second->addChild(bodyNodes[objectAId]);
					bodyNodes[objectAId]->setParent(constraintItr->second);
					//markiere in ConstraintNode, das die Reihenfolge "vertauscht" ist (B->Constraint->A)
					
					//constraintItr->second->setInverse(true);
					constraintItr->second->getConstraint()->swapBodies();
					
					//füge neuen Körper der leaf-liste hinzu
					leafs.push(bodyNodes[objectAId]);
					//lösche Körper aus der entsprechenden Liste
					bodyNodes.erase(objectAId);
					//zwischenmerken, das der Constraint noch gelöcht werden muss
					toDelete.push(constraintItr->first);
				} 
			}
			//löschen der Constraint-Knoten, die behandelt wurden
			while (! toDelete.empty()) {
				constraintNodes.erase(toDelete.front());
				toDelete.pop();
			}
			leafs.pop();//Körperknoten wurde behandelt, aus queue löschen
		}
	}
}

/**
\brief Print the Node graph(s) to console
*/
void ConstraintSystem::printGraphs() {
	std::map<Id, ConstraintMatrixNode*>::iterator rootItr;
	printf("Constraint Graphs:\n\n");
	int loopcount = 0;
	for(rootItr = mRoots.begin(); rootItr != mRoots.end();++rootItr) {
		printf("Graph: %i\n",loopcount);
		printGraph(rootItr->second, 0);
		printf("\n\n");
		loopcount++;
	}
}

/**
\brief Print the Node graph with given root to console
\param root, the root of the graph
*/
void ConstraintSystem::printGraph(ConstraintMatrixNode* node, int depth) {
	//std::map<Id, ConstraintMatrixNode*, compareIds>::iterator roottItr;
	for (int i = 0; i < depth; ++i) {
			printf("     ");
		}
	
	if (node->isConstraint()) {
		printf("->Constraint %i:%i",node->getConstraint()->getId().getType(),node->getConstraint()->getId().getNumber());
		if (node->isInverse()) {
			printf(" isInverse ");
		}
		printf("\n");
	} else {
		printf("->Body %i:%i\n",node->getBody()->getId().getType(),node->getBody()->getId().getNumber());	
	}
	
	if (node->numberOfChildren() > 0) {
		for (int i = 0; i < node->numberOfChildren(); ++i) {
			printGraph(node->getChild(i), depth+1);
		}
	}
}

/**
\brief Compute some values for all constraints in the system.

Computes Jacobian-, Constraint- and RightHandSide-Matrix for all constraints in the system.
*/
void ConstraintSystem::computeValues() {
	std::map<Id, ConstraintMatrixNode*>::iterator rootItr;
	//printf("Number of roots:%i\n",mRoots.size());
	
	std::map<Id, ConstraintMatrixNode*>::iterator endOfItr = mRoots.end();
	for(rootItr = mRoots.begin(); rootItr != endOfItr; ++rootItr) {
		computeNodeValues(rootItr->second);
	}
}

/**
\brief Compute some values for all constraint in the graph
\param root, the root of the graph

Computes Jacobian-, Constraint- and RightHandSide-Matrix if given node represents a constraint. All childs are recusively called.
*/
void ConstraintSystem::computeNodeValues(ConstraintMatrixNode* node) {
	if (node->isConstraint()) {
		node->getConstraint(mTempConstraintPtr);

		mTempConstraintPtr->setStepSize(0.1);
		mTempConstraintPtr->compute();
	} 
	
	if (node->numberOfChildren() > 0) {
		for (int i = 0; i < node->numberOfChildren(); ++i) {
			computeNodeValues(node->getChild(i));
		}
	}
}


/**
 * \brief After the computeSolution step, the results have to be
 * written back into the RigidBodies
*/
void ConstraintSystem::updateAllRigidBodies() {
	std::map<Id, PrimaryConstraintPtr>::iterator constraintItr;
	PrimaryConstraintPtr constraint;
	for(constraintItr = mConstraints.begin(); 
		constraintItr != mConstraints.end();
		++constraintItr) {
		constraint = constraintItr->second;
		constraint->updateRigidBodies();
	}
}

/**
\brief Set the used solve algorithm
\param numberOfAlgorithm Number of the implementation of computeSolution to use

There are multiple computeSolution implementations with different complexity. This function sets the algorithm to use.
*/
void ConstraintSystem::setComputationAlgorithm(int numberOfAlgorithm) {
	mNumberOfAlgorithm = numberOfAlgorithm;
}

/**
 * \brief Call all necessary functions for a complete computation step of the constraint system
 */
void ConstraintSystem::step() {
	Clock timer;
	if (mRoots.size() > 0) {//! \todo besser woanders lösen
		timer.start();
		computeValues();
		timer.stop();
//		cout << "computeValues : " << timer << endl;
		switch (mNumberOfAlgorithm) {
	  	case 1:
			computeSolution1();
			break;
		case 2:
			computeSolution2();
			break;
		case 3:
			computeSolution3();
			break;
		case 4:
			computeSolution4();
			break;
		default:
			computeSolution4();
		}
		timer.start();
		updateAllRigidBodies();
		timer.stop();
		//	cout << "update RB's : " << timer << endl;
	}//besser woanders lösen
}

/**
 * \brief Call all necessary functions for a complete computation step of the
 * constraint system (same as step, really only calls step () once)
 */
void ConstraintSystem::computeConstraints ()
{

  step ();

}

/**
 * \brief Compute and process post-stabilization
 *
 * This is an implementation of Cline and Pai
 * "Post-Stabilization for Rigid Body Simulation with Contact and
 * Constraints", Proc IEEE Int'l Conf. on Robotics and Automation 03,
 * section V. Contact ist not implemented. It is fitted in
 * Baraff's method of solving a system of multibody equality
 * constraints. So the post stabilization computations really only
 * draw upon the functions already present to solve the linear
 * system. The only difference is the right hand side. It is solved
 * for a \f$\Delta \mathbf{x}\f$, that is the deviation of the
 * position and orientation of each rigid body from its
 * supposed position suggested by the constraint. The orientation is
 * given as an Euler angle and appended to the linear position to form
 * a 6x1 vector (space notation). Please see the Quaternion class for
 * further information on the conventions for Euler angles used
 * in this project.
 *
 * \pre Please note that this function has to be called after
 * computing and processing constraints and after integrating
 * (linear and angular) velocities and positions / orientations.
 *
 */
void ConstraintSystem::computePostStabilization (bool fast)
{

  std::map<Id, ConstraintMatrixNode*>::iterator rootsIterator;
  std::map<Id, ConstraintMatrixNode*>::iterator rootsEnd;
  ConstraintMatrixNode* root;

  rootsEnd = mRoots.end ();

  bool forPostStabilization = true;

  //process all independant graphs seperately
  for (rootsIterator = mRoots.begin (); 
	   rootsIterator != rootsEnd; 
	   ++rootsIterator) {

    root = rootsIterator->second;

    // mList.clear();      // it is supposed that no new constraints
    // ordermatrix(root);  // have been added since the last "step ()"
    if (!fast) {
      computeValues ();
      factor();
    }
    solve (forPostStabilization);
  }

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//solution algorithms 1 - Baraffs O(n^3), dense factor


/**
\brief Solve the linear system, version 1.

This algorithm solves the linear system A*lambda=b (Baraff 141). It does so in using cholesky decomposition for inverting the positive definite matrix A. This implementation has compleyity O(n^3), and is rather a brute force approach.
*/
void ConstraintSystem::computeSolution1() {

	cout << "Achtung! Code auskommentiert in computeSolution1()" << endl;
	//! \todo reparieren und wieder einkommentieren
/*
//variables
std::map<Id, RigidBodyPtr> bodies;//all body nodes
std::map<Id, PrimaryConstraintPtr>::iterator pcItr;
std::map<Id, int> bodyPositions;
std::map<Id, PrimaryConstraintPtr>::iterator constraintItr;
std::map<Id, RigidBodyPtr>::iterator bodyItr;
	
float mass;
	
Matrix<float> jacobian1;
Matrix<float> jacobian2;
Matrix<float> rightHandSide1;
Matrix<float> rightHandSide2;
Matrix<float> testLambda(3,1);
Matrix<float> invWorldInertiaTensor;
Matrix<float> bigInvA;
Matrix<float> bigLambda;
Matrix<float> addedRightHandSide;
	
	
PrimaryConstraintPtr constraint;
	
//build a list of all bodys in the system
for (pcItr = mConstraints.begin(); pcItr != mConstraints.end();++pcItr) {
//Knoten für die verbundenen Körper erzeugen, falls nicht schon vorhanden
Id objectAId = pcItr->second->getObjectA()->getId();
if (bodies.count(objectAId) == 0) {
bodies[objectAId] = boost::static_pointer_cast<RigidBody>(pcItr->second->getObjectA()); 
}
Id objectBId = pcItr->second->getObjectB()->getId();
if (bodies.count(objectBId) == 0) {
bodies[objectBId] = boost::static_pointer_cast<RigidBody>(pcItr->second->getObjectB()); 
}
}
	
//define these matrixes here, because the need data computed above
Matrix<float> bigJacobian(3*mConstraints.size(),6*bodies.size());
Matrix<float> bigInvMass(6*bodies.size(),6*bodies.size());
Matrix<float> bigA(3*mConstraints.size(),3*mConstraints.size());
Matrix<float> bigRightHandSide(3*mConstraints.size(),1);
	
//build big mass matrix M^-1 which contains all mass values
//see Baraff 140 for M's structure
//an entry for a rigid body is 6x6
//the first 3 diagonal elements contain 1/mass
//the lower right 3x3 rectangle contains the inverse WorldInertia tensor
//all other values are zero (matrixes are initialized with zero)
int bodyCounter = 0;
for(bodyItr = bodies.begin(); bodyItr != bodies.end();++bodyItr) {
//get required data of body
mass = bodyItr->second->getMass();
invWorldInertiaTensor = bodyItr->second->getInvWorldInertiaTensor();
		
//fill mass matrix with body values
for (int i = 0; i < 3; ++i) {
bigInvMass[i+(bodyCounter*6)][i+(bodyCounter*6)] = 1/mass;
for (int j = 0; j < 3; ++j) {
bigInvMass[i+(bodyCounter*6)+3][j+(bodyCounter*6)+3] = invWorldInertiaTensor[i][j];
}
}
		
//there is a connection between the position of a body in the mass matrix
//and the position of the element in the big jacobian matrix
//thus:
//remember position of bodies
bodyPositions[bodyItr->second->getId()] = bodyCounter;
		
bodyCounter++;
}
	
//build matrix big jacobian J
//see Bartaff 140 for J's structure
//all small Jacobians for a constraint are stored in one of big J's rows
int constraintCounter = 0;
for(constraintItr = mConstraints.begin(); constraintItr != mConstraints.end();++constraintItr) {
//get required data out of the constraint matrix nodes
//these values have been computed in computeValues();
constraint = constraintItr->second;
int numberBody1 = bodyPositions[constraint->getObjectA()->getId()];
int numberBody2 = bodyPositions[constraint->getObjectB()->getId()];
jacobian1 = constraint->getJacobian1();
jacobian2 = constraint->getJacobian2();
rightHandSide1 = constraint->getRightHandSide1();
rightHandSide2 = constraint->getRightHandSide2();
		
//fill big J
for (int i = 0; i < 3; ++i) {
for (int j = 0; j < 6; ++j) {
bigJacobian[i+(constraintCounter*3)][j+(numberBody1*6)] = jacobian1[i][j];
bigJacobian[i+(constraintCounter*3)][j+(numberBody2*6)] = jacobian2[i][j];
}
}
		
//compute big Right Hand side
//the right hand side is the b-Value in A*lambda=b (Baraff 141)
//the right hand side has been precomputed in computeValues and is only "put together" here
addedRightHandSide = rightHandSide1 + rightHandSide2;
		
bigRightHandSide[constraintCounter*3][0] = addedRightHandSide[0][0];
bigRightHandSide[(constraintCounter*3)+1][0] = addedRightHandSide[1][0];
bigRightHandSide[(constraintCounter*3)+2][0] = addedRightHandSide[2][0];
		
constraintCounter++;
		
}
	
//compute bigA and solve the linear system (Baraff 141)
bigA = bigJacobian * bigInvMass * bigJacobian.T();
bigInvA = choleskyInverse(bigA);
bigLambda = bigInvA * bigRightHandSide;
	
	
//get the according lambda value for each constraint
//write this value back into the constraint node
constraintCounter = 0;
for(constraintItr = mConstraints.begin(); constraintItr != mConstraints.end();++constraintItr) {
constraint = constraintItr->second;
//lambda = constraint->getLambda();
		
testLambda[0][0] = bigLambda[constraintCounter*3][0];
testLambda[1][0] = bigLambda[(constraintCounter*3)+1][0];
testLambda[2][0] = bigLambda[(constraintCounter*3)+2][0];
		
constraint->setLambda(testLambda); 
		
constraintCounter++;
}
*/	

}

//solution 1 end
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//solution algorithms 2 - Baraffs O(n^3),

/**
 * \brief This is Baraffs sparse factor algorithm without complexity optimizations. It has complexity O(n^3).
 
 * See Baraff in "SIGGRAPH Proceedings 96" p. 142
 */
void ConstraintSystem::computeSolution2() {
	cout << "Achtung! Code auskommentiert in computeSolution2()" << endl;
	//! \todo reparieren und wieder einkommentieren
/*

std::map<Id, RigidBodyPtr> bodies;//alle Knoten
std::map<Id, PrimaryConstraintPtr>::iterator pcItr;
	
for (pcItr = mConstraints.begin(); pcItr != mConstraints.end();++pcItr) {
//Knoten für die verbundenen Körper erzeugen, falls nicht schon vorhanden
Id objectAId = pcItr->second->getObjectA()->getId();
if (bodies.count(objectAId) == 0) {
bodies[objectAId] = boost::static_pointer_cast<RigidBody>(pcItr->second->getObjectA()); 
}
Id objectBId = pcItr->second->getObjectB()->getId();
if (bodies.count(objectBId) == 0) {
bodies[objectBId] = boost::static_pointer_cast<RigidBody>(pcItr->second->getObjectB()); 
}
		
pcItr->second->computeConstraint();
pcItr->second->getConstraint().show();
}
	
	
	


	
	
	
vector<Matrix<float> > matrixH;
vector<Matrix<float> > vectorX;
int matrixHDim = bodies.size()+mConstraints.size();
Matrix<float> dummyMatrix(1,1);
	
//initialize with matrices so that = operator works
for (int i = 0; i < matrixHDim * matrixHDim; ++i) {
matrixH.push_back(dummyMatrix.zero());
}
	
for (int i = 0; i < matrixHDim; ++i) {
vectorX.push_back(dummyMatrix.zero());
}
	
	
	
Matrix<float> jacobian1;
Matrix<float> jacobian2;
Matrix<float> rightHandSide1;
Matrix<float> rightHandSide2;
//Matrix<float> lambda;
Matrix<float> testLambda(3,1);
float mass;
Matrix<float> worldInertiaTensor;
	
	
Matrix<float> bigMass(6,6);
Matrix<float> bigA(3*mConstraints.size(),3*mConstraints.size());
Matrix<float> bigInvA;
	
Matrix<float> bigLambda;
Matrix<float> addedRightHandSide;
std::map<Id, PrimaryConstraintPtr>::iterator constraintItr;
std::map<Id, RigidBodyPtr>::iterator bodyItr;
	
std::map<Id, int> bodyPositions;
	
PrimaryConstraintPtr constraint;
	
int bodyCounter = 0;
	
	
//null initialisierung
for (unsigned int i=0; i<bodies.size();++i) {
vectorX[i] = Matrix<float> (6,1).zero();
for (unsigned int j=0; j<bodies.size();++j) {
matrixH[(i*matrixHDim)+j] = bigMass.zero();
}
}
	
	
{
int i = 0;
for(constraintItr = mConstraints.begin(); constraintItr != mConstraints.end();++constraintItr) {
//for (int i=0; i<mConstraints.size();++i) {
constraint = constraintItr->second;
Matrix<float> bigJacobian(constraint->getJacobian1().getSizeM(),6);
vectorX[bodies.size()+i] = Matrix<float> (constraint->getJacobian1().getSizeM(),1).zero();
		
for (unsigned int j=0; j<bodies.size();++j) {
matrixH[(matrixHDim*(bodies.size()+i))+j] = bigJacobian.zero();//lower left quadrant of H
matrixH[(matrixHDim*j)+bodies.size()+i] = bigJacobian.T().zero();//upper right quadrant of H
}
for (unsigned int j=0; j<mConstraints.size();++j) {
matrixH[(matrixHDim*(bodies.size()+i))+bodies.size()+j] = 
(bigJacobian*bigJacobian.T()).zero();//lower right quadrant
}
i++;
}
}	
	
	
for(bodyItr = bodies.begin(); bodyItr != bodies.end();++bodyItr) {
mass = bodyItr->second->getMass();
worldInertiaTensor = bodyItr->second->getWorldInertiaTensor();
		
for (int i = 0; i < 3; ++i) {
bigMass[i][i] = mass;
for (int j = 0; j < 3; ++j) {
bigMass[i+3][j+3] = worldInertiaTensor[i][j];
}
}
		
		
matrixH[(matrixHDim*bodyCounter)+bodyCounter]=bigMass;
		
bodyPositions[bodyItr->second->getId()] = bodyCounter;
bodyCounter++;
}
	
	
	
int constraintCounter = 0;
for(constraintItr = mConstraints.begin(); constraintItr != mConstraints.end();++constraintItr) {
constraint = constraintItr->second;
int numberBody1 = bodyPositions[constraint->getObjectA()->getId()];
int numberBody2 = bodyPositions[constraint->getObjectB()->getId()];
jacobian1 = constraint->getJacobian1();
jacobian2 = constraint->getJacobian2();
rightHandSide1 = constraint->getRightHandSide1();
rightHandSide2 = constraint->getRightHandSide2();
		
		
jacobian1.show();
jacobian2.show();
rightHandSide1.show();
rightHandSide2.show();
		
matrixH[(matrixHDim*(bodies.size()+constraintCounter))+numberBody1] = jacobian1;
matrixH[(matrixHDim*numberBody1)+bodies.size()+constraintCounter] = jacobian1.T();
matrixH[(matrixHDim*(bodies.size()+constraintCounter))+numberBody2] = jacobian2;
matrixH[(matrixHDim*numberBody2)+bodies.size()+constraintCounter] = jacobian2.T();
		
vectorX[bodies.size()+constraintCounter] = -1.0*(rightHandSide1 + rightHandSide2);
//vectorX[bodies.size()+constraintCounter] = (rightHandSide1 + rightHandSide2);
		
constraintCounter++;


}
	
	
	
// for (int i = 0; i < matrixHDim; i++) {
// for (int j = 0; j < matrixHDim; j++) {
// cout << matrixH[(i*matrixHDim)+j].getSizeM() << "x" << matrixH[(i*matrixHDim)+j].getSizeN() << " ";
// }
		
// }
	
// for (int i = 0; i < matrixHDim; i++) {
// cout << endl << vectorX[i].getSizeM() << "x" << vectorX[i].getSizeN() << " ";
// }
	

	
// for (int i=0; i<matrixHDim*matrixHDim; i++) {
// cout << "Matrix " << i << ":\n";
// matrixH[i].show ();
// }
	
//densefactor
for (int i = 0; i < matrixHDim; ++i) {
	for (int k = i-1; k >= 0; k--) {
		(matrixH[(k*matrixHDim)+i].T () * matrixH[(k*matrixHDim)+k] * matrixH[(k*matrixHDim)+i]).show();
			
		matrixH[(i*matrixHDim)+i] = 
			matrixH[(i*matrixHDim)+i] - 
			(matrixH[(k*matrixHDim)+i].T () * matrixH[(k*matrixHDim)+k] * matrixH[(k*matrixHDim)+i]);
	}
	for (int j = i+1; j < matrixHDim; ++j) {
		for (int k = i-1; k >= 0; k--) {
			(matrixH[(k*matrixHDim)+i].T() * matrixH[(k*matrixHDim)+k] * matrixH[(k*matrixHDim)+j]).show();
				
			matrixH[(i*matrixHDim)+j] = 
				matrixH[(i*matrixHDim)+j] - 
				(matrixH[(k*matrixHDim)+i].T() * matrixH[(k*matrixHDim)+k] * matrixH[(k*matrixHDim)+j]);
		}
		matrixH[(i*matrixHDim)+j] = choleskyInverse(matrixH[(i*matrixHDim)+i])*matrixH[(i*matrixHDim)+j]; 
	}
}
	
// for (int i = 0; i < matrixHDim; i++) {
//   for (int j = 0; j < matrixHDim; j++) {
//   cout << matrixH[(i*matrixHDim)+j].getSizeM() << "x" << matrixH[(i*matrixHDim)+j].getSizeN() << " ";
//   }
//   cout << endl;
//   }
	
//   for (int i = 0; i < matrixHDim; i++) {
//   cout << endl << vectorX[i].getSizeM() << "x" << vectorX[i].getSizeN() << " ";
//   }
//   cout << endl;
	
//densesolve
for (int i = 0; i < matrixHDim; ++i) {
	for (int j=0; j < i; j++) {
// 		cout << "i : " << i << " j: " << j << endl;
// 		  cout << "minus first MxN: " << vectorX[i].getSizeM() << "x" << vectorX[i].getSizeN() << endl;
// 		  cout << "first MxN: " << matrixH[(j*matrixHDim)+i].T().getSizeM() << "x" << matrixH[(j*matrixHDim)+i].T().getSizeN() << endl;
// 		  cout << "second MxN: " << vectorX[j].getSizeM() << "x" << vectorX[j].getSizeN() << endl << endl;
// 		
		vectorX[i] = vectorX[i] - (matrixH[(j*matrixHDim)+i].T() * vectorX[j]);
			
	}
}
	
	
	
	
// for (int i = 0; i < matrixHDim; i++) {
//   for (int j = 0; j < matrixHDim; j++) {
//   matrixH[(i*matrixHDim)+j].show();
//   cout << endl;
//   }
//   }
//   cout << "-----" << endl;
	
for (int i = matrixHDim-1; i >= 0; --i) {
	vectorX[i] = choleskyInverse(matrixH[(i*matrixHDim)+i]) * vectorX[i];
	for (int j = i+1; j < matrixHDim; ++j) {
		vectorX[i] = vectorX[i] - (matrixH[(i*matrixHDim)+j] * vectorX[j]);
	}
}
	
	
	
	
//vector<Matrix<float> >::iterator testItr;
	
	
// for (testItr = vectorX.begin(); testItr != vectorX.end(); ++testItr) {
//   testItr->show();
//   cout << endl;
//   }
	
//write back
constraintCounter = 0;
for(pcItr = mConstraints.begin(); pcItr != mConstraints.end();++pcItr) {
	constraint = pcItr->second;
		
	constraint->setLambda(vectorX[bodies.size()+constraintCounter]); 
		
	constraintCounter++;
}

*/
}

//solution 2 end
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//solution algorithms 3 - Baraffs O(n), recursive - OUT_OF_DATE

/**
\brief  This is out first implementation of Baraffs O(n) solution algorithm (Baraff 146). 

The idea was to omit the ordermatrix-step in Baraff's algorithms by recursive traversal (depth-first, breadth first). However, 
this has proven not equivalent to the iterative traveral of the ordermatrix output. So this method, as well as 
solveLinearSystem(), factorizeMatrix() and its according recursive functions are out of date. For now, we will keep them in the 
code for backup reasons.
*/
void ConstraintSystem::computeSolution3() {
	factorizeMatrix();
      solveLinearSystem();
}

/**
 * \brief Factorize constraint system matrix  _OUT OF DATE !!!_
 *
 * See Baraff in "SIGGRAPH Proceedings 96" p. 146
 */
void ConstraintSystem::factorizeMatrix ()
{

  std::map<Id, ConstraintMatrixNode*>::iterator rootsIterator;
  std::map<Id, ConstraintMatrixNode*>::iterator rootsEnd;
  ConstraintMatrixNode* root;

  rootsEnd = mRoots.end ();
  for (rootsIterator = mRoots.begin (); rootsIterator != rootsEnd; ++rootsIterator) {
    root = rootsIterator->second;
    factorizeMatrixPart1 (root);
    factorizeMatrixPart2 (root);
  }
  //printGraphs();

}

/**
 * \brief Recursion for facorizeMatrix  Part 1 _OUT OF DATE !!!_
 *
 * See Baraff in "SIGGRAPH Proceedings 96" p. 146
 */
void ConstraintSystem::factorizeMatrixPart1 (ConstraintMatrixNode* matrixNode)
{

	cout << "Achtung. Auskommentierter Code soll ausgeführt werden!" << endl;
	//! \todo Soll das nich einfach entfernt werden?

/*
// --- Backward ---
// do nothing!

// --- Recursion ---
// for all children
std::vector<ConstraintMatrixNode*>::iterator childrenIterator;
std::vector<ConstraintMatrixNode*>::iterator childrenEnd;
ConstraintMatrixNode* child;
childrenIterator = matrixNode->getChildren ()->begin ();
childrenEnd      = matrixNode->getChildren ()->end ();
for (; childrenIterator != childrenEnd; ++childrenIterator) {
child = *childrenIterator;
factorizeMatrixPart1 (child);
}
  

// --- Forward ---

// some variables needed for the computations
unsigned int dimConstraint;
RigidBodyPtr body;
PrimaryConstraintPtr constraint;
ConstraintMatrixNode* parent;
float mass;
Matrix3x3 inertiaTensor;
Matrix6x6 spaceMass;

// if the current Node is a Constaint:
// set the Node's Diagonal = zero...
if (matrixNode->isConstraint ()) {
constraint = matrixNode->getConstraint ();
dimConstraint = constraint->getConstraint().getSizeM ();
matrixNode->setDiagonal(Matrix6x6(0.0f));
}

// if the current Node is a Body:
// set the current Node's Diagonal the RigidBody's Mass matrix
else {
body = matrixNode->getBody ();
mass = body->getMass ();
inertiaTensor = body->getWorldInertiaTensor ();
assert (inertiaTensor.getSizeM () == 3);
assert (inertiaTensor.getSizeN () == 3);
spaceMass = Matrix6x6(0.0f);
for (unsigned int i=0; i<3; ++i) {
spaceMass [i][i] = mass;
for (unsigned int j=0; j<3; ++j) {
spaceMass [i+3][j+3] = inertiaTensor [i][j];
}
}
matrixNode->setDiagonal (spaceMass);
}

// if parent != NULL: set Jacobian according to what follows now
parent = matrixNode->getParent ();
if (parent != NULL) {

    
    
// if the current Node is a Constraint:
// set Jacobian the first Jacobian of the corresponding PrimaryConstraint
// according to the inverse flag of the current Node
if (matrixNode->isConstraint ()) {
constraint = matrixNode->getConstraint ();
//if (!(matrixNode->isInverse ())) {
//matrixNode->setJacobian (constraint->getJacobian1 ());
//}
//else {
matrixNode->setJacobian (constraint->getJacobian2 ());
//}
}

// if the current Node is not a constraint:
// its parent Node will be a Constraint,
// set Jacobian the parent Node's corresponding PrimaryConstraint's
// second Jacobian ^T according to the inverse flag of the current Node
else {
assert (parent->isConstraint ());
constraint = parent->getConstraint ();
//if (!(matrixNode->isInverse ())) {
//  matrixNode->setJacobian (constraint->getJacobian2 ().T ());
//}
//else {
matrixNode->setJacobian (constraint->getJacobian1 ().T ());
//}
}

//debug    
//printf("Jacobian set for %i:%i\n",matrixNode->getId().getType(),matrixNode->getId().getNumber());
//matrixNode->getJacobian().show();
}
*/
}

//{@
/**
 * \brief Recursion for facorizeMatrix  Part 1 _OUT OF DATE !!!_
 *
 * See Baraff in "SIGGRAPH Proceedings 96" p. 146
 */
void ConstraintSystem::factorizeMatrixPart2 (ConstraintMatrixNode* matrixNode)
{

	cout << "Achtung! Veraltete funktion void ConstraintSystem::factorizeMatrixPart2 (ConstraintMatrixNode* matrixNode)" << endl;
/*
  // --- Backward ---
  // do nothing!

  // --- Recursion ---
  // for all children
  std::vector<ConstraintMatrixNode*>::iterator childrenIterator;
  std::vector<ConstraintMatrixNode*>::iterator childrenEnd;
  ConstraintMatrixNode* child;
  childrenIterator = matrixNode->getChildren ()->begin ();
  childrenEnd      = matrixNode->getChildren ()->end ();
  
  	
	
  for (; childrenIterator != childrenEnd; ++childrenIterator) {
    child = *childrenIterator;
    factorizeMatrixPart2 (child);
  }

  // --- Forward ---

  // some useful variables...
  Matrix6x6 jacobianDiagonalJacobian;

  // for all children c of the current node:
  // Diagonal -= c.Jacobian^T c.Diagonal c.Jacobian
  std::vector<ConstraintMatrixNode*>::iterator cIterator;
  std::vector<ConstraintMatrixNode*>::iterator cEnd;
  ConstraintMatrixNode* currentChild;
  cIterator = matrixNode->getChildren ()->begin ();
  cEnd      = matrixNode->getChildren ()->end ();
  for (; cIterator != cEnd; ++cIterator) {
    currentChild = *cIterator;
    
    //debug
    //printf("Jacobian Size M: %i\n",matrixNode->getJacobian().getSizeM());
    //printf("Jacobian Size N: %i\n",matrixNode->getJacobian().getSizeN());
    //printf("Jacobian get for %i:%i\n",matrixNode->getId().getType(),matrixNode->getId().getNumber());
    //matrixNode->getJacobian().show();
    
    jacobianDiagonalJacobian = currentChild->getJacobian ().T () *
                              (currentChild->getDiagonal () *
                               currentChild->getJacobian ());
    matrixNode->setDiagonal (matrixNode->getDiagonal () -
                             jacobianDiagonalJacobian);
  }

  // set DiagonalInv the inverse of Diagonal
  // CHOLESKY_INVERSE MUSS ZUR MATRIX-KLASSE GEHOEREN!!!!! -> STEPHAN!
  
  matrixNode->setDiagonalInv(choleskyInverse(matrixNode->getDiagonal()));

  // if exists parent of current node:
  // set Jacobian = DiagonalInv * Jacobian
  if (matrixNode->getParent ()) {
    matrixNode->setJacobian (matrixNode->getDiagonalInv () *
                             matrixNode->getJacobian ());
  }
*/
}
//@}


/**
 * \brief solve constraint system linear system _OUT OF DATE !!!_
 
 * See Baraff in "SIGGRAPH Proceedings 96" p. 146
 */
void ConstraintSystem::solveLinearSystem ()
{
	std::map<Id, ConstraintMatrixNode*>::iterator rootsIterator;
	std::map<Id, ConstraintMatrixNode*>::iterator rootsEnd;
	ConstraintMatrixNode* root;
	rootsIterator = mRoots.begin ();
	rootsEnd = mRoots.end ();
	for (; rootsIterator != rootsEnd; ++rootsIterator) {
		root = rootsIterator->second;
		solveLinearSystemPart1(root);
		solveLinearSystemPart2(root);
	}


}

/**
 * \brief solve constraint system linear system, part 1 _OUT OF DATE !!!_
 
 * See Baraff in "SIGGRAPH Proceedings 96" p. 146
 */
void ConstraintSystem::solveLinearSystemPart1(ConstraintMatrixNode* node) {

	cout << "Achtung. Auskommentierter code soll ausgeführt werden (ConstraintSystem::solveLinearSystemPart1" << endl;
/*
//variables
Matrix6x1 rightHandSides;
Matrix6x1 lambda;
PrimaryConstraintPtr constraint;
//ConstraintMatrixNode* parent;
ConstraintMatrixNode* currentChild;
std::vector<ConstraintMatrixNode*>::iterator childItr;
std::vector<ConstraintMatrixNode*>* children;
Matrix6x1 zero(0.0f);
	
//recursion
children = node->getChildren();
for(childItr = children->begin(); childItr != children->end();++childItr) {
solveLinearSystemPart1(*childItr);
}
	
//do things here
if(node->isConstraint()) {
constraint = node->getConstraint();
rightHandSides = -1.0f * (constraint->getRightHandSide1() + constraint->getRightHandSide2());
node->setLambda(rightHandSides);
} else {
		
node->setLambda(zero);
}
	
children = node->getChildren();
for(childItr = children->begin(); childItr != children->end();++childItr) {
currentChild = *childItr;
lambda = node->getLambda();
lambda =  lambda - (currentChild->getJacobian().T() * currentChild->getLambda());
node->setLambda(lambda);
}
*/
}

/**
 * \brief solve constraint system linear system, part 2 _OUT OF DATE !!!_
 
 * See Baraff in "SIGGRAPH Proceedings 96" p. 146
 */
void ConstraintSystem::solveLinearSystemPart2(ConstraintMatrixNode* node) {
	
	cout << "Achtung. Auskommentierter code soll ausgeführt werden (ConstraintSystem::solveLinearSystemPart2" << endl;
/*
//variables
std::vector<ConstraintMatrixNode*>::iterator childItr;
std::vector<ConstraintMatrixNode*>* children;
Matrix6x1 lambda;
ConstraintMatrixNode* parent;
	
//debug
//printf("Node %i:%i\n",node->getId().getType(),node->getId().getNumber());
//node->getDiagonalInv().show();
//node->getLambda().show();
	
//do things here
lambda = node->getDiagonalInv() * node->getLambda();
	
	
	
node->setLambda(lambda);
	
parent = node->getParent();

if (parent != NULL) {
lambda = node->getLambda();
lambda = lambda - (node->getJacobian() * parent->getLambda());
node->setLambda(lambda);
}
	
if (node->isConstraint()) {
lambda = node->getLambda();
//if (node->isInverse()) {
// 	lambda = -1.0 * lambda;
//} 
node->getConstraint()->setLambda(lambda);
//GEHÖRT EIGENTLICH IN EINE EIGENE FKT
//updaten der rigid bodys
//node->getConstraint()->updateRigidBodies();
}
	
//recursion
children = node->getChildren();
for(childItr = children->begin(); childItr != children->end();++childItr) {
solveLinearSystemPart2(*childItr);
}
*/
}

//solution 3 end
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//solution algorithms 4 - Baraffs O(n), iterative

/**
\brief This is our recent implementation of Baraffs O(n) solution algorithm (Baraff 146).

This algorithm solves the A*lambda=b linear system. It has been introduced bya David Baraff in his paper 
(the paper we) refer to all the time ;). It uses the graphs built earlier in the implementation (buildGraphs).

The algorithm has 3 steps:
-ordermatrix: 
This step only generate a specially sorted list of all nodes in the tree. 
This list is traversed later in the algorithm

-factorize (See Baraff paper)
-solve (See Baraff paper)

 \todo Besser Doku von factorize und solve
*/
void ConstraintSystem::computeSolution4() {
	std::map<Id, ConstraintMatrixNode*>::iterator rootsIterator;
	std::map<Id, ConstraintMatrixNode*>::iterator rootsEnd;
	ConstraintMatrixNode* root;
	
	rootsIterator = mRoots.begin();
	rootsEnd = mRoots.end();
	
	//process all independant graphs seperately
	for (; rootsIterator != rootsEnd; ++rootsIterator) {
		root = rootsIterator->second;
		//empty mList
		mList.clear();
		ordermatrix(root);
		factor();
 		solve();
 	}
}


/**
 * \brief Order the nodes in the computation graph
 *
 * See Baraff in "SIGGRAPH Proceedings 96" p. 146
 *
 * This function generates the Forward-List of Baraffs algorithms. As the stl::list-structure can traverse its data 
 * in both directions, Backward is not build. Instead, solve() traverses the forward-list in reverse. The forward list is stored
 * in mList, an has to be recomputed for every graph in the system.
 */
void ConstraintSystem::ordermatrix(ConstraintMatrixNode* node) {
	std::vector<ConstraintMatrixNode*>::iterator childItr;
	std::vector<ConstraintMatrixNode*>* children;
	
	children = node->getChildren();
	std::vector<ConstraintMatrixNode*>::iterator end = children->end();
	for(childItr = children->begin(); childItr != end; ++childItr) {
		ordermatrix(*childItr);
	}
	mList.push_back(node);
}


/**
 * \brief Factorize constraint system matrix
 *
 * See Baraff in "SIGGRAPH Proceedings 96" p. 146
 */
void ConstraintSystem::factor() {
	std::list<ConstraintMatrixNode*>::iterator listItr;
	
	// some variables needed for the computations
	RigidBodyPtr body;
	PrimaryConstraintPtr constraint;
	ConstraintMatrixNode* parent;
	float mass;
	Matrix3x3 inertiaTensor(0.0);
	Matrix6x6 spaceMass(0.0);
	ConstraintMatrixNode *matrixNode;
	
	//forward
	std::list<ConstraintMatrixNode*>::iterator listEnd = mList.end();
	for (listItr = mList.begin(); listItr != listEnd; ++listItr) {
		matrixNode = *listItr;
  		// if the current Node is a Constaint:
  		// set the Node's Diagonal = zero...
  		if (matrixNode->isConstraint()) {
			constraint = matrixNode->getConstraint();
			matrixNode->setDiagonal (Matrix6x6(0.0f));
  		}
		
		// if the current Node is a Body:
		// set the current Node's Diagonal the RigidBody's Mass matrix
		else {
			body = matrixNode->getBody();
			mass = body->getMass();
			inertiaTensor = (body->getWorldInertiaTensor());
			assert (inertiaTensor.getSizeM() == 3);
			assert (inertiaTensor.getSizeN() == 3);
			spaceMass = Matrix6x6(0.0f);
			for (unsigned int i=0; i<3; ++i) {
				spaceMass [i][i] = mass;
				for (unsigned int j=0; j<3; ++j) {
					spaceMass [i+3][j+3] = inertiaTensor [i][j];
				}
			}
			matrixNode->setDiagonal (spaceMass);
		}
		
		// if parent != NULL: set Jacobian according to what follows now
		parent = matrixNode->getParent ();
		if (parent != NULL) {

			// if the current Node is a Constraint:
			// set Jacobian the first Jacobian of the corresponding PrimaryConstraint
			// according to the inverse flag of the current Node
			if (matrixNode->isConstraint ()) {
				constraint = matrixNode->getConstraint ();
				
				matrixNode->setJacobian (constraint->getJacobian1());			}
	
			// if the current Node is not a constraint:
			// its parent Node will be a Constraint,
			// set Jacobian the parent Node's corresponding PrimaryConstraint's
			// second Jacobian ^T according to the inverse flag of the current Node
			else {
				assert (parent->isConstraint ());
				constraint = parent->getConstraint ();
				matrixNode->setJacobian (constraint->getJacobian2 ().T ());
			}
  		}
	}
	
		
	std::vector<ConstraintMatrixNode*>::iterator cIterator;
	std::vector<ConstraintMatrixNode*>::iterator cEnd;
	ConstraintMatrixNode* currentChild;
	Matrix6x6 jacobianDiagonalJacobian;
	
	//forward
	for (listItr = mList.begin(); listItr != mList.end(); ++listItr) {
		matrixNode = *listItr;
		// some useful variables...  
		// for all children c of the current node:
		// Diagonal -= c.Jacobian^T c.Diagonal c.Jacobian
		cIterator = matrixNode->getChildren ()->begin ();
		cEnd      = matrixNode->getChildren ()->end ();
		for (; cIterator != cEnd; ++cIterator) {
			currentChild = *cIterator;
			
			jacobianDiagonalJacobian = currentChild->getJacobian ().T () *
				(currentChild->getDiagonal () *
				 currentChild->getJacobian ());
			matrixNode->setDiagonal (matrixNode->getDiagonal () -
									 jacobianDiagonalJacobian);
		}

		if (matrixNode->isConstraint()) {
			if (matrixNode->getConstraint()->getDegreeOfFreedom() == PrimaryConstraint::dof3) {
				Matrix3x3 temporal;
				matrixNode->getDiagonal().get3x3Matrix(0,0,temporal);
				Matrix6x6 temporal2(0.0f);
				temporal2.setValues(choleskyInverse(temporal));
				matrixNode->setDiagonalInv(temporal2);
			} else
				cout << "Arrrg. HingeConstraint is not yet implementet!" << endl;
		} else
			matrixNode->setDiagonalInv (choleskyInverse (matrixNode->getDiagonal ()));

		// if exists parent of current node:
		// set Jacobian = DiagonalInv * Jacobian
		if (matrixNode->getParent ()) {
			matrixNode->setJacobian (matrixNode->getDiagonalInv () *
									 matrixNode->getJacobian ());
		}	
	}
}

/**
 * \brief solve constraint system linear system
 
 * See Baraff in "SIGGRAPH Proceedings 96" p. 146.
 *
 * \param postStabilization indicates that the linear system is
 * not to be solved to compute constraint forces but to do a
 * post stabilization step. Please note that post stabilization
 * results are processed immediately. For information on how
 * post stabilization is computed see Cline and Pai "Post-
 * Stabilization for Rigid Body Simulation with Contact and
 * Constraints", Proceedings of IEEE International Condference
 * on Robotics and Autom. 2003, section V, equation (14).
 * Post stabilization for contact forces is not implemented
 * as the computation of post stabilization is made to use
 * Baraffs linear system for computing bilateral equality constraints.
 */
void ConstraintSystem::solve(bool postStabilization) {
	std::list<ConstraintMatrixNode*>::iterator listItr;
	std::list<ConstraintMatrixNode*>::reverse_iterator listReverseItr;
	
	//variables
	Matrix6x1 rightHandSides;
	Matrix6x1 lambda;
	PrimaryConstraintPtr constraint;
	ConstraintMatrixNode* parent;
	ConstraintMatrixNode* node;
	ConstraintMatrixNode* currentChild;
	std::vector<ConstraintMatrixNode*>::iterator childItr;
	std::vector<ConstraintMatrixNode*>* children;
	Matrix6x1 zero(0.0f);
	
	std::list<ConstraintMatrixNode*>::iterator endOfList = mList.end();
	//forward
	for (listItr = mList.begin(); listItr != endOfList; ++listItr) {
		node = *listItr;
	
		if(node->isConstraint()) {
			constraint = node->getConstraint();
			if (!postStabilization)
				rightHandSides = -1.0f * (constraint->getRightHandSide1() + constraint->getRightHandSide2());
			else
				rightHandSides = -1.0f * (constraint->getConstraint());
			//rightHandSides = (constraint->getRightHandSide1() + constraint->getRightHandSide2());
			node->setLambda(rightHandSides);
		} else {
			
			node->setLambda(zero);
		}
	
		children = node->getChildren();
		std::vector<ConstraintMatrixNode*>::iterator endOfChilds = children->end();
		for(childItr = children->begin(); childItr != endOfChilds; ++childItr) {
			currentChild = *childItr;
			lambda = node->getLambda();
			lambda =  lambda - (currentChild->getJacobian().T() * currentChild->getLambda());
			node->setLambda(lambda);
		}
	}
	
	
	//backward
	std::list<ConstraintMatrixNode*>::reverse_iterator endOfReverse = mList.rend();
	for (listReverseItr = mList.rbegin(); listReverseItr != endOfReverse; ++listReverseItr) {
		node = *listReverseItr;
		lambda = node->getDiagonalInv() * node->getLambda();
		node->setLambda(lambda);
	
		parent = node->getParent();

		if (parent != NULL) {
			lambda = node->getLambda();
			lambda = lambda - (node->getJacobian() * parent->getLambda());
			node->setLambda(lambda);
		}
		
		if (node->isConstraint() && !postStabilization) {
			lambda = node->getLambda();
			node->getConstraint()->setLambda(lambda);
		}
		if (!(node->isConstraint()) && postStabilization) {
			lambda = node->getLambda ();
			node->getBody ()->processConstraintPostStabilization (lambda);
		}
	}
}

//solution 4 end
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
