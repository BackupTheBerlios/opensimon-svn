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
Universitaet Koblenz - Computergraphik
Projektpraktikum  --- "MARIONETTE" ---
*/

// $Id: ConstraintSystem.h,v 1.32 2004/12/14 18:29:46 alangs Exp $
//------------------------------------------------------------------------------
/**
*  \file ConstraintSystem.h
*  \class ConstraintSystem
*  \author Stephan Palmer
*  \author Nils Hornung
*  $Author: alangs $
*  $Date: 2004/12/14 18:29:46 $
*  $Revision: 1.32 $
*/
//------------------------------------------------------------------------------

//includes
#include <simon/BallAndSocketConstraint.h>
#include <simon/HingeConstraint.h>
#include <simon/ConstraintMatrixNode.h>
#include <simon/Matrix.h>
#include <simon/Id.h>
#include <simon/RigidBody.h>
#include <map> //stl maps
#include <list> //stl lists



#ifndef CONSTRAINT_SYSTEM_H
#define CONSTRAINT_SYSTEM_H

class ConstraintSystem {
	public:
		ConstraintSystem();//Konstructor
		~ConstraintSystem();//Destructor
		
		//create and access constraints 
		//! \todo einen BallAndSocketConstraint zurückliefern!
		void createBallAndSocketConstraint(Id Ident, RigidBodyPtr, RigidBodyPtr, Vec3, Vec3);
		BallAndSocketConstraintPtr getBallAndSocketConstraint(Id Ident);
		
		void createHingeConstraint(Id Ident, RigidBodyPtr, RigidBodyPtr, Vec3, Vec3, Vec3, Vec3, bool = true);
		HingeConstraintPtr getHingeConstraint(Id Ident);
		
		//functions for initialization
		void buildGraphs(); //build the graphs out of the constraints
		
		//function for printing
		void printGraphs(); //for test purposes
		
		//do one computation step
		void step();//Funktion zum ausrechnen des nächsten Schritts

		// the same as step (for style-guide purposes)
		void computeConstraints ();

		// compute post stabilization
		void computePostStabilization (bool fast = false);
		
		//which computeSolution?() to use
		void setComputationAlgorithm(int);
		
		 /*void enableMaxForceLengthCheck();
    		 void disableMaxForceLengthCheck();
		 void setMaxForceLength();
    		 void disableMaxForceLengthCheck();*/
		  void setTau(float);
		
	private:
		//data elements
		//pointers to constraints
		std::map<Id, PrimaryConstraintPtr> mConstraints;//(Basisklassen-)Zeiger auf alle Constraints
		std::map<Id, BallAndSocketConstraintPtr> mBallAndSocketConstraints;//Zeiger auf alle BallAndSocketConstraints
		std::map<Id, HingeConstraintPtr> mHingeConstraints;//Zeiger auf alle BallAndSocketConstraints
		std::map<Id, ConstraintMatrixNodePtr> mConstraintNodes;//Zeiger auf alle ConstraintMatrixNodes
		std::map<Id, ConstraintMatrixNodePtr> mBodyNodes;//Zeiger auf alle BodyMatrixNodes
		
		//pointer to graph nodes
		std::map<Id, ConstraintMatrixNode*> mRoots;//Roots der Bäume
		
		//element list for ordermatrix, factor, solve (siehe Baraff, 146)
		std::list<ConstraintMatrixNode*> mList;
		
		//which altgorithm to use
		int mNumberOfAlgorithm; 
		
		
		//Rekursion fuer printGraphs
		void printGraph(ConstraintMatrixNode*, int depth);
		
		//Rekursion fpr computeValues
		void computeNodeValues(ConstraintMatrixNode*);

		// Rekursion fuer factorizeMatrix _DEPRACTED_
		void factorizeMatrixPart1 (ConstraintMatrixNode* matrixNode);
		void factorizeMatrixPart2 (ConstraintMatrixNode* matrixNode);
		// Rekursion fuer solveLinearSystem _DEPRACTED_
		void solveLinearSystemPart1 (ConstraintMatrixNode* matrixNode);
		void solveLinearSystemPart2 (ConstraintMatrixNode* matrixNode);
		
		//the different sulition algorithms and related methodes
		void computeSolution1();//Brute Force Matrix Invertierung mit cholesky
		void computeSolution2();//Baraff Algorithmus O(n^3) (Baraff 142)
		void computeSolution3();//Baraff Algorithmus O(n) (Baraff 146) __OLD_VERSION
		void computeSolution4();//Baraff Algorithmus O(n) (Baraff 146)
		
		
		//functions for computations in each step
		//O(n)-algorithms
		void factorizeMatrix (); // factorize constraint system matrix _DEPRACTED_
		void solveLinearSystem (); // solve constraint system linear system _DEPRACTED_
		void factor();//factor-part of Baraffs O(n)-algorithm
		void solve(bool postStabilization = false);//solve-part of Baraffs O(n)-algorithm
		void ordermatrix(ConstraintMatrixNode*);//build element list for one tree for factor, solve
		
		//pre and post solution methods
		void computeValues(); //executes computation of misc values for each constraint
		void updateAllRigidBodies();//writes the constraint forces back into the rigid bodies

		// any function can use this pointer temporal
		PrimaryConstraintPtr mTempConstraintPtr;
};

#endif
