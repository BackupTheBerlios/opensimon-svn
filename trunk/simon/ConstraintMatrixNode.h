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
// $Id: ConstraintMatrixNode.h,v 1.18 2004/12/14 18:29:46 alangs Exp $
//------------------------------------------------------------------------------
/**
*  \file ConstraintMatrixNode.h
*  \class ConstraintMatrixNode
*  $Author: alangs $
*  $Date: 2004/12/14 18:29:46 $
*  $Revision: 1.18 $
*/
//------------------------------------------------------------------------------


//includes
#include <simon/WorldObject.h>
#include <simon/PrimaryConstraint.h>
#include <simon/Id.h>

#include <vector> //stl vector
#include <map> //stl vector


#ifndef CONSTRAINT_MATRIX_NODE_H
#define CONSTRAINT_MATRIX_NODE_H

class ConstraintMatrixNode {
public:
	//Anmerkung: Vorläufig werden die Referenzen der Objekte übergeben. Es kann sein das sich dies später als 
	//unpraktisch erweist
	ConstraintMatrixNode(RigidBodyPtr);
	ConstraintMatrixNode(PrimaryConstraintPtr);
		
	//um was für einen Knoten handelt es sich ?
	bool isBody();
	bool isConstraint();
		
	//Zugriff auf den Kindknoten
	RigidBodyPtr getBody();
	PrimaryConstraintPtr getConstraint();
	void getConstraint(PrimaryConstraintPtr &constraint);
		
	//Funktionen für Eltern-/Kind-Zeiger 
	int numberOfChildren();
	void setParent(ConstraintMatrixNode*);
	ConstraintMatrixNode* getParent();
	void addChild(ConstraintMatrixNode*);
	void setChild(int, ConstraintMatrixNode*);
	ConstraintMatrixNode* getChild(unsigned int indexOfChild);//return n-th child
	std::vector<ConstraintMatrixNode*>* getChildren();//return child vector
		
	Id getId(); //return Id of represented constraint or body
		
	//Inverse setting
	void setInverse(bool);
	bool isInverse ();

	// Convenients (get) for Baraff members
	const Matrix6x6 getDiagonal    ();
	const Matrix6x6 getDiagonalInv ();
	const Matrix6x6 getJacobian    ();
	const Matrix6x1 getLambda      ();
	const unsigned int   getBlockIndex  ();

	// Convenients (set) for Baraff members
	void setDiagonal    (Matrix6x6 diagonal);
	void setDiagonalInv (Matrix6x6 diagonalInv);
	void setJacobian    (Matrix6x6 jacobian);
	void setLambda      (Matrix6x1 lambda);
	void setBlockIndex  (unsigned int   blockIndex);
		
private:

	RigidBodyPtr mBody;
	PrimaryConstraintPtr mConstraint;
	//Eltern-/Kind-Zeiger
	ConstraintMatrixNode* mParent;
	std::vector<ConstraintMatrixNode*> mChildren;
	//Reihenfolge
	bool mIsInverse;
		
	//@{
	//! \brief Datenstruktur nach Baraff "Lagrange Multipl.", S.146
	Matrix6x6 mDiagonal;        //!< Matrix des Blocks 
	Matrix6x6 mDiagonalInv;     //!< Matrix des Blocks
	Matrix6x6 mJacobian;        //!< Matrix des Blocks
	unsigned int mBlockIndex;   //!< Index zur Numerierung aller Blöcke
	Matrix6x1 mLambda;          //!< Anteil des Blocks am Lösungsvektor
	//@}

};

#endif
