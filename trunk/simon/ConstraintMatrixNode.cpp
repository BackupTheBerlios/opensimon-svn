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


/* Universitaet Koblenz - Computergraphik
Projektpraktikum  --- "MARIONETTE" ---
File: PrimaryConstraint.cpp*/
// $Id: ConstraintMatrixNode.cpp,v 1.21 2004/11/02 12:32:51 trappe Exp $
/**
 *  \class ConstraintMatrixNode
 *  \author Palmer, Hornung
 *
 * CVS:
 *   - $Author: trappe $
 *   - $Date: 2004/11/02 12:32:51 $
 *   - $Revision: 1.21 $
 *
 * \brief Klasse, die einen Bestandteil der gesamten Constraint Matrix darstellt.
 *
 * Zur Berechnung der Zwangskräfte mit Lagrange Multiplikatoren muss eine Matrix aus Constraints und
 * Körpermassen aufgestellt werden. Diese Klasse stellt die nicht mit 0 besetzten Bestandteile der Matrix dar.
 *
 * \todo 
 * - assert in get/set besser mit try-catch
 */

// ////////////////////////////////////////////////////////////////////////////

/**
\brief Includes
*/
#include "ConstraintMatrixNode.h"
#include "assert.h"

using namespace std;

/**
\brief Constructor for object node

\param object The world object to represent with this node
*/
ConstraintMatrixNode::ConstraintMatrixNode (RigidBodyPtr object) :
	mDiagonal(0.0),
	mDiagonalInv(0.0),
	mJacobian(0.0),
	mLambda(0.0) {
	mConstraint = PrimaryConstraintPtr();
	mBody = object;
	mParent = NULL;
}

/**
\brief Constructor for constraint node

\param object The constraint to represent with this node
*/
ConstraintMatrixNode::ConstraintMatrixNode (PrimaryConstraintPtr constraint) :
	mDiagonal(0.0),
	mDiagonalInv(0.0),
	mJacobian(0.0),
	mLambda(0.0) {

	mConstraint = constraint;
	mBody = RigidBodyPtr();
	mParent = NULL;
}

/**
\brief Return if node represents an objcet or not
*/
bool ConstraintMatrixNode::isBody () {
	if (mBody) {
		return true;
	} else {
		return false;
	}
}

/**
\brief Return if node represents a constraint or not
*/
bool ConstraintMatrixNode::isConstraint () {

	if (mConstraint) {
		return true;
	} else {
		return false;
	}

}

/**
\brief Return constraint pointer (can be NULL)
*/
PrimaryConstraintPtr ConstraintMatrixNode::getConstraint () {
	return mConstraint;
}

/**
\brief Return constraint pointer (can be NULL)
*/
void ConstraintMatrixNode::getConstraint(PrimaryConstraintPtr &constraint) {
	constraint = mConstraint;
}

/**
\brief Return body constraint pointer (can be NULL)
*/
RigidBodyPtr ConstraintMatrixNode::getBody() {
	return mBody;
}

/**
\brief Return number of child nodes
*/
int ConstraintMatrixNode::numberOfChildren () {
	return (int) mChildren.size();
}

/**
\brief Set the parent of the node
\param ConstraintMatrixNode& node, Referenz auf den Knoten, der als Vater gesetzt werden soll
*/
void ConstraintMatrixNode::setParent (ConstraintMatrixNode* node) {
	mParent = node;
}

/**
\brief Returns the parent of the node
*/
ConstraintMatrixNode* ConstraintMatrixNode::getParent () {
	return mParent;
}

/**
\brief Set the parent of the node
\param ConstraintMatrixNode& node, Referenz auf den Knoten, der als Kind gesetzt werden soll
\param int indexOfChild, index des zu ersetzenden Kinds
*/
void ConstraintMatrixNode::setChild (int indexOfChild, ConstraintMatrixNode* node) {
	assert(indexOfChild < (int)mChildren.size());
	mChildren[indexOfChild] = node;
}

/**
\brief Set the parent of the node
\param ConstraintMatrixNode& node, Referenz auf den Knoten, der als Kind hinzugefügt werden soll
*/
void ConstraintMatrixNode::addChild (ConstraintMatrixNode* node) {
	mChildren.push_back(node);
}

/**
\brief Returns the parent of the node
\param int indexOfChild, index des zu ersetzenden Kinds
*/
ConstraintMatrixNode* ConstraintMatrixNode::getChild (unsigned int indexOfChild) {
	assert(indexOfChild < mChildren.size());
	/*std::map<Id, ConstraintMatrixNode*, compareIds>::iterator nodeItr = mChildren.begin();
	for (unsigned int i = 0; i < indexOfChild;++i) {//das muss doch schöner gehen
		++nodeItr;
	}
	return nodeItr->second;*/
	return mChildren[indexOfChild];
	
}

//@{
/**
\brief Set / get the inverse state of the node
\param inv, boolean flag (true=inverse)
*/
void ConstraintMatrixNode::setInverse (bool inv)
{mIsInverse = inv;}

bool ConstraintMatrixNode::isInverse ()
{return mIsInverse;}
//@}

/**
\brief Return Id of represented constraint/body
\return Id

A ConstraintMatrixNode has no Id of its own. Other parts of the Constraint System need to know 
the Id of the represented body/constraint. This function makes te access to this information easier.
*/
Id ConstraintMatrixNode::getId () {
	if (isConstraint()) {
		return mConstraint->getId();
	}
	if (isBody()) {
		return mBody->getId();
	}
	return Id(Id::typeNone,0);
}

/**
\brief Return pointer to mChildren

*/
std::vector<ConstraintMatrixNode*>* ConstraintMatrixNode::getChildren() {
	return &mChildren;
}

//@{
/**
 * \brief Convenient get-/set-Methods for Baraff members
 *
 * \todo Ist blockIndex überflüssig? Stephan fragen!
 */
const Matrix6x6 ConstraintMatrixNode::getDiagonal    ()
{return mDiagonal;}

const Matrix6x6 ConstraintMatrixNode::getDiagonalInv ()
{return mDiagonalInv;}

const Matrix6x6 ConstraintMatrixNode::getJacobian    ()
{return mJacobian;}

const Matrix6x1 ConstraintMatrixNode::getLambda      ()
{return mLambda;}

const unsigned int   ConstraintMatrixNode::getBlockIndex  ()
{return mBlockIndex;}

void ConstraintMatrixNode::setDiagonal    (Matrix6x6 diagonal)
{mDiagonal = diagonal;}

void ConstraintMatrixNode::setDiagonalInv (Matrix6x6 diagonalInv)
{mDiagonalInv = diagonalInv;}

void ConstraintMatrixNode::setJacobian    (Matrix6x6 jacobian)
{mJacobian = jacobian;}

void ConstraintMatrixNode::setLambda      (Matrix6x1 lambda)
{mLambda = lambda;}

void ConstraintMatrixNode::setBlockIndex  (unsigned int   blockIndex)
{mBlockIndex = blockIndex;}
//@}
