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


//------------------------------------------------------------------------------
/**
 * \class Box
 * \author Kilian, Kipermann, Haeusler
 *
 * \brief oriented Box für Kollisionstest
 */
//------------------------------------------------------------------------------
#include <cassert>
#include <simon/Box.h>
#include <simon/Sphere.h>
#include <simon/Capsule.h>
#include <simon/Plane.h>
#include <simon/SimonState.h>
#include <simon/Clip.h>
#include <simon/Clock.h>

#include <iostream>

using namespace std;

// Graphik-Zugriff nur für Testzwecke. Im Multi-Thread-Betrieb nicht erlaubt.
//#define TEST_ENVIRONMENT
//#define MASSIVE_OUTPUT
/**
* \brief Constructor mit Angaben zur HÃ¶he, Breite und LÃ¤nge.
* \param rigidBody FestkÃ¶rper, dem die Geometrie zugeordnet werden soll.
* \param scale Skalierung der Box, aus HÃ¶he, Breite und Tiefe
*/
Box::Box(RigidBodyPtr& rigidBody, Vec3 scale)
	:Geometry(rigidBody) {

	mScale = scale;
	float mass = mRigidBody->getMass();
	float tensorElement1 = (mass * (getHeight()* getHeight() + getDepth() * getDepth()))*10; 
	float tensorElement2 = (mass * (getWidth()* getWidth() + getDepth() * getDepth()))*10;
	float tensorElement3 = (mass * (getWidth()* getWidth() + getHeight() * getHeight()))*10;
	mRigidBody->setInertiaTensor(tensorElement1, tensorElement2, tensorElement3);
};
        
/**
* \brief Destruktor
*/
Box::~Box(){
}


/**
* \brief testet intersection mit OBB (wird fuer box-caps benutzt)
* \param positionOBB position der OBB
* \param oriOBB orientierung der 0BB
* \param scaleOBB ausdehnung der OBB	
*/
bool Box::intersectionWithOBB (Vec3 positionOBB, Quaternion oriOBB, Vec3 scaleOBB)
{
	float c[3][3];
	float absC[3][3];
	float d[3];
	
	Vec3 normalsA[3];
	Vec3 normalsB[3];
	
	float r0, r1, r;

	int i;

	const float cutoff = 0.999999f;
	bool parallelPair = false;
	
	normalsA[0] = qRotate(Vec3(1,0,0), mRigidBody->getOrientation());
	normalsA[1] = qRotate(Vec3(0,1,0), mRigidBody->getOrientation());
	normalsA[2] = qRotate(Vec3(0,0,1), mRigidBody->getOrientation());
	
	normalsB[0] = qRotate(Vec3(1,0,0), oriOBB);
	normalsB[1] = qRotate(Vec3(0,1,0), oriOBB);
	normalsB[2] = qRotate(Vec3(0,0,1), oriOBB);
	
	
	// difference of box centers
	Vec3 diff = mRigidBody->getPosition() - positionOBB;
	
	//----------------------------------------------------------------------------------------------------------------
	//axis 1
	for (i  = 0; i < 3; i++)
	{
		c[0][i] = dot(normalsA[0], normalsB[i]);
		absC[0][i] = fabs(c[0][i]);
		if (absC[0][i] > cutoff)
			parallelPair = true;
	}
	d[0] = dot(diff, normalsA[0]);
	
	r = fabs(d[0]);
	r0 = getScale()[0];
	r1 = (scaleOBB[0] * absC[0][0]) + (scaleOBB[1] * absC[0][1]) + (scaleOBB[2] * absC[0][2]);
	
	if (r > r0 + r1)
		return false;

	//----------------------------------------------------------------------------------------------------------------	
	//axis 2
	for (i  = 0; i < 3; i++)
	{
		c[1][i] = dot(normalsA[1], normalsB[i]);
		absC[1][i] = fabs(c[1][i]);
		if (absC[1][i] > cutoff)
			parallelPair = true;
	}
	d[1] = dot(diff, normalsA[1]);
	
	r = fabs(d[1]);
	r0 = getScale()[1];
	r1 = (scaleOBB[0] * absC[1][0]) + (scaleOBB[1] * absC[1][1]) + (scaleOBB[2] * absC[1][2]);
	
	if (r > r0 + r1)
		return false;
	
	//----------------------------------------------------------------------------------------------------------------
	// axis 3
	for (i  = 0; i < 3; i++)
	{
		c[2][i] = dot(normalsA[2], normalsB[i]);
		absC[2][i] = fabs(c[2][i]);
		if (absC[2][i] > cutoff)
			parallelPair = true;
	}
	d[2] = dot(diff, normalsA[2]);
	
	r = fabs(d[2]);
	r0 = getScale()[2];
	r1 = (scaleOBB[0] * absC[2][0]) + (scaleOBB[1] * absC[2][1]) + (scaleOBB[2] * absC[2][2]);
	
	if (r > r0 + r1)
		return false;
	

	//----------------------------------------------------------------------------------------------------------------	
	// axis 4
	r = fabs(dot(diff, normalsB[0]));
	r0 = (getScale()[0] * absC[0][0]) + (getScale()[1] * absC[1][0]) + (getScale()[2] * absC[2][0]);
	r1 = scaleOBB[0];
	
	if (r > r0 + r1)
		return false;
		
	//----------------------------------------------------------------------------------------------------------------
	// axis 5
	r = fabs(dot(diff, normalsB[1]));
	r0 = (getScale()[0] * absC[0][1]) + (getScale()[1] * absC[1][1]) + (getScale()[2] * absC[2][1]);
	r1 = scaleOBB[1];
	
	if (r > r0 + r1)
		return false;
	

	//----------------------------------------------------------------------------------------------------------------	
	// axis 6
	r = fabs(dot(diff, normalsB[2]));
	r0 = (getScale()[0] * absC[0][2]) + (getScale()[1] * absC[1][2]) + (getScale()[2] * absC[2][2]);
	r1 = scaleOBB[1];
	
	if (r > r0 + r1)
		return false;

	
	//----------------------------------------------------------------------------------------------------------------
	// boxen parallel
	if (parallelPair)
	{
		#ifdef TEST_ENVIRONMENT
			setColor(Graphics::yellow);
		#endif
		//return false;
		return true;
	}
	
	
	//----------------------------------------------------------------------------------------------------------------	
	// edege 1
	r = fabs((d[2] * c[1][0]) - (d[1] * c[2][0]));
	r0 = (getScale()[1] * absC[2][0]) + (getScale()[2] * absC[1][0]);
	r1 = (scaleOBB[1] * absC[0][2]) + (scaleOBB[2] * absC[0][1]);
	
	if (r > r0 + r1)
		return false;
	
	
	//----------------------------------------------------------------------------------------------------------------
	// edege 2
	r = fabs((d[2] * c[1][1]) - (d[1] * c[2][1]));
	r0 = (getScale()[1] * absC[2][1]) + (getScale()[2] * absC[1][1]);
	r1 = (scaleOBB[0] * absC[0][2]) + (scaleOBB[2] * absC[0][0]);
	
	if (r > r0 + r1)
		return false;


	//----------------------------------------------------------------------------------------------------------------	
	// edege 3
	r = fabs((d[2] * c[1][2]) - (d[1] * c[2][2]));
	r0 = (getScale()[1] * absC[2][2]) + (getScale()[2] * absC[1][2]);
	r1 = (scaleOBB[0] * absC[0][1]) + (scaleOBB[1] * absC[0][0]);
	
	if (r > r0 + r1)
		return false;


	//----------------------------------------------------------------------------------------------------------------
	// edege 4
	r = fabs((d[0] * c[2][0]) - (d[2] * c[0][0]));
	r0 = (getScale()[0] * absC[2][0]) + (getScale()[2] * absC[0][0]);
	r1 = (scaleOBB[1] * absC[1][2]) + (scaleOBB[2] * absC[1][1]);
	
	if (r > r0 + r1)
		return false;


	//----------------------------------------------------------------------------------------------------------------
	// edege 5
	r = fabs((d[0] * c[2][1]) - (d[2] * c[0][1]));
	r0 = (getScale()[0] * absC[2][1]) + (getScale()[2] * absC[0][1]);
	r1 = (scaleOBB[0] * absC[1][2]) + (scaleOBB[2] * absC[1][0]);
	
	if (r > r0 + r1)
		return false;


	//----------------------------------------------------------------------------------------------------------------
	// edege 6
	r = fabs((d[0] * c[2][2]) - (d[2] * c[0][2]));
	r0 = (getScale()[0] * absC[2][2]) + (getScale()[2] * absC[0][2]);
	r1 = (scaleOBB[0] * absC[1][1]) + (scaleOBB[1] * absC[1][0]);
	
	if (r > r0 + r1)
		return false;


	//----------------------------------------------------------------------------------------------------------------
	// edege 7
	r = fabs((d[1] * c[0][0]) - (d[0] * c[1][0]));
	r0 = (getScale()[0] * absC[1][0]) + (getScale()[1] * absC[0][0]);
	r1 = (scaleOBB[1] * absC[2][2]) + (scaleOBB[2] * absC[2][1]);
	
	if (r > r0 + r1)
		return false;


	//----------------------------------------------------------------------------------------------------------------	
	// edege 8
	r = fabs((d[1] * c[0][1]) - (d[0] * c[1][1]));
	r0 = (getScale()[0] * absC[1][1]) + (getScale()[1] * absC[0][1]);
	r1 = (scaleOBB[0] * absC[2][2]) + (scaleOBB[2] * absC[2][0]);
	
	if (r > r0 + r1)
		return false;

	//----------------------------------------------------------------------------------------------------------------
	// edege 9
	r = fabs((d[1] * c[0][2]) - (d[0] * c[1][2]));
	r0 = (getScale()[0] * absC[1][2]) + (getScale()[1] * absC[0][2]);
	r1 = (scaleOBB[0] * absC[2][1]) + (scaleOBB[1] * absC[2][0]);
	
	if (r > r0 + r1)
		return false;

	#ifdef TEST_ENVIRONMENT
		setColor(Graphics::yellow);
	#endif
	
	//return false;
	return true;
}



/**
* \brief liefert die Querschnittsfläche
*/
float thetaAngle(Vec3 p1, Vec3 p2);
int wrap(std::vector<Vec3 > &a, int n);
float getAngle(Vec3, Vec3);

float Box::getArea()
{
	float cw = 0.9f;
	Quaternion boxOrientation;
	Quaternion orgOrientation = mRigidBody->getOrientation();
	orgOrientation.normalize();
	
	Vec3 velocity = mRigidBody->getVelocity();

	velocity = qRotate(velocity, orgOrientation);
	
	float angle;
	Vec3 axis;
	
	if (velocity.length() > 0.01)	
		angle = acosf(dot(velocity, Vec3(0.0, 1.0, 0.0)) / velocity.length());
	else
		return 0.0;

	
	if (velocity[2] == 0)
		axis = Vec3(0.0, 0.0, 1.0);
	else if (velocity[0] == 0)
		axis = Vec3(1.0, 0.0, 0.0);
	else
		axis = Vec3(velocity[2] / velocity[0], 0.0, 1.0);
	
	
	boxOrientation.setValues(angle, axis);
	boxOrientation.normalize();
	
	velocity = qRotate(velocity, boxOrientation);
	
	vector<Vec3 > vertices(8);
	vertices[0] = mScale;
	vertices[1] = Vec3( mScale[0],  mScale[1], -mScale[2]);
	vertices[2] = Vec3(-mScale[0],  mScale[1], -mScale[2]);
	vertices[3] = Vec3(-mScale[0],  mScale[1],  mScale[2]);
	vertices[4] = Vec3( mScale[0], -mScale[1],  mScale[2]);
	vertices[5] = Vec3( mScale[0], -mScale[1], -mScale[2]);
	vertices[6] = Vec3(-mScale[0], -mScale[1], -mScale[2]);
	vertices[7] = Vec3(-mScale[0], -mScale[1],  mScale[2]);
	for (int i = 0; i < 8; i++)
	{
		vertices[i] = qRotate(vertices[i], boxOrientation);
		vertices[i][1] = 0;
	}
	
	int convexCnt = wrap(vertices, 8);
	
	int j;
	float sum = 0.0;
	
	for (int i = 0; i < convexCnt; i++)
	{
		j = i+1;
		if (j == convexCnt)
			j = 0;
		
		sum += ((sinf(getAngle(vertices[i], vertices[j])) * vertices[j].length()) * vertices[i].length()) / 2.f;
	}
	
	return cw * sum;
}


//Funktionen zur Berechnung der konvexen Huelle

/**
* \brief berechnet den Winkel zwischen zwei Vektoren
*/
float getAngle(Vec3 v1, Vec3 v2)
{
	float angle = 0.0;
	if (v1.length() != 0 && v2.length() != 0)	
		angle = acosf(dot(v1, v2) / (v1.length() * v2.length()));
	else
		angle = 0.0;
	
	return angle;
}
	
/**
* \brief berechnet den Winkel zwischen p1, p2 und der Horizontalen
*/
float thetaAngle(Vec3 p1, Vec3 p2)
{
	float dx, dy, ax, ay;
	float t;
	
	dx = p2[0] - p1[0];
	ax = fabs(dx);
	
	dy = p2[2] - p1[2];
	ay = fabs(dy);
	
	t = (ax + ay == 0) ? 0 : (float) dy / (ax + ay);
	
	if (dx < 0) 
		t = 2 - t;
	else if (dy < 0)
		t = 4 + t;
	
	return t * 90.f;
}

/**
* \brief berechnet die konvexe Huelle einer Punktemenge
* \return Anzahl der Punkte auf der konvexen Huelle
*/
int wrap(vector<Vec3 > &a, int n)
{
	int i, minx, minz, m;
	float th, v;
	
	minx = 0;
	minz = 0;
	for (i = 1; i < n; i++)
	{
		if (a[i][2] < a[minz][2]) 
			minz = i;
		else if (a[i][2] == a[minz][2])
			if (a[i][0] > a[minz][0])
				minz = i;
			
	}
	
	a[n] = a[minz];
	th = 0.0;
	
	for (m = 0; m < n; m++)
	{
		Vec3 tmp = a[m];
		a[m] = a[minz];
		a[minz] = tmp;
		
		minz = n;
		v = th;
		th = 360.0;
		for (i = m+1; i <= n; i++)
		{
			if (thetaAngle(a[m], a[i]) > v)
			{
				if (thetaAngle (a[m], a[i]) < th)
				{
					minz = i;
					th = thetaAngle(a[m], a[minz]);
				}
			}
		}
		if (minz == n)
			return m + 1;
	}

	assert(false);
}


void Box::setScale (Vec3 scale)  { 
	mScale = scale;
	hasChanged(true);
};    

void Box::setHeight(float height) { 
	mScale[Y] = height/2; 
	hasChanged(true);
};

void Box::setWidth (float width)  { 
	mScale[X] = width/2; 
	hasChanged(true);
};

void Box::setDepth (float depth)  { 
	mScale[Z] = depth/2; 
	hasChanged(true);
};
