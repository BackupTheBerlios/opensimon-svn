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
 * \file GraphicsGL.cpp
 * \class GraphicsGL
 *
 */
//------------------------------------------------------------------------------

#include "GraphicsGL.h"
#include "Quaternion.h"
#include "ContactPoint.h"

#include <iostream>

using namespace std;

const GLfloat GraphicsGL::red[] = {1, 0, 0};
const GLfloat GraphicsGL::green[] = {0, 1, 0};
const GLfloat GraphicsGL::blue[] = {0, 0, 1};
const GLfloat GraphicsGL::grey[] = {0.5f, 0.5f, 0.5f};
const GLfloat GraphicsGL::magenta[] = {1, 0, 1};
const GLfloat GraphicsGL::yellow[] = {1, 1, 0};


GraphicsGL::GraphicsGL(){

	glClearColor (0.5, 0.5, 0.6, 0.0); //Hintergrundfarbe
	glEnable(GL_DEPTH_TEST);//Tiefentest einschalten
	glEnable(GL_NORMALIZE);//Automatisches normalisieren einschalten
	glEnable(GL_LIGHTING);//Licht einschalten
	glEnable(GL_LIGHT0);//Lichtquelle 0 einschalten
	glEnable(GL_LIGHT1);//Lichtquelle 1 einschalten

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45,1,1,10000);

	// Farben und Formen
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(1);

	mLightPosition1[0] = mLightPosition1[1] = 0.5;
	mLightPosition1[2] = mLightPosition1[3] = 0.0;
	mLightPosition2[0] = mLightPosition2[1] = 0.0;
	mLightPosition2[2] = mLightPosition2[3] = 0.0;
}

GraphicsGL::~GraphicsGL(){
	
}


void GraphicsGL::beginFrame(float camX, float camY, float camZ, 
							float coiX, float coiY, float coiZ,
							float upX, float upY, float upZ){

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);

	// look at (0,0,-1).
	glLoadIdentity();

	gluLookAt(camX, camY, camZ,
			  coiX, coiY, coiZ,
			  upX, upY, upZ);

	glLightfv(GL_LIGHT0, GL_POSITION, mLightPosition1);//Lichtquelle definieren
	glLightfv(GL_LIGHT1, GL_POSITION, mLightPosition2);//Lichtquelle definieren

    GLfloat color[3] = { 0.9, 0.8, 0.9};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, color);//Lichtquelle definieren
	glLightfv(GL_LIGHT1, GL_DIFFUSE, color);//Lichtquelle definieren

	glLightfv(GL_LIGHT0, GL_AMBIENT, color);//Lichtquelle definieren
	glLightfv(GL_LIGHT1, GL_AMBIENT,color);//Lichtquelle definieren

	if (useWireframe()){ //Falls mWireframe = true zeige Gitternetz an
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);//MWireframe eingeschaltet
	} else { 
		glPolygonMode(GL_FRONT, GL_FILL);//Polygone werden gefüllt
		glPolygonMode(GL_BACK, GL_LINE);
	}
}

void GraphicsGL::finishFrame(){

 	glutSwapBuffers();
}

const GLfloat* GraphicsGL::getColor(Material material){

	switch (material){
	case materialStone :
		return grey;
	default:
		return red;
	}
}

void GraphicsGL::drawLine(
	const Vec3& from, 
	const Vec3& to, 
	const Material material){ 

}
		
void GraphicsGL::drawBox(
	const Vec3& position,
	const Vec3& scale,
	const Material material){  

	Quaternion quaternion(0,Vec3(0,1,0));
	drawBox(position,
			scale,
			quaternion,
			material);
}

void GraphicsGL::drawBox(
	const Vec3& position,
	const Vec3& scale,
	const Quaternion& orientation,
	const Material material){  

	const GLfloat* color = getColor(material);

	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glPushMatrix();
	glTranslatef(position[X], position[Y], position[Z]);
	Vec3 axis;
	float angle;
	orientation.getAxisAngle(axis, angle);
	glRotated(RADTODEG(angle), axis[X], axis[Y], axis[Z]);
	glScalef(scale[X], scale[Y], scale[Z]);			
	glutSolidCube(2);
	glPopMatrix();
}

void GraphicsGL::drawPlane(
	const Vec3& position,
	const Quaternion& orientation, 
	const Material material,
	float size){  

	glPushMatrix();   
	glTranslatef( position[X], position[Y], position[Z]);
	Vec3 axis;
	float angle;
	orientation.getAxisAngle(axis, angle);
	glRotated(RADTODEG(angle), axis[X], axis[Y], axis[Z]);

	const GLfloat* color = getColor(material);

	bool flipFlag = false;
	float flipingColor[3] = {color[0], color[1], color[2]};

	glBegin(GL_QUADS);
	for (int i = -(int)size/2; i < (int)size/2; i += 100) {
		for (int j = -(int)size/2; j < (int) size/2; j += 100) {
			if (flipFlag) {
				flipFlag = false;
				flipingColor[0] += 0.1f;
				flipingColor[1] += 0.1f;
				flipingColor[2] += 0.1f;
			}
			else {
				flipFlag = true;
				flipingColor[0] -= 0.1f;
				flipingColor[1] -= 0.1f;
				flipingColor[2] -= 0.1f;
			}
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, flipingColor);
			glNormal3f(0,1,0);
			glVertex3f((float) i, 0, (float) j);
			glVertex3f((float) i, 0, (float) j+100);
			glVertex3f((float) i+100, 0, (float) j+100);
			glVertex3f((float) i+100, 0, (float) j);
		}
	}
	glEnd();

	// draw a normal
 	glBegin(GL_LINES);
	glVertex3f( 0, 0, 0);
	glVertex3f( 0, 0, size/10.0f);
	glEnd();
	glPopMatrix();
 
}  

	void GraphicsGL::drawPlane(
		const Vec3& position,
		const Material material){  

	Quaternion quaternion(0,Vec3(0,1,0));
	drawPlane(position, quaternion, material);
}

void GraphicsGL::drawSphere(
	const Vec3& position, 
	float radius, 
	const Quaternion& orientation,
	const Material material){  

	const GLfloat* color = getColor(material);

	glPushMatrix();
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glTranslatef(position[0], position[1], position[2]);
	Vec3 axis;
	float angle;
	orientation.getAxisAngle(axis, angle);
	glRotated(RADTODEG(angle), axis[X], axis[Y], axis[Z]);
	glutSolidSphere(radius, 10, 10);
	glPopMatrix();
}

	void GraphicsGL::drawSphere(
		const Vec3& position, 
	    float radius, 
		const Material material){ 

	Quaternion quaternion(0,Vec3(0,1,0));
	drawSphere(position, radius, quaternion, material);
}

void GraphicsGL::drawCapsule(
	const Vec3& position, 
	float radius, 
	float height,
	const Quaternion& orientation,
	const Material material){  

	const GLfloat* color = getColor(material);

	glPushMatrix();
	glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, color );
	glTranslatef(position[X], position[Y], position[Z]);    
	Vec3 axis;
	float angle;
	orientation.getAxisAngle(axis, angle);
	glRotated(RADTODEG(angle), axis[X], axis[Y], axis[Z]);
	glRotatef(90, 1,0,0);
	GLUquadric* quad;
	quad = gluNewQuadric();
	gluNewQuadric();
	gluQuadricDrawStyle(quad, GLU_FILL); //solid
	//gluQuadricDrawStyle(qobj, GLU_LINE); //wireframe 
	glPushMatrix();
	glTranslatef(0, 0, -height/2);  
	gluCylinder(quad, radius, radius, height, 10, 10);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(0, 0, height/2);  
	glutSolidSphere(radius, 10, 10);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(0,0, -height/2);    
	glutSolidSphere(radius, 10, 10);
	glPopMatrix();
    glPopMatrix();

}

void GraphicsGL::drawCapsule(
	const Vec3& position, 
	float radius, 
	float height,
	const Material material){  

	Quaternion quaternion(0,Vec3(0,1,0));
	drawCapsule(position, radius, height, quaternion, material);
}
							
void GraphicsGL::drawContactPoint(
	ContactPoint& contactPoint,
	const Material material){  

}
