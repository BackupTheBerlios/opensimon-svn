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
 * \file test-environment.h
 * \author trappe
 */

#ifndef VISUAL_SIMON_H
#define VISUAL_SIMON_H

#include <cstdlib>
#include <cstdio>
#include <cmath>

#include <iostream>
#include <fstream>

#include <simon/simon.h>

#include "GraphicsGL.h"
#include "SceneDescription.h"

#include "GlutWindow.h"

class VisualSimon : public GlutWindow{

public:
	VisualSimon(const char* fileName);
	~VisualSimon();
 
	virtual void display();
	virtual bool keyboard(unsigned char key, int x, int y);
	virtual void mouse(int button, int /*state*/, int x, int y);
	virtual void mouseMotion(int x, int y);

private:

	//! contact Visualisation
	void drawContacts(ContactInformationContainerPtr);
	
	//! read in the scene description
	void processSceneDescription();

	GraphicsBase* mGraphicContext;
	GraphicsBase* mRIBContext;

	// save the mousestate
	bool mLeftButton;
	bool mRightButton;
	bool mMiddleButton;

	// saves the old mouspositions
	GLint mOldX;
	GLint mOldY;

	// camera position on a sphere
	GLdouble mTheta;
	GLdouble mPhi;

	// cam radius
	GLdouble mViewRadius;
	GLdouble mZoomSensibility;

	// mouse sensibility
	GLdouble mMouseSensibilityX;
	GLdouble mMouseSensibilityY;

	GLfloat mTranslateX;
	GLfloat mTranslateY;
};

int main(int argc, char** argv);


#endif
