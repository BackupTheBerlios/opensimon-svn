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
 * \file test-environment.h
 */
//------------------------------------------------------------------------------


#ifndef TEST_ENVIRONMENT_H
#define TEST_ENVIRONMENT_H

#include <simon/config.h>

#include <cstdlib>
#include <cstdio>
#include <GL/glut.h>
#include <GL/glu.h>
#include <cmath>

#include <iostream>
#include <fstream>


#include <simon/Particle.h>
#include <simon/Connection.h>
#include <simon/RigidBody.h>


#include <simon/SmartPointer.h>
#include <simon/Id.h>
#include <simon/Quaternion.h>
#include <simon/Matrix.h>
#include <simon/Vector3.h>
#include <simon/Clock.h>
#include <simon/Graphics.h>



#include <simon/SimonState.h>

#include <simon/ConstraintSystem.h>

#include <simon/RigidBodySystem.h>

#include <simon/ParticleSystem.h>
#include <simon/ClothSystem.h>

#include <simon/GeometrySystem.h>
#include <simon/Geometry.h>
#include <simon/Sphere.h>
#include <simon/Plane.h>
#include <simon/Box.h>



// Diese Methode wird immer wieder aufgerufen und soll
// von den Teams selbst implementiert werden
void displayLoop();

// Diese Methode wird beim Start einmal aufgerufen und kann
// von den Teams selbst implementiert werden
void initialize(int argc=0, char** argv=0);

// Diese Methode verarbeitet keybord events
void keyHandler(unsigned char key);

//! liefert die letzte dauer des display-Loops zurück
float getLastTime();

// Hier folgen standard Funktionen
void display();
void init();
void idle();
void clickTracker(int button, int state, int x, int y);
void motionTracker(int x, int y);
void keyboard(unsigned char key, int x, int y);
int main(int argc, char** argv);


#endif
