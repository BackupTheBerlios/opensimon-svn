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


// $ID $
//------------------------------------------------------------------------------
/**
*  \file TestEnvironment.h
*  \class TestEnvironment
*  $Author: alangs $
*  $Date: 2004/09/18 20:04:55 $
*  $Revision: 1.14 $
*/
//------------------------------------------------------------------------------

#ifdef OS_WINDOWS

// Klasse, die Zwei Funktionspointer auf eine Display, eine Init-Funktion
// und einen Keyboard-Handler kapselt.

class TestEnvironment {
public:
	typedef void (*InitializeFunctionPointer)(int, char**);
	typedef void (*DisplayLoopFunctionPointer)(void);
	typedef void (*KeyHandlerFunctionPointer)(const unsigned char key);

	TestEnvironment(
		const InitializeFunctionPointer initialize,
		const DisplayLoopFunctionPointer displayLoop,
		const KeyHandlerFunctionPointer keyHandler);
	InitializeFunctionPointer getInitialize() const;
	DisplayLoopFunctionPointer getDisplayLoop() const;
	KeyHandlerFunctionPointer getKeyHandler() const;
private:
	const InitializeFunctionPointer mInitialize;
	const DisplayLoopFunctionPointer mDisplayLoop;
	const KeyHandlerFunctionPointer mKeyHandler;
};

// Instanzen dieser Klasse, der Einfachheit halber für alle verfügbar
extern const TestEnvironment testEnvironmentStress;
extern const TestEnvironment testEnvironmentFreefall;
extern const TestEnvironment testEnvironmentNetwork;
extern const TestEnvironment testEnvironmentSimon;
extern const TestEnvironment testEnvironmentBodies;
extern const TestEnvironment testEnvironmentBodies2;
extern const TestEnvironment testEnvironmentBodies3;
extern const TestEnvironment testEnvironmentClothSimonWish;
extern const TestEnvironment testEnvironmentCollision;
extern const TestEnvironment testEnvironmentConstraints;
extern const TestEnvironment testEnvironmentConstraints2;
extern const TestEnvironment testEnvironmentConstraints3;
extern const TestEnvironment testEnvironmentCloth;

#define TEFUNC static
#endif
#ifdef OS_LINUX
#define TEFUNC
#endif
