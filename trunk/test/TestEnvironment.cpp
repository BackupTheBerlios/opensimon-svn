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


// $Id: TestEnvironment.cpp,v 1.6 2004/09/03 09:06:12 ritschel Exp $
//------------------------------------------------------------------------------
/**
*  \class TestEnvironment
*  \author Ritschel
*
* CVS:
*   - $Author: ritschel $
*   - $Date: 2004/09/03 09:06:12 $
*   - $Revision: 1.6 $
*
*  \brief Establishes connection to remote components
*
*  
*
*/
//------------------------------------------------------------------------------

#include "../../config.h"
#include "TestEnvironment.h"

#ifdef OS_WINDOWS

TestEnvironment::TestEnvironment(
	const InitializeFunctionPointer initialize,
	const DisplayLoopFunctionPointer displayLoop,
	const KeyHandlerFunctionPointer keyHandler) :
	mInitialize(initialize),
	mDisplayLoop(displayLoop),
	mKeyHandler(keyHandler)
{
};

TestEnvironment::InitializeFunctionPointer TestEnvironment::getInitialize() const {
	return mInitialize;
}

TestEnvironment::DisplayLoopFunctionPointer TestEnvironment::getDisplayLoop() const {
	return mDisplayLoop;
}

TestEnvironment::KeyHandlerFunctionPointer TestEnvironment::getKeyHandler() const {
	return mKeyHandler;
}
#endif
