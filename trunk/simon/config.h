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


// If compiler and/or platform specifics are needed
// this file hast to be included and all specific includes and defines
// have to be defined here

// Linux ----------------------------------------------------------------------
#if HAVE_CONFIG_H
#	include "auto_config.h"
// this is a hack. Hopefully the autotools will define it later.
#	define OS_LINUX
#	define ISNAN isnan
#	define ISINF isinf

#elif _MSC_VER
// Visual Studio, alle Versionen -----------------------------------------------
#	define OS_WINDOWS

//Nimmt aus der windows.h nur die notwendigsten includes
#   ifndef WIN32_LEAN_AND_MEAN
#       define WIN32_LEAN_AND_MEAN
#   endif

// Unter Visual Studio kann es zu Kollisionen zwischen std-min() und max() und
// der STL kommen. Wieso das nur vereinzelt auftritt, ist mir bis jetzt nicht
// klar. Abhilfe: STL ohne Makros min und max bestellen.
// Abhilfe (besser): class Tools, Vector3, anderes(?) so umstellen das sie immer
// STL nutzen und nirgendwo std includen (wozu auch, printf(), naklar ..).
#	ifndef NOMINMAX
#		define NOMINMAX
#	endif

#   include <windows.h>

#	define ISNAN _isnan
#	define ISINF !_finite
#else
// GCC unter Windows -----------------------------------------------------------
#   define OS_WINDOWS
//#	define USE_EAX // EAX nur unter Windows
#   define ISNAN _isnan
#endif

// Damit es ANSI Standard konform ist und alle das selbe PI nutzen.(siehe auch Wiki):
#ifdef M_PI
#	undef M_PI
#endif
#define M_PI (3.1415926535897932384626433832795)

#ifndef MAX
#   define MAX(a,b) a > b ? a : b
#endif
