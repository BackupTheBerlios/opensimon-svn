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
* \class Clip.h
* $Author: alangs $
* $Date: 2004/12/14 18:29:47 $
* $Revision: 1.5 $
* \brief Cohen Sutherland Clipping Gerade am Fenster
*/
//------------------------------------------------------------------------------

#ifndef CLIP_H
#define CLIP_H

#include <simon/Vector3.h>

class Clip
{
	static const int EMPTY = 0;
	static const int LEFT = 1;
	static const int RIGHT = 2;
	static const int BOTTOM = 4;
	static const int TOP = 8;

public:
	Clip(void);
	~Clip(void);
	static unsigned int clipLine(Vec3* window, Vec3* line, unsigned int ignoreAxis);
	static int region_code(unsigned int a, unsigned int b, 
			       Vec3* P, float txmin, 
			       float txmax, float tymin,float tymax);
	static unsigned int cohenSutherland(unsigned int a, unsigned int b, 
				    Vec3* P1, Vec3* P2, float txmin, 
				    float txmax, float tymin, float tymax);
};
#endif
