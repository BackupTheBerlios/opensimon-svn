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



#ifndef POOL_H
#define POOL_H

#include <vector>
#include <cstddef>

/**
 * \file Pool.h
 * \class Pool
 * 
 * \brief The Pool provides a speed optimized way to store Objects.
 */
class Pool {

public:
	
	Pool (size_t size);

	~Pool();

	void * alloc();

	void free(void * ptr);

private:

	static const size_t M_BLOCK_SIZE = 1000;

	size_t mSize;
	
	std::vector<char*> mBlocks;
	std::vector<char*> mFree; 

};

#endif

