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
 * \file Pool.h
 * \class Pool
 * 
 */

#include "Pool.h"

using namespace std;

Pool::Pool (size_t size) {
	mSize = size;
}

Pool::~Pool() {
	
	unsigned int count = mBlocks.size();
	for (unsigned int i = 0; i < count; ++i)
		delete[] mBlocks[i];

}

void * Pool::alloc() {
	
	char* address;

	if (! mFree.empty()) {
		address = mFree.back();
		mFree.pop_back();
		return address;
	} 

	address = new char[mSize * M_BLOCK_SIZE];
	mBlocks.push_back(address);

	for (unsigned int i = 0; i < M_BLOCK_SIZE - 1; ++i, address += mSize)
		mFree.push_back(address);

	return address;
}

void Pool::free(void * ptr) {
	mFree.push_back(static_cast<char*>(ptr));
}
