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


#include "GlutApplication.h"
#include "GlutWindow.h"
#include "OpenGL.h"
#include <cstdlib>

using namespace std;


GlutApplication::map_type GlutApplication::mWindows;


void GlutApplication::init(int* argc, char** argv)
{
   glutInit(argc, argv);
}


void GlutApplication::run()
{
   glutMainLoop();
}

void GlutApplication::addWindow(GlutWindow* window, int handle)
{
   mWindows.insert(map_type::value_type(handle, window));

   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutIdleFunc(idle);
   glutKeyboardFunc(keyboard);
   glutSpecialFunc(keyboardSpecial);
   glutMouseFunc(mouse);
   glutEntryFunc(mouseEntry);
   glutMotionFunc(mouseMotion);
   glutPassiveMotionFunc(mousePassiveMotion);
}


void GlutApplication::removeWindow(int handle)
{
   map_type::iterator item = mWindows.find(handle);
   mWindows.erase(item);
}


void GlutApplication::display()
{
   map_type::iterator item = mWindows.find(glutGetWindow());
   item->second->display();
}


void GlutApplication::reshape(int width, int height)
{
   map_type::iterator item = mWindows.find(glutGetWindow());
   item->second->reshape(width, height);
}

void GlutApplication::idle()
{
	map_type::iterator item = mWindows.find(glutGetWindow());
	item->second->idle();
}

void GlutApplication::keyboard(unsigned char key, int x, int y)
{
   map_type::iterator item = mWindows.find(glutGetWindow());
   bool eventHandled = item->second->keyboard(key, x, y);

   if (!eventHandled && key == 27)
      exit(0);
}


void GlutApplication::keyboardSpecial(int key, int x, int y)
{
   map_type::iterator item = mWindows.find(glutGetWindow());
   item->second->keyboardSpecial(key, x, y);
}


void GlutApplication::mouse(int button, int state, int x, int y)
{
   map_type::iterator item = mWindows.find(glutGetWindow());
   item->second->mouse(button, state, x, y);
}


void GlutApplication::mouseEntry(int state)
{
   map_type::iterator item = mWindows.find(glutGetWindow());
   item->second->mouseEntry(state);
}


void GlutApplication::mouseMotion(int x, int y)
{
   map_type::iterator item = mWindows.find(glutGetWindow());
   item->second->mouseMotion(x, y);
}


void GlutApplication::mousePassiveMotion(int x, int y)
{
   map_type::iterator item = mWindows.find(glutGetWindow());
   item->second->mousePassiveMotion(x, y);
}
