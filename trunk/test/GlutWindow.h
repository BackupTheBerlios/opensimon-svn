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

#ifndef GLUTWINDOW_H
#define GLUTWINDOW_H


#include "OpenGL.h"


class GlutWindow
{
   public:
      GlutWindow(const char* title, 
				 unsigned int mode = GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

      virtual ~GlutWindow();

      virtual void display() = 0;
      virtual void reshape(int width, int height);
	  virtual void idle();

      virtual bool keyboard(unsigned char key, int x, int y);
      virtual void keyboardSpecial(int key, int x, int y);

      virtual void mouse(int button, int state, int x, int y);
      virtual void mouseEntry(int state);
      virtual void mouseMotion(int x, int y);
      virtual void mousePassiveMotion(int x, int y);

	  virtual void setSize(unsigned int width, unsigned int height);

   private:
      int mHandle;
};


#endif   // !GLUTWINDOW_H
