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

#ifndef GLUTAPPLICATION_H
#define GLUTAPPLICATION_H


#include <map>


class GlutApplication
{
   public:
      static void init(int* argc, char** argv);
      static void run();


   private:
      friend class GlutWindow;

      typedef std::map<int, GlutWindow*> map_type;

      static map_type mWindows;

      static void addWindow(GlutWindow* window, int handle);
      static void removeWindow(int handle);

      static void display();
      static void reshape(int width, int height);
      static void idle();

      static void keyboard(unsigned char key, int x, int y);
      static void keyboardSpecial(int key, int x, int y);

      static void mouse(int button, int state, int x, int y);
      static void mouseEntry(int state);
      static void mouseMotion(int x, int y);
      static void mousePassiveMotion(int x, int y);
};


#endif   // !GLUT_APPLICATION_H
