## Simon is the legal property of its developers, whose names are too
## numerous to list here.  Please refer to the COPYRIGHT file
## distributed with this source distribution.
##
## This library is free software; you can redistribute it and/or
## modify it under the terms of the GNU Lesser General Public
## License as published by the Free Software Foundation; either
## version 2.1 of the License, or (at your option) any later version.
## 
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
## Lesser General Public License for more details.
## 
## You should have received a copy of the GNU Lesser General Public
## License along with this library; if not, write to the Free Software
## Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 


AC_DEFUN([SIMON_CHECK_OPENGL],
	[
		AC_CHECK_LIB([GL], [glTranslatef])
		AC_CHECK_LIB([GLU], [gluLookAt])
		AC_CHECK_LIB([glut], [glutSolidSphere])
		AC_CHECK_LIB([freeglut], [glutSolidSphere])

		AC_CHECK_HEADER(
			[GL/freeglut.h], 
			[
				AC_DEFINE(HAVE_GL_FREEGLUT_H, 1,
					[Tells weather GL/freeglut.h is available or not]
				)
			]
		)
	]
)
