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
 



AC_DEFUN([SIMON_CHECK_SWIFT],
	[
		AC_MSG_CHECKING([whether SWIFT library is available])
		swift_avail=no
		swift_save_libs=$LIBS
		LIBS="$LIBS -lSWIFT"
		AC_LANG_PUSH(C++)
		AC_TRY_LINK(
			[#include <SWIFT.h>],
			[SWIFT_Scene s(true,false);],
			[swift_avail=yes]
		)
		AC_MSG_RESULT([$swift_avail])
		AC_LANG_POP
		if test "${swift_avail}" = no; then
			LIBS=$swift_save_libs
		else
			AC_DEFINE(DHAVE_SWIFT, 1,
				[Controles the use of the SWIFT library])
		fi
		COMPILE_SWIFT=${swift_avail}
	]
)
