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
 

AC_DEFUN([SIMON_ENABLE_OLD_TESTS],
	[
		AC_MSG_CHECKING([whether old test programs should be build])
		AC_ARG_ENABLE(old-tests,
			[AC_HELP_STRING([--enable-old-tests],
			[build old test programs])],
			[
				case "${enableval}" in
					yes|no) ;;
					*) AC_MSG_ERROR([bad value ${enableval} for old-test option]) ;;
				esac
			],
			[enableval=no]
		)
		AC_MSG_RESULT($enableval)

		COMPILE_OLD_TESTS=${enableval}
	]
)
