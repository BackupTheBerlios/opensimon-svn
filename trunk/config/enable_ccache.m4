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
 


AC_DEFUN([SIMON_ENABLE_CCACHE],
	[
		AC_MSG_CHECKING([whether ccache acceleration is requested])
		AC_ARG_ENABLE(ccache,
        	[AC_HELP_STRING([--enable-ccache],
            [Uses ccache compile acceleration])],
        	[
            	case "${enableval}" in
                  	yes|no) ;;
                	*) AC_MSG_ERROR([bad value ${enableval} for ccache option]) ;;
	   			esac
         	],
   			[enableval=no]
		)
		AC_MSG_RESULT($enableval)

		if test "${enableval}" = yes; then
			# check for ccache in $PATH
			AC_CHECK_PROG([PROG_CCACHE], [ccache], [yes])
			if test "$PROG_CCACHE" = yes; then
				SIMON_LIBTOOL_TAG="--tag=CXX"
				CXX="ccache $CXX"
				CC="ccache $CC"
			else
				AC_MSG_ERROR([can not find ccache, acceleration not enabled])		
			fi
		fi
	]
)
