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
 


AC_PREREQ(2.57)
AC_INIT([Simon], [0.18], [trappe@uni-koblenz.de])

AC_CONFIG_AUX_DIR(config)
AM_INIT_AUTOMAKE([foreign subdir-objects])
AM_CONFIG_HEADER(simon/auto_config.h:config/auto_config.h.in)

# directory tests
AC_CONFIG_SRCDIR([simon/config.h])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_LIBTOOL
AM_PROG_LEX
AC_PROG_YACC

# if we are at the University of Koblenz, use a special path
SIMON_CHECK_UNI

# This variable is used for extra flags in the Makefiles
AC_SUBST(SIMON_CXXFLAGS)
AC_SUBST(SIMON_LIBTOOL_TAG)


# Tell Autotools to use c++
AC_LANG([C++])

# Checks for libraries.
SIMON_CHECK_OPENGL
SIMON_CHECK_BOOST
AC_CHECK_LIB([jpeg], [main])
AC_CHECK_LIB([m], [sin])

# Checks for header files.
AC_CHECK_HEADER([unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_CONST

# Checks for library functions.
AC_CHECK_FUNCS([sqrt])


# checks the enable switches

SIMON_ENABLE_OLD_TESTS
AM_CONDITIONAL(USE_OLD_TESTS, test "${COMPILE_OLD_TESTS}" = yes)

SIMON_ENABLE_DEBUG
SIMON_ENABLE_ALL_WARNINGS
SIMON_ENABLE_RELEASE

SIMON_WITH_SWIFT     # maybe an extra path to SWIFT?
SIMON_ENABLE_SWIFT
AM_CONDITIONAL(USE_SWIFT, test "${COMPILE_SWIFT}" = yes)

SIMON_ENABLE_PROFILING
SIMON_ENABLE_CCACHE

AC_CONFIG_FILES([Makefile])

AC_OUTPUT


