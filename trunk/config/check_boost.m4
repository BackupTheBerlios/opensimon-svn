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
 


AC_DEFUN([SIMON_CHECK_BOOST],
	[
		SIMON_CHECK_BOOST_THREAD
		AC_CHECK_HEADER([boost/shared_ptr.hpp], 
			[],
			[
				AC_MSG_ERROR([Can't find boost/shared_ptr.hpp])
			]
		)

		AC_CHECK_HEADER([boost/graph/graph_utility.hpp], 
			[],
			[
				AC_MSG_ERROR([Can't find boost/graph/graph_utility.hpp])
			]
		)

		AC_CHECK_HEADER([boost/graph/graphviz.hpp], 
			[],
			[
				AC_MSG_ERROR([Can't find boost/graph/graphviz.hpp])
			]
		)

		AC_CHECK_HEADER([boost/graph/strong_components.hpp], 
			[],
			[
				AC_MSG_ERROR([Can't find boost/graph/strong_components.hpp])
			]
		)
		
	]
)

# checks boost_thread
# momentanly unused
AC_DEFUN([SIMON_CHECK_BOOST_THREAD],
	[
		AC_MSG_CHECKING([whether boost_thread library is available])
		boost_thread_avail=no
		boost_thread_save_libs=$LIBS
		boost_thread_save_flags=$CXXFLAGS
		LIBS="$LIBS -lboost_thread"
		CXXFLAGS="$CXXFLAGS -pthread"
		AC_LANG_PUSH(C++)
		AC_TRY_LINK(
			[#include <boost/thread.hpp>],
			[boost::thread();],
			[boost_thread_avail=yes]
		)
		AC_MSG_RESULT([$boost_thread_avail])
		AC_LANG_POP
		if test "${boost_thread_avail}" = no; then
			LIBS=$boost_thread_save_libs
			CXXFLAGS=$boost_thread_save_flags
		fi
	]
)

