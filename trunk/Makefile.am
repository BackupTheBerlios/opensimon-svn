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
 

####
## Auto Makefile for libsimon and test programs
##
####

# append libtooltags (used for exaple by ccache autoconf script)
LIBTOOL = @LIBTOOL@ @SIMON_LIBTOOL_TAG@

AM_CXXFLAGS   = @SIMON_CXXFLAGS@

AM_YFLAGS = -d

if USE_SWIFT
SWIFT_PROGS = swiftTest

swiftTest_SOURCES = \
test/swiftTest.cpp \
test/test-environment.cpp test/test-environment.h

swiftTest_LDADD = libsimon.la

SWIFT_CLASSES = simon/SwiftCollisionDetection.cpp simon/SwiftCollisionDetection.h

else
SWIFT_PROGS = 
SWIFT_CLASSES = 
endif

if USE_OLD_TESTS

OLD_TEST_PROGS = \
test-collision-no-visual \
test-environment \
test-bodies \
test-bodies2 \
test-bodies3 \
test-quaternions \
test-cloth-hair \
test-cloth-hair-simon-wish \
test-collision \
test-constraints \
test-constraints2 \
test-constraints3 \
test-simon \
test-stress \
test-freefall

else
OLD_TEST_PROGS = 
endif


## list of programs which should be build
##simon \
bin_PROGRAMS = \
$(SWIFT_PROGS) \
$(OLD_TEST_PROGS) \
visualSimon 


lib_LTLIBRARIES     = libsimon.la
libsimon_la_SOURCES = \
\
simon/Particle.cpp simon/Particle.h \
simon/Connection.cpp simon/Connection.h \
simon/SpringConnection.cpp simon/SpringConnection.h \
simon/WorldObject.cpp simon/WorldObject.h \
simon/RigidBody.cpp simon/RigidBody.h \
simon/SimonState.cpp simon/SimonState.h \
simon/Clock.cpp simon/Clock.h \
simon/ContactInformationContainer.cpp simon/ContactInformationContainer.h \
simon/ContactPoint.cpp simon/ContactPoint.h \
simon/Matrix.cpp simon/Matrix.h \
simon/Vector3.cpp simon/Vector3.h \
simon/Quaternion.cpp simon/Quaternion.h \
simon/Id.cpp simon/Id.h \
simon/Pool.cpp simon/Pool.h \
\
simon/Geometry.cpp simon/Geometry.h \
simon/Sphere.cpp simon/Sphere.h \
simon/Plane.cpp simon/Plane.h \
simon/Box.cpp simon/Box.h \
simon/Capsule.cpp simon/Capsule.h \
simon/Clip.cpp simon/Clip.h \
simon/GeometrySystem.cpp simon/GeometrySystem.h \
simon/AnalyticCollisionDetection.cpp \
simon/Interference.cpp simon/Interference.h \
\
simon/PrimaryConstraint.cpp simon/PrimaryConstraint.h \
simon/BallAndSocketConstraint.cpp simon/BallAndSocketConstraint.h \
simon/HingeConstraint.cpp simon/HingeConstraint.h \
simon/ConstraintMatrixNode.cpp simon/ConstraintMatrixNode.h \
simon/ConstraintSystem.cpp simon/ConstraintSystem.h \
\
simon/ParticleSystem.cpp simon/ParitcleSystem.h \
simon/ClothSystem.cpp simon/ClothSystem.h \
\
simon/RigidBodySystem.cpp simon/RigidBodySystem.h \
\
$(SWIFT_CLASSES) \
\
simon/config.h simon/auto_config.h


######################### simon
## source files for the main prgram simon
#simon_SOURCES = server/Simon.cpp server/Simon.h
#simon_LDADD = libsimon.la


########################## test-environment
## source files for the standard test-environment 
test_environment_SOURCES = \
test/test-environment.cpp test/test-environment.h \
test/test-environment-own-stuff.cpp

### libs for the standard test-environment
test_environment_LDADD = libsimon.la

########################## test-bodies
### source files for the bodies playground
test_bodies_SOURCES = \
test/test-environment-bodies.cpp \
test/test-environment.cpp test/test-environment.h

test_bodies2_SOURCES = \
test/test-environment-bodies2.cpp \
test/test-environment.cpp test/test-environment.h

test_bodies3_SOURCES = \
test/test-environment-bodies3.cpp \
test/test-environment.cpp test/test-environment.h

test_quaternions_SOURCES = \
test/test-quaternions.cpp

### libs for the bodies playground
test_bodies_LDADD = libsimon.la
test_bodies2_LDADD = libsimon.la
test_bodies3_LDADD = libsimon.la
test_quaternions_LDADD = libsimon.la

########################## test-cloth-hair
### source files for the cloth-hair playground
test_cloth_hair_SOURCES = \
test/test-environment-cloth-hair.cpp \
test/test-environment.cpp test/test-environment.h

test_cloth_hair_simon_wish_SOURCES = \
test/test-environment-cloth-hair-like-simon-use-it.cpp \
test/test-environment.cpp test/test-environment.h

### libs for the cloth-hair playground
test_cloth_hair_LDADD = libsimon.la
test_cloth_hair_simon_wish_LDADD = libsimon.la

########################## test-collision
### source files for the collison playground
test_collision_SOURCES = \
test/test-environment-collision.cpp \
test/test-environment.cpp test/test-environment.h

### libs for the collisions playground
test_collision_LDADD = libsimon.la

########################## test-collision-no-visual
### source files for the collison playground
test_collision_no_visual_SOURCES = \
test/profile-collisions.cpp

### libs for the collisions playground
test_collision_no_visual_LDADD = libsimon.la

########################## test-simon
### source files for simons playground 
test_simon_SOURCES = \
test/test-environment-simon.cpp \
test/test-environment.cpp test/test-environment.h

#test_network_SOURCES = \
#test/test-environment-network.cpp \
#test/test-environment.cpp test/test-environment.h

test_stress_SOURCES = \
test/test-environment-stress.cpp \
test/test-environment.cpp test/test-environment.h

test_freefall_SOURCES = \
test/test-environment-freefall.cpp \
test/test-environment.cpp test/test-environment.h

### libs for simons playground
test_simon_LDADD = libsimon.la
#test_network_LDADD = libsimon.la
test_stress_LDADD = libsimon.la
test_freefall_LDADD = libsimon.la

########################## test-constraints1-3
### source files for the constraints playgrounds
test_constraints_SOURCES = \
test/test-environment-constraints.cpp \
test/test-environment.cpp test/test-environment.h

test_constraints2_SOURCES = \
test/test-environment-constraints2.cpp \
test/test-environment.cpp test/test-environment.h

test_constraints3_SOURCES = \
test/test-environment-constraints3.cpp \
test/test-environment.cpp test/test-environment.h

### libs for the constrants playgrounds
test_constraints_LDADD = libsimon.la
test_constraints2_LDADD = libsimon.la
test_constraints3_LDADD = libsimon.la

########################## visualSimon
### source files for the visual testing application
visualSimon_SOURCES = \
test/VisualSimon.h test/VisualSimon.cpp \
test/VSSParser.yy test/VSSScanner.ll \
test/OpenGl.h \
test/GlutApplication.cpp test/GlutApplication.h \
test/GlutWindow.cpp test/GlutWindow.h \
test/ObjectDescription.h test/ObjectDescription.cpp \
test/SphereDescription.h test/SphereDescription.cpp \
test/BoxDescription.h test/BoxDescription.cpp \
test/PlaneDescription.h test/PlaneDescription.cpp \
test/CapsuleDescription.h test/CapsuleDescription.cpp \
test/SceneDescription.h test/SceneDescription.cpp\
\
test/GraphicsBase.cpp test/GraphicsBase.h \
test/GraphicsGL.cpp test/GraphicsGL.h \
test/GraphicsRIB.cpp test/GraphicsRIB.h


visualSimon_LDADD = libsimon.la


### Install header files into ${prefix}/include/simon/
nobase_include_HEADERS = \
simon/simon.h \
simon/BallAndSocketConstraint.h \
simon/Box.h \
simon/CallbackAdapter.h \
simon/Capsule.h \
simon/Clip.h \
simon/Clock.h \
simon/Cloth-Hair.h \
simon/ClothSystem.h \
simon/GeometrySystem.h \
simon/CollisionDetection.h \
simon/AnalyticCollisionDetection.h \
simon/config.h \
simon/Connection.h \
simon/ConstraintMatrixNode.h \
simon/ConstraintSystem.h \
simon/ContactInformationContainer.h \
simon/ContactPoint.h \
simon/Geometry.h \
simon/HingeConstraint.h \
simon/Id.h \
simon/Interference.h \
simon/Matrix.h \
simon/Particle.h \
simon/ParticleSystem.h \
simon/Plane.h \
simon/Pool.h \
simon/PrimaryConstraint.h \
simon/Quaternion.h \
simon/RigidBody.h \
simon/RigidBodySystem.h \
simon/SimonState.h \
simon/SmartPointer.h \
simon/Sphere.h \
simon/SpringConnection.h \
simon/Vector3.h \
simon/WorldObject.h
