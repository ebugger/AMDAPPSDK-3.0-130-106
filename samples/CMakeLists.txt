#################################################################################
# Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# •	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# •	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
#  other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#################################################################################


set( FOLDER_NAME samples )
set( SUBDIRECTORIES "")

option( BUILD_OPENCL "BUILD_OPENCL" ON)
option( BUILD_OPENCV "BUILD_OPENCV" ON)

if( WIN32)
	option( BUILD_AMP "BUILD_AMP" ON)
endif()

option( BUILD_BOLT "BUILD_BOLT" ON)

if (BUILD_OPENCL MATCHES ON)
	set ( SUBDIRECTORIES ${SUBDIRECTORIES} opencl )
 endif()
 
if (BUILD_OPENCV MATCHES ON)
	set ( SUBDIRECTORIES ${SUBDIRECTORIES} opencv )
endif()
 if (BUILD_BOLT MATCHES ON)
	set ( SUBDIRECTORIES ${SUBDIRECTORIES} bolt )
 endif()
 
if (BUILD_AMP MATCHES ON)
	set( SUBDIRECTORIES_WIN C++Amp )
endif()


############################################################################

set(CMAKE_SUPPRESS_REGENERATION TRUE)
cmake_minimum_required( VERSION 2.8.0 )
project( ${FOLDER_NAME} )

# Group samples by folder
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set( FOLDER_GROUP ${FOLDER_GROUP}/${FOLDER_NAME} )

if( WIN32)
	set( SUBDIRECTORIES  ${SUBDIRECTORIES} ${SUBDIRECTORIES_WIN})
	set( SUBDIRECTORIES_WIN "")
endif()

foreach( subdir ${SUBDIRECTORIES} )
	if( IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${subdir}" )
		MESSAGE( STATUS "Found sample ${subdir}, adding it" )
		add_subdirectory( ${subdir} )
	endif()
endforeach( subdir )