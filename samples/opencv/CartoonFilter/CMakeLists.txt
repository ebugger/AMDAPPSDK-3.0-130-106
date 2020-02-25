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


set( SAMPLE_NAME CartoonFilter )
set( SOURCE_FILES CartoonFilter.cpp )
set( EXTRA_FILES CartoonFilter_Kernels.cl CartoonFilter_Input.jpg )

############################################################################

set(CMAKE_SUPPRESS_REGENERATION TRUE)
cmake_minimum_required( VERSION 2.8.0 )
project( ${SAMPLE_NAME} )

if(CMAKE_BUILD_TYPE MATCHES "[Tt][Bb][Bb]")
	return( )
endif()

# Auto-select bitness based on platform
if( NOT BITNESS )
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(BITNESS 64)
    else()
        set(BITNESS 32)
    endif()
endif()

# Select bitness for non-msvc platform. Can be specified as -DBITNESS=32/64 at command-line
if( NOT MSVC )
    set(BITNESS ${BITNESS} CACHE STRING "Specify bitness")
    set_property(CACHE BITNESS PROPERTY STRINGS "64" "32")
endif()
# Unset OPENCL_LIBRARIES, so that corresponding arch specific libs are found when bitness is changed
unset(OPENCL_LIBRARIES CACHE)

if( BITNESS EQUAL 64 )
    set(BITNESS_SUFFIX x86_64)
elseif( BITNESS EQUAL 32 )
    set(BITNESS_SUFFIX x86)
else()
    message( FATAL_ERROR "Bitness specified is invalid" )
endif()

# Set CMAKE_BUILD_TYPE (default = Release)
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
	set(CMAKE_BUILD_TYPE Release)
endif()

# Set platform
if( NOT UNIX )
	set(PLATFORM win)
else()
	set(PLATFORM lnx)
endif()
#############################################################################
# Find OpenCL include and libs
find_path( OPENCL_INCLUDE_DIRS 
    NAMES OpenCL/cl.h CL/cl.h
    HINTS ../../../include/ $ENV{AMDAPPSDKROOT}/include/
)
mark_as_advanced(OPENCL_INCLUDE_DIRS)

find_library( OPENCL_LIBRARIES
	NAMES OpenCL
	HINTS ../../../lib/ $ENV{AMDAPPSDKROOT}/lib/
	PATH_SUFFIXES ${PLATFORM}${BITNESS} ${BITNESS_SUFFIX}
)
mark_as_advanced( OPENCL_LIBRARIES )

if( OPENCL_INCLUDE_DIRS STREQUAL "" OR OPENCL_LIBRARIES STREQUAL "")
	message( FATAL_ERROR "Could not locate OpenCL include & libs" )
endif( )

############################################################################
# Find OpenCV include and libs
set( OpenCV_DIR $ENV{OPENCV_DIR} )

if( NOT EXISTS "${OpenCV_DIR}")
message (FATAL_ERROR "Please specify OpenCV directory using OPENCV_DIR env. variable  " )
endif()

if ( NOT MSVC )
	set (OpenCV_DIR_LIB  "${OpenCV_DIR}/lib" )
else()
	if ( BITNESS EQUAL 32)
		set (OpenCV_DIR_LIB  "${OpenCV_DIR}/x86/" )
	else()
	    set (OpenCV_DIR_LIB  "${OpenCV_DIR}/x64/" )
	endif()
		
    if ( MSVC_VERSION EQUAL 1600)
		set (OpenCV_DIR_LIB  "${OpenCV_DIR_LIB}/vc10/lib" )
	elseif ( MSVC_VERSION EQUAL 1700)
		set (OpenCV_DIR_LIB  "${OpenCV_DIR_LIB}/vc11/lib" )
	elseif ( MSVC_VERSION EQUAL 1800)
		set (OpenCV_DIR_LIB  "${OpenCV_DIR_LIB}/vc12/lib" )
	endif ()	    
endif()

find_path(OpenCV_INCLUDE_DIRS1 "cv.h"
          PATHS "${OpenCV_DIR}"
          PATH_SUFFIXES "include" "include/opencv"
		 )
		  
mark_as_advanced(OpenCV_INCLUDE_DIRS1)

find_path(OpenCV_INCLUDE_DIRS2 "core.hpp"
          PATHS "${OpenCV_DIR}"
          PATH_SUFFIXES "include" "include/opencv2/" "include/opencv2/core"
		 )
		  
mark_as_advanced(OpenCV_INCLUDE_DIRS2)

find_path(OpenCV_INCLUDE_DIRS3 "opencv"
          PATHS "${OpenCV_DIR}/include"
         )
 
mark_as_advanced(OpenCV_INCLUDE_DIRS3)
        
find_path(OpenCV_INCLUDE_DIRS4 "opencv.hpp"
          PATHS "${OpenCV_DIR}"
          PATH_SUFFIXES "include" "include/opencv2/" "include/opencv2/core"
		 )
		  
mark_as_advanced(OpenCV_INCLUDE_DIRS4)

set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS1} ${OpenCV_INCLUDE_DIRS2} ${OpenCV_INCLUDE_DIRS3} ${OpenCV_INCLUDE_DIRS4})

if( OpenCV_INCLUDE_DIRS STREQUAL "")
	message( FATAL_ERROR "Could not locate OpenCV include " )
endif()
	
#Finding OpenCV lib component 
set(OpenCV_LIB_COMPONENTS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect opencv_contrib opencv_legacy opencv_ocl)
set(CVLIB_SUFFIX $ENV{OCVCL_VER})        
set(GLOBAL OpenCV_LIBS "")
set(OpenCV_FOUND_TMP true)

		
 ## Loop over each components
foreach(__CVLIB ${OpenCV_LIB_COMPONENTS})

    find_library(OpenCV_${__CVLIB}_LIBRARY_DEBUG NAMES "${__CVLIB}${CVLIB_SUFFIX}d" "lib${__CVLIB}${CVLIB_SUFFIX}d" "${__CVLIB}d" PATHS "${OpenCV_DIR_LIB}" NO_DEFAULT_PATH)
    find_library(OpenCV_${__CVLIB}_LIBRARY_RELEASE NAMES "${__CVLIB}${CVLIB_SUFFIX}" "lib${__CVLIB}${CVLIB_SUFFIX}" "${__CVLIB}" PATHS "${OpenCV_DIR_LIB}" NO_DEFAULT_PATH)
    mark_as_advanced( OpenCV_${__CVLIB}_LIBRARY_DEBUG NAMES OpenCV_${__CVLIB}_LIBRARY_RELEASE)


    # Remove the cache value
    set(OpenCV_${__CVLIB}_LIBRARY "" )

    # Both debug/release
    if(OpenCV_${__CVLIB}_LIBRARY_DEBUG AND OpenCV_${__CVLIB}_LIBRARY_RELEASE)
        set(OpenCV_${__CVLIB}_LIBRARY debug ${OpenCV_${__CVLIB}_LIBRARY_DEBUG} optimized ${OpenCV_${__CVLIB}_LIBRARY_RELEASE} )
        
	# only debug
    elseif(OpenCV_${__CVLIB}_LIBRARY_DEBUG)
        set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_DEBUG} )
    
	# only release
    elseif(OpenCV_${__CVLIB}_LIBRARY_RELEASE)
        set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_RELEASE} )
    
	# no library found
    else( )
        set(OpenCV_FOUND_TMP false)
    endif( )

	# Add to the general list
	if(OpenCV_${__CVLIB}_LIBRARY)
        set(OpenCV_LIBS ${OpenCV_LIBS} ${OpenCV_${__CVLIB}_LIBRARY})
    endif( )
	
endforeach( )
        
set(OpenCV_FOUND ${OpenCV_FOUND_TMP} )

    
# Search for ocl in OpenCV_LIB_COMPONENTS
list(FIND OpenCV_LIB_COMPONENTS opencv_ocl CONTAINS_OCL)
if( ${CONTAINS_OCL} EQUAL -1 )
    message(FATAL_ERROR "opencv_ocl component is missing. Please enable ocl and build opencv again ")
    set(OpenCV_FOUND false)
endif( )

if(NOT OpenCV_FOUND)
    message( FATAL_ERROR "Could not locate OpenCV libs " )
endif(NOT OpenCV_FOUND)

############################################################################
# Tweaks for cygwin makefile to work with windows-style path

if( CYGWIN )
    set( PATHS_TO_CONVERT
           OPENCL_INCLUDE_DIRS
           OPENCL_LIBRARIES
           OpenCV_INCLUDE_DIRS
           OpenCV_LIBS
       )
       
    foreach( pathVar ${PATHS_TO_CONVERT} )
        # Convert windows paths to cyg linux absolute path
        execute_process( COMMAND cygpath -ua ${${pathVar}}
                            OUTPUT_VARIABLE ${pathVar}
                            OUTPUT_STRIP_TRAILING_WHITESPACE
                       )
    endforeach( pathVar )
endif( )
############################################################################

set( COMPILER_FLAGS " " )
set( LINKER_FLAGS " " )
set( ADDITIONAL_LIBRARIES "" )

file(GLOB INCLUDE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.h" )
include_directories( ${OPENCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ../../../include/SDKUtil $ENV{AMDAPPSDKROOT}/include/SDKUtil)

add_executable( ${SAMPLE_NAME} ${SOURCE_FILES} ${INCLUDE_FILES} )

# gcc/g++ specific compile options
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set( COMPILER_FLAGS "${COMPILER_FLAGS} -msse2 " )
    
    # Note: "rt" is not present on mingw
    if( UNIX )
        set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} "rt" )
    endif( )
    
    if( BITNESS EQUAL 32 )
        set( COMPILER_FLAGS "${COMPILER_FLAGS} -m32 " )
        set( LINKER_FLAGS "${LINKER_FLAGS} -m32 " )
    else( )
        set( COMPILER_FLAGS "${COMPILER_FLAGS} -m64 " )
        set( LINKER_FLAGS "${LINKER_FLAGS} -m64 " )
    endif( )
    
    set( COMPILER_FLAGS "${COMPILER_FLAGS} ${EXTRA_COMPILER_FLAGS_GXX} " )
    set( LINKER_FLAGS "${LINKER_FLAGS} ${EXTRA_LINKER_FLAGS_GXX} " )
    set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${EXTRA_LIBRARIES_GXX} )
elseif( MSVC )  
    # Samples can specify additional libs/flags using EXTRA* defines
	add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4005 /wd4996 /nologo" )

    set( COMPILER_FLAGS "${COMPILER_FLAGS} ${EXTRA_COMPILER_FLAGS_MSVC} " )
    set( LINKER_FLAGS "${LINKER_FLAGS} ${EXTRA_LINKER_FLAGS_MSVC}  /SAFESEH:NO ")
    set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${EXTRA_LIBRARIES_MSVC} )
endif( )

set_target_properties( ${SAMPLE_NAME} PROPERTIES
                        COMPILE_FLAGS ${COMPILER_FLAGS}
                        LINK_FLAGS ${LINKER_FLAGS}
                     )
target_link_libraries( ${SAMPLE_NAME} ${OPENCL_LIBRARIES} ${OpenCV_LIBS} ${ADDITIONAL_LIBRARIES} )

# Set output directory to bin
if( MSVC )
	set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin/${BITNESS_SUFFIX})
else()
	set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin/${BITNESS_SUFFIX}/${CMAKE_BUILD_TYPE})
endif()

# Copy extra files to binary directory
foreach( extra_file ${EXTRA_FILES} )
    add_custom_command(
        TARGET ${SAMPLE_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/${extra_file}  ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/${extra_file}  ./
        )
endforeach( extra_file )

# Group sample based on FOLDER_GROUP defined in parent folder
if( FOLDER_GROUP )
    set_target_properties(${SAMPLE_NAME} PROPERTIES FOLDER ${FOLDER_GROUP})
endif( )

