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


set( SAMPLE_NAME PerlinNoise )
set( SOURCE_FILES PerlinNoise.cpp )
set( EXTRA_FILES  )
option( ENABLE_TBB "ENABLE_TBB" OFF)

############################################################################

set(CMAKE_SUPPRESS_REGENERATION TRUE)
cmake_minimum_required( VERSION 2.8.0 )
project( ${SAMPLE_NAME} )

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
if( CMAKE_BUILD_TYPE MATCHES "[Dd]ebug" )
    set(LIB_SUFFIX .debug)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "")
	set(CMAKE_BUILD_TYPE Release)
endif()

# Set platform
if( NOT UNIX )
	set(PLATFORM win)
else()
	set(PLATFORM lnx)
endif()
############################################################################
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
# Find Bolt include and libs

find_path( BOLT_INCLUDE_DIRS 
    NAMES bolt/BoltVersion.h
    HINTS $ENV{BOLTLIB_DIR}/include
)
mark_as_advanced(BOLT_INCLUDE_DIRS)

if( BITNESS EQUAL 64 )
    find_path( BOLT_LIB_DIR
	if( NOT UNIX )
	    NAMES clBolt.runtime.vc110.lib clBolt.runtime.vc100.lib
	else()
	    NAMES libclBolt.runtime.gcc.a
	endif()
	HINTS $ENV{BOLTLIB_DIR}
	PATH_SUFFIXES lib64
    )
else( )
    find_path( BOLT_LIB_DIR
	if( NOT UNIX )
	    NAMES clBolt.runtime.vc110.lib clBolt.runtime.vc100.lib
	else()
        NAMES libclBolt.runtime.gcc.a
    endif()
	HINTS $ENV{BOLTLIB_DIR}
	PATH_SUFFIXES lib
    )
endif( )
mark_as_advanced( BOLT_LIB_DIR )

if( BOLT_INCLUDE_DIRS STREQUAL "" OR BOLT_LIB_DIR STREQUAL "")
	message( FATAL_ERROR "Could not locate Bolt include & libs" )
endif( )

if(CMAKE_BUILD_TYPE MATCHES "[Tt][Bb][Bb]")
	set(BOLT_INCLUDE_DIRS "${BOLT_INCLUDE_DIRS} $ENV{TBB_ROOT}/include")
endif()

############################################################################
# Find TBB include and libs

if( ${ENABLE_TBB} MATCHES ON)

message( STATUS "Enabling TBB..")

find_path( TBB_INCLUDE_DIRS 
    NAMES tbb/tbb.h
    HINTS $ENV{TBB_ROOT}/include
)

mark_as_advanced(BOLT_INCLUDE_DIRS)

if( NOT UNIX )
    if( MSVC_VERSION LESS 1700)
	if( BITNESS EQUAL 64 )
		set ( TBB_LIB_PATH_SUFFIX  "lib/intel64/vc10" )
	else( )
		set ( TBB_LIB_PATH_SUFFIX  "lib/ia32/vc10" )
	endif( )
    else( )
        if(BITNESS EQUAL 64)
		set ( TBB_LIB_PATH_SUFFIX  "lib/intel64/vc11")
	else( )
		set ( TBB_LIB_PATH_SUFFIX  "lib/ia32/vc11")
	endif( )
    endif( )

    find_path( TBB_LIB_DIR
        NAMES  tbb_debug.lib tbb.lib
        HINTS $ENV{TBB_ROOT}/${TBB_LIB_PATH_SUFFIX}
    )
else( )
    if(BITNESS EQUAL 64)
		set ( TBB_LIB_PATH_SUFFIX  "lib/intel64/gcc4.4")
    else( )
		set ( TBB_LIB_PATH_SUFFIX  "lib/ia32/gcc4.4")
    endif( )

    find_path( TBB_LIB_DIR
        NAMES  libtbb_debug.so libtbb.so
        HINTS $ENV{TBB_ROOT}/${TBB_LIB_PATH_SUFFIX}
    )
endif( )

mark_as_advanced( TBB_LIB_DIR )

if( TBB_INCLUDE_DIRS STREQUAL "" OR TBB_LIB_DIR STREQUAL "")
	message( FATAL_ERROR "Could not locate Bolt include & libs" )
endif( )


endif( )

############################################################################
# Tweaks for cygwin makefile to work with windows-style path

if( CYGWIN )
    set( PATHS_TO_CONVERT
           OPENCL_INCLUDE_DIRS
           OPENCL_LIBRARIES
           BOLT_INCLUDE_DIRS
           BOLT_LIB_DIR
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
include_directories( ${OPENCL_INCLUDE_DIRS} ../../../include/SDKUtil $ENV{AMDAPPSDKROOT}/include/SDKUtil ${BOLT_INCLUDE_DIRS} )
link_directories( ${BOLT_LIB_DIR} )

if( ${ENABLE_TBB} MATCHES ON)
include_directories( ${OPENCL_INCLUDE_DIRS} ../../../include/SDKUtil  ${BOLT_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS}  )
link_directories(  ${BOLT_LIB_DIR} ${TBB_LIB_DIR} )
set ( SAMPLE_NAME "${SAMPLE_NAME}_TBB" )
else()
include_directories(${OPENCL_INCLUDE_DIRS}../../../include/SDKUtil  ${BOLT_INCLUDE_DIRS} )
link_directories(  ${BOLT_LIB_DIR} )
endif( )


add_executable( ${SAMPLE_NAME} ${SOURCE_FILES} ${INCLUDE_FILES} ${EXTRA_FILES})

# gcc/g++ specific compile options
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set( COMPILER_FLAGS "${COMPILER_FLAGS} -msse2 " )
    
    # Note: "rt" is not present on mingw
    if( UNIX )
		if( CMAKE_BUILD_TYPE MATCHES "[Dd]ebug" )
			set( COMPILER_FLAGS " -g " )
		endif( )
		set( COMPILER_FLAGS "${COMPILER_FLAGS} -msse2 -fpermissive -std=c++0x " )
        set( LINKER_FLAGS " -Xlinker -zmuldefs " )
        set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} "rt" )
	if( ${ENABLE_TBB} MATCHES ON)
	    if( CMAKE_BUILD_TYPE MATCHES "[Dd]ebug" )
		set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} "tbb_debug" )
	    else( )
		set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} "tbb" )
	    endif( )
	endif( )
    endif( )
    
    set( COMPILER_FLAGS "${COMPILER_FLAGS} -m${BITNESS} ${EXTRA_COMPILER_FLAGS_GXX} " )
    set( LINKER_FLAGS "${LINKER_FLAGS} -m${BITNESS} ${EXTRA_LINKER_FLAGS_GXX} " )
    set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${EXTRA_LIBRARIES_GXX} )
elseif( MSVC )
    # Samples can specify additional libs/flags using EXTRA* defines
	add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4005 /wd4996  /wd4244 /nologo" )
	
    set( COMPILER_FLAGS "${COMPILER_FLAGS} ${EXTRA_COMPILER_FLAGS_MSVC} " )
    set( LINKER_FLAGS "${LINKER_FLAGS} ${EXTRA_LINKER_FLAGS_MSVC} " )
    set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${EXTRA_LIBRARIES_MSVC} )
endif( )

if( ${ENABLE_TBB} MATCHES ON)
	 set( COMPILER_FLAGS "${COMPILER_FLAGS} -DENABLE_TBB=1 " )
endif()

if( MSVC10 )
    set( VCVER vc100 )
elseif( MSVC11 )
    set( VCVER vc110 )
endif()

if ( NOT UNIX )
    set( BOLTLIB optimized clBolt.runtime.${VCVER} debug clBolt.runtime.${VCVER}.debug )
else()
    set( BOLTLIB clBolt.runtime.gcc${LIB_SUFFIX} boost_system )
endif()

set_target_properties( ${SAMPLE_NAME} PROPERTIES
                        COMPILE_FLAGS ${COMPILER_FLAGS}
                        LINK_FLAGS ${LINKER_FLAGS}
                     )
target_link_libraries( ${SAMPLE_NAME} ${OPENCL_LIBRARIES} ${ADDITIONAL_LIBRARIES} )
target_link_libraries( ${SAMPLE_NAME} ${BOLTLIB} )

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
