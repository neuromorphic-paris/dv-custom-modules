# Init CMake (require at least version 3.10.0)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

# General build settings
IF (NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Possible build types: None Debug Release RelWithDebInfo MinSizeRel")
ENDIF()

IF (NOT CMAKE_INSTALL_PREFIX)
	IF (UNIX AND NOT APPLE)
		SET(CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "CMake default install prefix, set to /usr on Unix/Linux")
	ELSEIF(APPLE)
		SET(CMAKE_INSTALL_PREFIX "/usr/local" CACHE STRING "CMake default install prefix, set to /usr/local on macOS")
	ELSE()
		MESSAGE(FATAL_ERROR "CMAKE_INSTALL_PREFIX is not set")
	ENDIF()
ENDIF()

# Project name and version
PROJECT(dv-custom-modules
	VERSION 1.0.0
	LANGUAGES C CXX)

# Define installation paths
INCLUDE(GNUInstallDirs)

# Custom CMake base modules
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
	${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/${CMAKE_INSTALL_DATAROOTDIR}/dv
	${CMAKE_INSTALL_PREFIX}/share/dv /usr/share/dv
	/usr/local/${CMAKE_INSTALL_DATAROOTDIR}/dv /usr/local/share/dv)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmakemod
	${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/caer /usr/${CMAKE_INSTALL_DATAROOTDIR}/caer
	${CMAKE_INSTALL_PREFIX}/share/caer /usr/share/caer
	/usr/local/${CMAKE_INSTALL_DATAROOTDIR}/caer /usr/local/share/caer)

# Basic setup
INCLUDE(dv-modules)
DV_MODULE_SETUP()


# libcaer devices and logging support.
FIND_PACKAGE(libcaer 3.3.8 REQUIRED)


# Set full RPATH, modules are libraries for DV
SET(CMAKE_INSTALL_RPATH ${DV_MODULES_DIR})

# Compile this module (standard C++ file, no additional dependencies)
ADD_LIBRARY(davis SHARED src/davis.cpp src/aedat4_convert.cpp)

SET_TARGET_PROPERTIES(davis
	PROPERTIES
	PREFIX "sync_"
)

TARGET_LINK_LIBRARIES(davis PRIVATE ${DV_LIBRARIES} libcaer::caer)

INSTALL(TARGETS davis DESTINATION ${DV_MODULES_DIR})
