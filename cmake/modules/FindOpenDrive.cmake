# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindOpenDrive
-------

Finds the OpenDrive library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``OpenDrive::OpenDrive``
  The OpenDrive library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``OpenDrive_FOUND``
  True if the system has the OpenDrive library.
``OpenDrive_VERSION``
  The version of the OpenDrive library which was found.
``OpenDrive_INCLUDE_DIRS``
  Include directories needed to use OpenDrive.
``OpenDrive_LIBRARIES``
  Libraries needed to link to OpenDrive.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``OpenDrive_INCLUDE_DIR``
  The directory containing ``xodr.h``, ``xosc.h`` etc..
``OpenDrive_LIBRARY``
  The path to the OpenDrive library.

#]=======================================================================]


find_package(PkgConfig)
pkg_check_modules(PC_OpenDrive QUIET OpenDrive)

find_path(OpenDrive_INCLUDE_DIR
  NAMES Lane.h
  PATHS ${PC_OpenDrive_INCLUDE_DIRS}
  PATH_SUFFIXES libOpenDrive
)
find_library(OpenDrive_LIBRARY
  NAMES OpenDrive
  PATHS ${PC_OpenDrive_LIBRARY_DIRS}
)
set(OpenDrive_VERSION ${PC_OpenDrive_VERSION})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenDrive
  FOUND_VAR OpenDrive_FOUND
  REQUIRED_VARS
    OpenDrive_LIBRARY
    OpenDrive_INCLUDE_DIR
  VERSION_VAR OpenDrive_VERSION
)

if(OpenDrive_FOUND)
  set(OpenDrive_LIBRARIES ${OpenDrive_LIBRARY})
  set(OpenDrive_INCLUDE_DIRS ${OpenDrive_INCLUDE_DIR})
  set(OpenDrive_DEFINITIONS ${PC_OpenDrive_CFLAGS_OTHER})
endif()

mark_as_advanced(
  OpenDrive_INCLUDE_DIR
  OpenDrive_LIBRARY
)