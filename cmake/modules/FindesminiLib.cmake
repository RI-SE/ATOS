# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
Findad-xolib
-------

Finds the ad-xolib library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``ad-xolib::ad-xolib``
  The ad-xolib library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``ad-xolib_FOUND``
  True if the system has the ad-xolib library.
``ad-xolib_VERSION``
  The version of the ad-xolib library which was found.
``ad-xolib_INCLUDE_DIRS``
  Include directories needed to use ad-xolib.
``ad-xolib_LIBRARIES``
  Libraries needed to link to ad-xolib.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``ad-xolib_INCLUDE_DIR``
  The directory containing ``xodr.h``, ``xosc.h`` etc..
``ad-xolib_LIBRARY``
  The path to the ad-xolib library.

#]=======================================================================]


find_package(PkgConfig)
pkg_check_modules(PC_esminiLib QUIET esminiLib)

find_path(esminiLib_INCLUDE_DIR
  NAMES esminiLib.hpp
  PATHS ${PC_esminiLib_INCLUDE_DIRS}
  PATH_SUFFIXES esmini
)
find_library(esminiLib_LIBRARY
  NAMES esminiLib esminiRMLib
  PATHS ${PC_esminiLib_LIBRARY_DIRS}
)

set(esminiLib_VERSION ${PC_esminiLib_VERSION})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(esminiLib
  FOUND_VAR esminiLib_FOUND
  REQUIRED_VARS
    esminiLib_LIBRARY
    esminiLib_INCLUDE_DIR
  VERSION_VAR esminiLib_VERSION
)

if(esminiLib_FOUND)
  set(esminiLib_LIBRARIES ${esminiLib_LIBRARY})
  set(esminiLib_INCLUDE_DIRS ${esminiLib_INCLUDE_DIR})
  set(esminiLib_DEFINITIONS ${PC_esminiLib_CFLAGS_OTHER})
endif()

mark_as_advanced(
    esminiLib_INCLUDE_DIR
    esminiLib_LIBRARY
)