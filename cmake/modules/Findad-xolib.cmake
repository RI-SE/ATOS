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
pkg_check_modules(PC_ad-xolib QUIET ad-xolib)

find_path(ad-xolib_INCLUDE_DIR
  NAMES xodr.h
  PATHS ${PC_ad-xolib_INCLUDE_DIRS}
  PATH_SUFFIXES ad-xolib
)
find_library(ad-xodr_LIBRARY
  NAMES xodr
  PATHS ${PC_ad-xolib_LIBRARY_DIRS}
)
find_library(ad-xosc_LIBRARY
  NAMES xosc
  PATHS ${PC_ad-xolib_LIBRARY_DIRS}
)

set(ad-xolib_VERSION ${PC_ad-xolib_VERSION})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ad-xolib
  FOUND_VAR ad-xolib_FOUND
  REQUIRED_VARS
    ad-xosc_LIBRARY
    ad-xodr_LIBRARY
    ad-xolib_INCLUDE_DIR
  VERSION_VAR ad-xolib_VERSION
)

if(ad-xolib_FOUND)
  set(ad-xolib_LIBRARIES ${ad-xosc_LIBRARY} ${ad-xodr_LIBRARY})
  set(ad-xolib_INCLUDE_DIRS ${ad-xolib_INCLUDE_DIR})
  set(ad-xolib_DEFINITIONS ${PC_ad-xolib_CFLAGS_OTHER})
endif()

mark_as_advanced(
  ad-xolib_INCLUDE_DIR
  ad-xolib_LIBRARY
)