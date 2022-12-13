# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindesminiLib
-------

Finds the esminiLib library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``esminiLib::esminiLib``
  The esminiLib library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``esminiLib_FOUND``
  True if the system has the esminiLib library.
``esminiLib_VERSION``
  The version of the esminiLib library which was found.
``esminiLib_INCLUDE_DIRS``
  Include directories needed to use esminiLib.
``esminiLib_LIBRARIES``
  Libraries needed to link to esminiLib.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``esminiLib_INCLUDE_DIR``
  The directory containing ``esminiLib.hpp``, ``esminiRMLib.hpp`` etc..
``esminiLib_LIBRARY``
  The path to the esminiLib library.

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