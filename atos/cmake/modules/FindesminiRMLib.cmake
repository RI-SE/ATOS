# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindesminiRMLib
-------

Finds the esminiRMLib library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``esminiRMLib::esminiRMLib``
  The esminiRMLib library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``esminiRMLib_FOUND``
  True if the system has the esminiRMLib library.
``esminiRMLib_VERSION``
  The version of the esminiRMLib library which was found.
``esminiRMLib_INCLUDE_DIRS``
  Include directories needed to use esminiRMLib.
``esminiRMLib_LIBRARIES``
  Libraries needed to link to esminiRMLib.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``esminiRMLib_INCLUDE_DIR``
  The directory containing ``esminiRMLib.hpp``, ``esminiRMRMLib.hpp`` etc..
``esminiRMLib_LIBRARY``
  The path to the esminiRMLib library.

#]=======================================================================]


find_package(PkgConfig)
pkg_check_modules(PC_esminiRMLib QUIET esminiRMLib)

find_path(esminiRMLib_INCLUDE_DIR
  NAMES esminiRMLib.hpp
  PATHS ${PC_esminiRMLib_INCLUDE_DIRS}
  PATH_SUFFIXES esmini
)
find_library(esminiRMLib_LIBRARY
  NAMES esminiRMLib esminiRMRMLib
  PATHS ${PC_esminiRMLib_LIBRARY_DIRS}
)

set(esminiRMLib_VERSION ${PC_esminiRMLib_VERSION})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(esminiRMLib
  FOUND_VAR esminiRMLib_FOUND
  REQUIRED_VARS
    esminiRMLib_LIBRARY
    esminiRMLib_INCLUDE_DIR
  VERSION_VAR esminiRMLib_VERSION
)

if(esminiRMLib_FOUND)
  set(esminiRMLib_LIBRARIES ${esminiRMLib_LIBRARY})
  set(esminiRMLib_INCLUDE_DIRS ${esminiRMLib_INCLUDE_DIR})
  set(esminiRMLib_DEFINITIONS ${PC_esminiRMLib_CFLAGS_OTHER})
endif()

mark_as_advanced(
    esminiRMLib_INCLUDE_DIR
    esminiRMLib_LIBRARY
)