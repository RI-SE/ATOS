cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

project(ISO22133 LANGUAGES C CXX)

# Dependencies
include(FetchContent)
execute_process(
	COMMAND ping www.github.com -c 1 -q
	ERROR_QUIET
	RESULT_VARIABLE NO_CONNECTION
)
FetchContent_Declare(googletest
	GIT_REPOSITORY https://github.com/google/googletest.git
	GIT_TAG main
)
FetchContent_MakeAvailable(googletest)


# Swig flags
set(WITH_SWIG OFF CACHE BOOL "Enable swigging")
set(SWIG_WITH_JAVA OFF CACHE BOOL "Swig to target-language java")
set(SWIG_WITH_PYTHON OFF CACHE BOOL "Swig to target-language python")

if(SWIG_WITH_JAVA)
    set(SWIG_TARGET_LANG java)
elseif(SWIG_WITH_PYTHON)
    set(SWIG_TARGET_LANG python)
endif()

set(ISO22133_TARGET ${TARGET_NAMESPACE}${PROJECT_NAME})

include(GNUInstallDirs)
# When building all packages in util top CMakeLists not only this lib we need to set this cmake policy to OLD due to target namespace are not unique in the same build tree in util
# ref for info about the policy https://cmake.org/cmake/help/latest/policy/CMP0002.html 
cmake_policy(SET CMP0002 OLD)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
add_library(${ISO22133_TARGET} SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/positioning.c
	${CMAKE_CURRENT_SOURCE_DIR}/iso22133.c
	${SOURCES}
)

target_include_directories(${ISO22133_TARGET} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${ISO22133_TARGET} m)

set_property(TARGET ${ISO22133_TARGET} PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/iso22133.h
)
set_property(TARGET ${ISO22133_TARGET} APPEND PROPERTY
	PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/positioning.h
)

install(CODE "MESSAGE(STATUS \"Installing target ${ISO22133_TARGET}\")")
install(TARGETS ${ISO22133_TARGET} 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)


# Tests

# Only build tests if we are on x86_64 and not cross compiling
if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" AND ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "${CMAKE_HOST_SYSTEM_PROCESSOR}"))
	enable_testing()
	file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
	add_executable(${ISO22133_TARGET}_test
		${TEST_SOURCES}
	)
	target_link_libraries(${ISO22133_TARGET}_test
		gtest_main
		${ISO22133_TARGET}
	)
	target_include_directories(${ISO22133_TARGET}_test PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/include
	)
	include(GoogleTest)
	gtest_discover_tests(${ISO22133_TARGET}_test)
endif()


# SWIG
if (WITH_SWIG)
    find_package(SWIG REQUIRED)
    if(SWIG_FOUND)
        #Policy for swig_add_library to put target inside first argument to the function
        cmake_policy(SET CMP0078 NEW)
        #Policy for passing SWIG_MODULE_NAME to swig compiler as -module <name>
        cmake_policy(SET CMP0086 NEW)

        include(${SWIG_USE_FILE})

        if(SWIG_WITH_JAVA)
            if (NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Android")
                find_package(JNI REQUIRED)
            endif()
            SET(CMAKE_SWIG_FLAGS -package org.asta.iso22133)
        elseif(SWIG_WITH_PYTHON)
            find_package(Python COMPONENTS Development)
            SET(CMAKE_SWIG_FLAGS "-threads")
        endif()

        set(ISO22133_SWIGGED iso22133_wrap)

        set_property(SOURCE iso22133.i PROPERTY CPLUSPLUS ON)
        set_property(SOURCE iso22133.i PROPERTY INCLUDE_DIRECTORIES
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${Python_INCLUDE_DIRS} #Note that cmake ignored empty variables
            ${JNI_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS}
        )

        swig_add_library(${ISO22133_SWIGGED}
            TYPE SHARED
            LANGUAGE ${SWIG_TARGET_LANG}
            SOURCES iso22133.i
        )
        set_property(TARGET ${ISO22133_SWIGGED} PROPERTY USE_TARGET_INCLUDE_DIRECTORIES TRUE)
        target_include_directories(${ISO22133_SWIGGED} PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${Python_INCLUDE_DIRS}
            ${JNI_INCLUDE_DIRS}
        )

        target_link_libraries(${ISO22133_SWIGGED}
            ${ISO22133_TARGET}
            atomic
            ${Python_LIBRARIES}
            ${JNI_LIBRARIES}
        )

        if(SWIG_WITH_PYTHON)
            execute_process(COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())" OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)
            install(TARGETS ${ISO22133_SWIGGED} DESTINATION ${PYTHON_SITE_PACKAGES})
            install(FILES ${CMAKE_CURRENT_BINARY_DIR}/iso22133.py DESTINATION ${PYTHON_SITE_PACKAGES})
        endif()
    else()
        message(WARNING "SWIG not found")
    endif()
endif()