# Findcinder.cmake
# Author: Pradeep Garigipati<pradeep.garigipati@gmail.com>
#
# cinder_ROOT - Set this variable in CMake-GUI to help find cinder header and library
#
# Finds the cinder libraries
# Sets the following variables:
#          cinder_FOUND
#          cinder_INCLUDE_DIR
#          cinder_STATIC_LIBRARY
#
# Usage:
# find_package(cinder)
# if (cinder_FOUND)
#    target_link_libraries(mylib PRIVATE cinder::cinder_STATIC)
# endif (cinder_FOUND)

find_path(cinder_INCLUDE_DIR
    NAMES Cinder.h
    PATHS
        /usr/include
        /usr/local/include
        /opt/local/include
		${cinder_ROOT}/include
		${cinder_ROOT}/include/cinder
    DOC "Directory where cinder.h is present")

#Move a directory up to get Cinder include directory
set(cinder_INCLUDE_DIR "${cinder_INCLUDE_DIR}/../")

find_library(cinder_STATIC_LIBRARY
    NAMES ${PX}cinder-v141${SX} ${PX}Cinder-v141${SX}
    HINTS ENV cinder_ROOT
    PATHS
        /usr/local
        /opt/local
        /usr/lib/x86_64-linux-gnu
		${cinder_ROOT}/lib/x64
		${cinder_ROOT}/lib/msw/x64
    PATH_SUFFIXES
        lib64        
    DOC "cinder static library")

mark_as_advanced(
    cinder_INCLUDE_DIR
    cinder_STATIC_LIBRARY
    )
message(STATUS "cinder_INCLUDE_DIR ${cinder_INCLUDE_DIR}")
message(STATUS "cinder_STATIC_LIBRARY ${cinder_STATIC_LIBRARY}")

if (cinder_STATIC_LIBRARY AND cinder_INCLUDE_DIR AND NOT TARGET Cinder::cinder_STATIC)
    add_library(Cinder::cinder_STATIC UNKNOWN IMPORTED)
    set_target_properties(Cinder::cinder_STATIC PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGE "C"
        IMPORTED_LOCATION "${cinder_STATIC_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${cinder_INCLUDE_DIR}")
else ()
    message(FATAL_ERROR "Cinder static library not found")
endif ()