cmake_minimum_required(VERSION 3.10)

project(DuotoneSurfaces VERSION 0.1.0 LANGUAGES C CXX)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_PREFIX_PATH "${CMAKE_BINARY_DIR}prefix;${CMAKE_PREFIX_PATH}")

include(SetupEnvironment)

find_package(cinder REQUIRED)
find_package(Boost REQUIRED COMPONENTS "filesystem")

set(target "${PROJECT_NAME}")

add_subdirectory(source/Fenghui_Zhang_Core)

if (WIN32)
    # force Unicode over Multi-byte
    add_definitions(-DUNICODE -D_UNICODE)
endif ()

add_executable(${target}
    ${CMAKE_SOURCE_DIR}/source/circularlist.h
    ${CMAKE_SOURCE_DIR}/source/circularlist.cpp
    ${CMAKE_SOURCE_DIR}/source/Resources.h
    ${CMAKE_SOURCE_DIR}/source/vector3d.h
    ${CMAKE_SOURCE_DIR}/source/vector3d.cpp
    ${CMAKE_SOURCE_DIR}/source/Tube.h
    ${CMAKE_SOURCE_DIR}/source/Tube.cpp
    ${CMAKE_SOURCE_DIR}/source/Weave.h
    ${CMAKE_SOURCE_DIR}/source/Weave.cpp
    ${CMAKE_SOURCE_DIR}/source/WeaveBoxApp.cpp
    )

target_include_directories(${target} PRIVATE ${Boost_INCLUDE_DIRS})

if (WIN32)
    target_compile_definitions(${target} PRIVATE OS_WIN WIN32_MEAN_AND_LEAN)
elseif (APPLE)
    target_compile_definitions(${target} PRIVATE OS_MAC)
else (WIN32)
    target_compile_definitions(${target} PRIVATE OS_LNX)
endif (WIN32)

target_link_libraries(${target}
    PRIVATE
	FenghuiZhangCore
	Cinder::cinder_STATIC
	${Boost_LIBRARIES})
