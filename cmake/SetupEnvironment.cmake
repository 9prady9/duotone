# Set a default build type if none was specified
set(default_build_type "Release")
if(EXISTS "${CMAKE_SOURCE_DIR}/.git")
  set(default_build_type "Debug")
endif()

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

# Includes the directory if the variable is set
function(conditional_directory variable directory)
  if(${variable})
    add_subdirectory(${directory})
  endif()
endfunction()

function(dependency_check VAR ERROR_MESSAGE)
  if(NOT ${VAR})
    message(SEND_ERROR ${ERROR_MESSAGE})
  endif()
endfunction()
