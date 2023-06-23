cmake_minimum_required(VERSION 3.10)

project(SmoothTrajectory VERSION 0.2
                         DESCRIPTION "Smooth trajectory generator"
                         LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR   ON)
set(CMAKE_VERBOSE_MAKEFILE      ON)
set(CMAKE_COLOR_MAKEFILE        ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)

macro(configure_msvc_runtime)

    # Default to statically-linked runtime.
    if("${MSVC_RUNTIME}" STREQUAL "")
      set(MSVC_RUNTIME "static")
    endif()

    # Set compiler options.
    set(variables
      CMAKE_C_FLAGS_DEBUG
      CMAKE_C_FLAGS_MINSIZEREL
      CMAKE_C_FLAGS_RELEASE
      CMAKE_C_FLAGS_RELWITHDEBINFO
      CMAKE_CXX_FLAGS_DEBUG
      CMAKE_CXX_FLAGS_MINSIZEREL
      CMAKE_CXX_FLAGS_RELEASE
      CMAKE_CXX_FLAGS_RELWITHDEBINFO
    )
    
    if(${MSVC_RUNTIME} STREQUAL "static")
      message(STATUS
        "MSVC -> forcing use of statically-linked runtime."
      )
      foreach(variable ${variables})
        if(${variable} MATCHES "/MD")
          string(REGEX REPLACE "/MD" "/MT" ${variable} "${${variable}}")
        endif()
      endforeach()
    
    else()
      message(STATUS
        "MSVC -> forcing use of dynamically-linked runtime."
      )
      foreach(variable ${variables})
        if(${variable} MATCHES "/MT")
          string(REGEX REPLACE "/MT" "/MD" ${variable} "${${variable}}")
        endif()
      endforeach()

    endif()

endmacro()

if(MSVC)
  configure_msvc_runtime()
else()
  add_compile_options(-Wall)
  add_compile_options(-Wno-reorder)
endif()

if(UNIX)
  add_definitions(-D_OS_UNIX)
elseif(WIN32)
  add_definitions(-D_OS_WINDOWS -DNOMINMAX)
  if(MSVC)
     add_compile_options(/bigobj)
  endif()
endif()

if(UNIX)
	set(ENV{EIGEN3_INCLUDE_DIR} /usr/include/eigen3)
	find_package( Eigen3 REQUIRED )
else()
	set(ENV{EIGEN3_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/external/eigen)
endif()

add_library(SmoothTrajectoryLib STATIC src/smoothTrajectory.cpp include/smoothTrajectory.h)

target_include_directories(SmoothTrajectoryLib PUBLIC include)
target_include_directories(SmoothTrajectoryLib PUBLIC $ENV{EIGEN3_INCLUDE_DIR})

target_compile_definitions(SmoothTrajectoryLib PUBLIC cxx_std_17)

add_executable(trajectory test/trajectory.cpp)

target_link_libraries(trajectory PUBLIC SmoothTrajectoryLib)