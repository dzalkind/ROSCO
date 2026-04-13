cmake_minimum_required(VERSION 3.14)
project(ROSCO VERSION 2.9.0 LANGUAGES C CXX)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the build type: Debug RelWithDebInfo Release" FORCE)
endif()

# C++17 required for toml++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# VIT: disable FP contraction for bit-identical verification during translation phase.
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-contract=off")
endif()

# toml++ (single-header, C++17, no external deps)
include(FetchContent)
FetchContent_Declare(
    tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG        v3.4.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(tomlplusplus)

set(SOURCES
    src/rosco_types_io.cpp
    src/discon.cpp
    src/ControllerBlocks/computevariablessetpoints.cpp
    src/ControllerBlocks/pitchsaturation.cpp
    src/ControllerBlocks/powercontrolsetpoints.cpp
    src/ControllerBlocks/refspeedexclusion.cpp
    src/ControllerBlocks/setpointsmoother.cpp
    src/ControllerBlocks/shutdown.cpp
    src/ControllerBlocks/startup.cpp
    src/ControllerBlocks/statemachine.cpp
    src/ControllerBlocks/windspeedestimator.cpp
    src/Controllers/activewakecontrol.cpp
    src/Controllers/cablecontrol.cpp
    src/Controllers/flapcontrol.cpp
    src/Controllers/floatingfeedback.cpp
    src/Controllers/foreaftdamping.cpp
    src/Controllers/ipc.cpp
    src/Controllers/picontroller.cpp
    src/Controllers/pidcontroller.cpp
    src/Controllers/piicontroller.cpp
    src/Controllers/pitchcontrol.cpp
    src/Controllers/rescontroller.cpp
    src/Controllers/structuralcontrol.cpp
    src/Controllers/variablespeedcontrol.cpp
    src/Controllers/yawratecontrol.cpp
    src/Filters/hpfilter.cpp
    src/Filters/lpfilter.cpp
    src/Filters/notchfilter.cpp
    src/Filters/notchfilterslopes.cpp
    src/Filters/prefiltermeasuredsignals.cpp
    src/Filters/seclpfilter.cpp
    src/Filters/seclpfilter_vel.cpp
    src/Functions/aerodyntorque.cpp
    src/Functions/colemantransform.cpp
    src/Functions/colemantransforminverse.cpp
    src/Functions/identity.cpp
    src/Functions/interp1d.cpp
    src/Functions/interp2d.cpp
    src/Functions/ratelimit.cpp
    src/Functions/saturate.cpp
    src/Functions/sigma.cpp
    src/Functions/unwrap.cpp
    src/Functions/wrap_180.cpp
    src/Functions/wrap_360.cpp
    src/IO/debug.cpp
    src/IO/extcontroller.cpp
    src/IO/readrestartfile.cpp
    src/IO/updatezeromq.cpp
    src/IO/writerestartfile.cpp
    src/ReadSetParameters/checkinputs.cpp
    src/ReadSetParameters/readavrswap.cpp
    src/ReadSetParameters/readcontrolparameterfilesub.cpp
    src/ReadSetParameters/readcpfile.cpp
    src/ReadSetParameters/setparameters.cpp
)

# ZMQ Library
find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ libzmq)
if(PC_ZeroMQ_FOUND)
    find_path(ZeroMQ_INCLUDE_DIR NAMES zmq.h PATHS ${PC_ZeroMQ_INCLUDE_DIRS})
    find_library(ZeroMQ_LIBRARY NAMES zmq PATHS ${PC_ZeroMQ_LIBRARY_DIRS})
    add_library(zmq_client OBJECT src/zmq_client.c)
    target_include_directories(zmq_client PRIVATE ${ZeroMQ_INCLUDE_DIR})
    if(NOT MSVC)
      target_compile_options(zmq_client PRIVATE -fPIC)
    endif()
    add_definitions(-DZMQ_CLIENT="TRUE")
    set(SOURCES ${SOURCES} $<TARGET_OBJECTS:zmq_client>)
endif()

add_library(discon SHARED ${SOURCES})
target_include_directories(discon PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../translations)
target_link_libraries(discon PRIVATE stdc++ dl tomlplusplus::tomlplusplus)

if(PC_ZeroMQ_FOUND)
    target_include_directories(discon PUBLIC ${ZeroMQ_INCLUDE_DIR})
    target_link_libraries(discon PUBLIC ${ZeroMQ_LIBRARY})
endif()

install(TARGETS discon
  EXPORT "${CMAKE_PROJECT_NAME}Libraries"
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

set(linuxDefault ${CMAKE_INSTALL_PREFIX} STREQUAL "/usr/local")
set(windowsDefault ${CMAKE_INSTALL_PREFIX} STREQUAL "C:\\Program Files (x86)")
if(${linuxDefault} OR ${windowsDefault})
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/../")
endif()
