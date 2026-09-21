cmake_minimum_required(VERSION 3.14)
project(ROSCO VERSION 2.9.0 LANGUAGES C CXX)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the build type: Debug RelWithDebInfo Release" FORCE)
endif()

# C++17 required for toml++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Reproducibility: the regression suite in test/regression/ compares controller
# output bit-for-bit against frozen baselines. Disabling FP contraction is what
# makes those baselines survive across compilers and optimisation levels.
# Do not remove, and do not let a CMake preset add -ffast-math.
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-contract=off")
endif()

# GCC emits STB_GNU_UNIQUE symbols for static data members of templates and
# static locals in inline functions (toml++ has both). glibc marks any library
# holding such a symbol NODELETE, so dlclose() never unloads libdiscon and the
# controller's static state survives into the next load. Callers that reload the
# library in one process — the Python ControllerInterface, OpenFAST with several
# turbines — then hit "this ROSCO dynamic library has already been loaded".
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-gnu-unique")
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
    src/controller_objects.cpp
    src/Stages/stage_1_sensing.cpp
    src/Stages/stage_2_setup.cpp
    src/Stages/stage_3_filtering.cpp
    src/Stages/stage_4_estimation.cpp
    src/Stages/stage_5_supervisory.cpp
    src/Stages/stage_6_setpoints.cpp
    src/Stages/stage_7_actuators.cpp
    src/Stages/stage_8_output.cpp
    src/Setpoints/speedsetpoints.cpp
    src/Setpoints/pitchsaturation.cpp
    src/Setpoints/powercontrolsetpoints.cpp
    src/Setpoints/refspeedexclusion.cpp
    src/Setpoints/setpointsmoother.cpp
    src/Supervisory/shutdown.cpp
    src/Supervisory/startup.cpp
    src/SubControllers/torquestatemachine.cpp
    src/Estimators/windspeedestimator.cpp
    src/SubControllers/activewakecontrol.cpp
    src/Controllers/cablecontrol.cpp
    src/Controllers/flapcontrol.cpp
    src/SubControllers/floatingfeedback.cpp
    src/SubControllers/foreaftdamping.cpp
    src/SubControllers/ipc.cpp
    src/Controllers/pitchcontrol.cpp
    src/Controllers/structuralcontrol.cpp
    src/Controllers/torquecontrol.cpp
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
    src/Functions/interpolation.cpp
    src/Functions/linalg.cpp
    src/Functions/sigma.cpp
    src/Functions/wrapping.cpp
    src/ControlElements/ratelimiter.cpp
    src/ControlElements/rescontroller.cpp
    src/ControlElements/saturate.cpp
    src/IO/debug.cpp
    src/IO/text_debug_writer.cpp
    src/IO/extcontroller.cpp
    src/IO/readrestartfile.cpp
    src/IO/updatezeromq.cpp
    src/IO/writerestartfile.cpp
    src/ReadSetParameters/checkinputs.cpp
    src/ReadSetParameters/readavrswap.cpp
    src/ReadSetParameters/readcontrolparameterfilesub.cpp
    src/ReadSetParameters/readcpfile.cpp
    src/ReadSetParameters/readconfigfiles.cpp
    src/ReadSetParameters/setparameters.cpp
)

# HDF5 Library (optional)
option(ROSCO_HDF5 "Enable HDF5 debug output" ON)
if(ROSCO_HDF5)
    find_package(HDF5 COMPONENTS C)
    if(HDF5_FOUND)
        message(STATUS "HDF5 found: ${HDF5_VERSION} — HDF5 debug output enabled")
        add_definitions(-DROSCO_HDF5=1)
        set(SOURCES ${SOURCES} src/IO/hdf5_debug_writer.cpp)
    else()
        message(STATUS "HDF5 not found — text-only debug output")
    endif()
endif()

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
target_link_libraries(discon PRIVATE ${CMAKE_DL_LIBS})
target_link_libraries(discon PRIVATE tomlplusplus::tomlplusplus)

# A mingw-built DLL that imports libstdc++-6 / libgcc_s_seh-1 / libwinpthread-1
# binds at load time to whichever copy comes first on PATH, which is rarely the
# GCC that compiled it: a conda environment, for instance, puts its own
# (differently versioned) copies ahead of the toolchain's. The controller then
# crashes as soon as C++ runtime state crosses that boundary. Linking the GCC
# runtimes statically makes libdiscon.dll self-contained and immune to PATH.
if(MINGW)
    target_link_options(discon PRIVATE
        -static-libgcc -static-libstdc++
        -Wl,-Bstatic,--whole-archive,-lwinpthread,--no-whole-archive,-Bdynamic)
endif()

if(HDF5_FOUND)
    target_include_directories(discon PRIVATE ${HDF5_INCLUDE_DIRS})
    target_link_libraries(discon PRIVATE ${HDF5_C_LIBRARIES})
endif()

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
