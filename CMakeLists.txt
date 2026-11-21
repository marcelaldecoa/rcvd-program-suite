cmake_minimum_required(VERSION 3.24)   # spec §4.2

project(rcvd
  VERSION 0.1.0
  DESCRIPTION "Race Car Vehicle Dynamics Program Suite (RCVD-SUITE-SPEC Rev 1.0)"
  LANGUAGES CXX)

# ---- §4.2 language and toolchain -------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)          # "no compiler extensions"

option(RCVD_BUILD_TESTS "Build the Catch2 test suite" ON)

# §4.2 / Appendix D: -ffast-math and anything implying it are FORBIDDEN — it
# breaks IEEE 754 reproducibility (Rule A2) and NaN propagation (§13.1, which
# makes a NaN a fatal defect that must be detectable). Contraction is disabled
# explicitly so an FMA cannot silently change results between platforms.
add_library(rcvd_flags INTERFACE)
if(MSVC)
  target_compile_options(rcvd_flags INTERFACE
    /W4 /WX /permissive-
    # /fp:precise is never /fp:fast, and it also leaves FMA contraction OFF:
    # MSVC only contracts under /fp:fast or an explicit /fp:contract. There is no
    # /fp:contract- spelling — passing one is a D9002 "unknown option" warning
    # that leaves you believing contraction is disabled when nothing was applied.
    /fp:precise
    /Zc:__cplusplus /utf-8)
else()
  target_compile_options(rcvd_flags INTERFACE
    -Wall -Wextra -Wpedantic -Werror
    -fno-fast-math -ffp-contract=off)
endif()

# Appendix D: "No -ffast-math in any build configuration — verified by inspecting
# the compile commands in CI." Fail the configure step rather than the audit.
get_directory_property(_rcvd_all_flags COMPILE_OPTIONS)
foreach(_f IN LISTS _rcvd_all_flags CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_RELEASE)
  if(_f MATCHES "fast-math|/fp:fast|-Ofast")
    message(FATAL_ERROR "§4.2 forbids -ffast-math and flags implying it; found: ${_f}")
  endif()
endforeach()

# ---- units + core + numerics (Appendix C steps 1-3) ------------------------
# Header-only so far; sources arrive with core's JSON layer at step 2.
add_library(rcvd_units INTERFACE)
target_include_directories(rcvd_units INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_library(rcvd_core INTERFACE)
target_include_directories(rcvd_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rcvd_core INTERFACE rcvd_units)

add_library(rcvd_numerics INTERFACE)
target_include_directories(rcvd_numerics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rcvd_numerics INTERFACE rcvd_core)

# §4.1 dependency rule: dependencies point strictly downward. numerics may see
# core; neither may ever see io/, capi/ or app/. Enforced here by PRIVATE linkage
# once those targets exist, plus the include-cycle check in CI.

add_library(rcvd_tire INTERFACE)
target_include_directories(rcvd_tire INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rcvd_tire INTERFACE rcvd_numerics)  # conformance uses (13.2)

add_library(rcvd_aero INTERFACE)
target_include_directories(rcvd_aero INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rcvd_aero INTERFACE rcvd_units)

add_library(rcvd_analysis INTERFACE)
target_include_directories(rcvd_analysis INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rcvd_analysis INTERFACE rcvd_core rcvd_numerics)

add_library(rcvd_models INTERFACE)
target_include_directories(rcvd_models INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rcvd_models INTERFACE rcvd_core rcvd_numerics rcvd_tire rcvd_aero)

add_library(rcvd_io INTERFACE)
target_include_directories(rcvd_io INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rcvd_io INTERFACE rcvd_core rcvd_tire)

add_library(rcvd::units    ALIAS rcvd_units)
add_library(rcvd::core     ALIAS rcvd_core)
add_library(rcvd::numerics ALIAS rcvd_numerics)
add_library(rcvd::tire     ALIAS rcvd_tire)
add_library(rcvd::models   ALIAS rcvd_models)
add_library(rcvd::aero     ALIAS rcvd_aero)
add_library(rcvd::analysis ALIAS rcvd_analysis)
add_library(rcvd::io       ALIAS rcvd_io)

# §4.1: the extern "C" ABI that the Python, C# and MATLAB bindings consume.
# The only compiled target so far; everything below it is header-only.
add_library(rcvd_capi src/capi/rcvd_capi.cpp)
target_include_directories(rcvd_capi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rcvd_capi
  PUBLIC  rcvd::core
  PRIVATE rcvd::models rcvd::analysis rcvd::io rcvd_flags)
add_library(rcvd::capi ALIAS rcvd_capi)

# §4.1 / §16.2: the command-line front end. Rule A1 makes it thin — no equation
# and no unit conversion beyond calling io/display.
add_executable(rcvd-cli app/rcvd_cli.cpp)
target_link_libraries(rcvd-cli PRIVATE
  rcvd::core rcvd::models rcvd::analysis rcvd::io rcvd::tire rcvd::aero rcvd_flags)

# §4.1 / §16: the Qt Widgets front end. Optional, because the suite must build and
# test with no GUI toolkit present — §18 gates nothing on it, and CI machines that
# have no Qt still have to go green.
option(RCVD_BUILD_GUI "Build the Qt Widgets GUI (§16)" ON)
if(RCVD_BUILD_GUI)
  find_package(Qt6 6.5 QUIET COMPONENTS Widgets)
  if(Qt6_FOUND)
    set(CMAKE_AUTOMOC ON)   # the Q_OBJECT widgets are header-only
    # Headers are listed as sources so AUTOMOC sees their Q_OBJECT.
    add_executable(rcvd-gui WIN32
      app/gui/main.cpp
      app/gui/unit_field.hpp
      app/gui/plot_widget.hpp
      app/gui/program_spec.hpp
      app/gui/program_window.hpp
      app/gui/launcher.hpp)
    target_include_directories(rcvd-gui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/app/gui)
    target_link_libraries(rcvd-gui PRIVATE
      Qt6::Widgets
      rcvd::core rcvd::models rcvd::analysis rcvd::io rcvd::tire rcvd::aero rcvd_flags)
    message(STATUS "rcvd-gui: Qt ${Qt6_VERSION} at ${Qt6_DIR}")
  else()
    message(STATUS "rcvd-gui: Qt6 Widgets not found; skipping the GUI target. "
                   "Pass -DCMAKE_PREFIX_PATH=<Qt>/msvc2022_64 to build it.")
  endif()
endif()

if(RCVD_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
