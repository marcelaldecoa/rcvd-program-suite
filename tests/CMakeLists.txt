include(FetchContent)

# §4.2: Catch2 v3.
FetchContent_Declare(Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.5.2)
FetchContent_MakeAvailable(Catch2)

add_executable(rcvd_tests
  test_units.cpp
  test_core.cpp
  test_numerics.cpp
  test_tire.cpp
  test_tire_models.cpp
  test_bicycle.cpp
  test_load_transfer.cpp
  test_four_wheeler.cpp
  test_ride_roll.cpp
  test_moment_method.cpp
  test_lap_solver.cpp
  test_transient.cpp
  test_io.cpp
  test_quantity.cpp
  test_display.cpp
  test_capi.cpp
  test_diagnostic_coverage.cpp)

target_include_directories(rcvd_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(rcvd_tests PRIVATE
  rcvd::units rcvd::core rcvd::numerics rcvd::tire rcvd::models
  rcvd::aero rcvd::analysis rcvd::io rcvd::capi
  rcvd_flags
  Catch2::Catch2WithMain)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(rcvd_tests)

# Rule U6 (§3.1) requires unit-mismatched arithmetic to FAIL to compile. A test
# that must fail to build cannot live in a passing translation unit, so each case
# in compile_fail/ is compiled on its own and the build is required to reject it.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
  add_test(NAME rule_u6_compile_failures
           COMMAND ${Python3_EXECUTABLE}
                   ${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_compile_fail.py
                   --compiler ${CMAKE_CXX_COMPILER}
                   --include  ${CMAKE_CURRENT_SOURCE_DIR}/../include
                   --dir      ${CMAKE_CURRENT_SOURCE_DIR}/compile_fail)

  # §18.3 requires the GUI, the CLI and the C API to agree bit-for-bit. The
  # C++/C half is tests/test_capi.cpp; the GUI half cannot live inside the
  # Catch2 suite, because the GUI links Qt and must be driven as a process.
  # Registered only when the GUI was built, so a Qt-less CI run still goes green.
  if(TARGET rcvd-gui)
    add_test(NAME front_ends_agree_bit_for_bit
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_front_ends.py
                     $<TARGET_FILE:rcvd-gui> $<TARGET_FILE:rcvd-cli>)
    # The GUI runs headless here (--selftest opens no window), but it still needs
    # the Qt DLLs beside it or on PATH.
    set_tests_properties(front_ends_agree_bit_for_bit PROPERTIES
                         ENVIRONMENT "PATH=$<SHELL_PATH:${QT6_INSTALL_PREFIX}/bin>;$ENV{PATH}")
  endif()
endif()
