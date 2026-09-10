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
  test_bicycle.cpp
  test_load_transfer.cpp
  test_four_wheeler.cpp
  test_ride_roll.cpp
  test_moment_method.cpp
  test_lap_solver.cpp
  test_io.cpp
  test_quantity.cpp)

target_include_directories(rcvd_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(rcvd_tests PRIVATE
  rcvd::units rcvd::core rcvd::numerics rcvd::tire rcvd::models
  rcvd::aero rcvd::analysis rcvd::io
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
endif()
