# Compile-failure cases

Rule U6 (§3.1) says the type system must make `Newton_per_radian + Newton` **fail
to compile**. A test that must fail to build cannot live in a passing translation
unit, so each case here is a standalone `.cpp` that the build is required to
*reject*.

`tools/check_compile_fail.py` compiles each one and fails if any of them succeeds.
