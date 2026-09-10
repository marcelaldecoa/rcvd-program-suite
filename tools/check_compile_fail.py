#!/usr/bin/env python3
"""Verify that every tests/compile_fail/*.cpp is REJECTED by the compiler.

Rule U6 (spec 3.1) requires the dimensional type system to make unit-mismatched
arithmetic fail to build. A test that must fail to build cannot live in a passing
translation unit, so each case is compiled on its own here and the build is
required to reject it.

Usage:
    python tools/check_compile_fail.py --compiler cl   [--include include]
    python tools/check_compile_fail.py --compiler g++
"""
import argparse
import pathlib
import subprocess
import sys
import tempfile


def command(compiler: str, src: pathlib.Path, include: pathlib.Path, out: pathlib.Path):
    if pathlib.Path(compiler).name.lower().startswith("cl"):
        return [compiler, "/nologo", "/std:c++20", "/EHsc", "/permissive-", "/c",
                f"/I{include}", f"/Fo{out}", str(src)]
    return [compiler, "-std=c++20", "-fsyntax-only", f"-I{include}", str(src)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--compiler", default="c++")
    ap.add_argument("--include", default="include")
    ap.add_argument("--dir", default="tests/compile_fail")
    args = ap.parse_args()

    include = pathlib.Path(args.include).resolve()
    cases = sorted(pathlib.Path(args.dir).glob("*.cpp"))
    if not cases:
        print(f"no compile-fail cases found in {args.dir}", file=sys.stderr)
        return 1

    failures = []
    with tempfile.TemporaryDirectory() as tmp:
        for src in cases:
            out = pathlib.Path(tmp) / (src.stem + ".obj")
            proc = subprocess.run(command(args.compiler, src, include, out),
                                  capture_output=True, text=True)
            if proc.returncode == 0:
                failures.append(src)
                print(f"  UNEXPECTEDLY COMPILED  {src.name}")
            else:
                print(f"  correctly rejected     {src.name}")

    print()
    if failures:
        print(f"{len(failures)} of {len(cases)} cases compiled but must not have.")
        return 1
    print(f"all {len(cases)} compile-failure cases correctly rejected")
    return 0


if __name__ == "__main__":
    sys.exit(main())
