#!/usr/bin/env python3
"""Spec SS18.3 gate: the GUI, the CLI and the C API must agree BIT-FOR-BIT.

    SS18.3  "Any behaviour reachable through the GUI SHALL be reachable through
             rcvd-cli and through the C API with identical results, bit-for-bit."

tests/test_capi.cpp closes the C++/C half of that. This script closes the
GUI/CLI half, which cannot be done from inside the Catch2 suite because the GUI
links Qt and runs as a separate process.

It never retypes a number. The GUI emits both halves of the comparison itself:

    rcvd-gui --dump-vehicle P1          the SS5.1 document the GUI would save
    rcvd-gui --dump-cli P1 <doc>        the command line its Run tab displays
    rcvd-gui --selftest P1              its own results at 17 significant figures

The CLI is then run on exactly that command line with --digits 17, and the two
sets of digits are compared as STRINGS. 17 significant figures round-trip a
binary64 exactly, so string equality here is bit equality.

Usage:
    python tools/check_front_ends.py <path-to-rcvd-gui> <path-to-rcvd-cli>

Exit status 0 when every shared quantity agrees, 1 otherwise.
"""
import os
import subprocess
import sys
import tempfile

# The CLI's label for a quantity, and the GUI's. They differ because the GUI
# names the defining equation and the CLI does not; the mapping is written out
# rather than guessed so that a renamed row fails loudly instead of silently
# dropping out of the comparison.
P1_ROWS = {
    "Y_beta": "Y_beta",
    "Y_r": "Y_r",
    "Y_delta": "Y_delta",
    "N_beta": "N_beta",
    "N_r": "N_r",
    "N_delta": "N_delta",
    "K_us": "K_us (8.18)",
    "NSP": "NSP (8.22)",
    "SM": "SM (8.23)",
    "V_ch": "V_ch (8.19)",
    "V_tan": "V_tan (8.21)",
}

P4_ROWS = {
    "time": "lap time",
    "event": "termination event",
}


def run(argv):
    r = subprocess.run(argv, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write("command failed (%d): %s\n%s%s\n"
                         % (r.returncode, " ".join(argv), r.stdout, r.stderr))
        sys.exit(1)
    return r.stdout


def parse_cli(text):
    """'  Y_beta   -310934.47636981681  N/rad' -> {'Y_beta': ('-310934...', 'N/rad')}"""
    out = {}
    for line in text.splitlines():
        if not line.startswith("  ") or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        name = parts[0]
        if name == "yaw" and len(parts) > 2:      # "yaw rate <v> <unit>"
            name, parts = "yaw_rate", [name] + parts[2:]
        out[name] = (parts[1], parts[2] if len(parts) > 2 else "-")
    return out


def parse_gui(text, program):
    out = {}
    for line in text.splitlines():
        f = line.split("\t")
        if len(f) < 4 or f[0] != program:
            continue
        out[f[1]] = (f[2], f[3])
    return out


def compare(program, cli, gui, rows):
    failures = []
    for cli_name, gui_name in sorted(rows.items()):
        if cli_name not in cli:
            failures.append("%s: %r missing from the CLI output" % (program, cli_name))
            continue
        if gui_name not in gui:
            failures.append("%s: %r missing from the GUI output" % (program, gui_name))
            continue
        cv, cu = cli[cli_name]
        gv, gu = gui[gui_name]
        if cv != gv:
            failures.append("%s %s: CLI %s != GUI %s" % (program, cli_name, cv, gv))
        elif cu != gu and cu != "-" and gu != "-":
            failures.append("%s %s: same value, different unit - CLI %r, GUI %r"
                            % (program, cli_name, cu, gu))
        else:
            print("  ok  %-10s %-22s %s %s" % (program, cli_name, cv, cu))
    return failures


def check(gui_exe, cli_exe, program, subcommand, rows, tmpdir):
    doc = os.path.join(tmpdir, program + "_vehicle.json")
    with open(doc, "w", encoding="utf-8") as f:
        f.write(run([gui_exe, "--dump-vehicle", program]))

    argv = run([gui_exe, "--dump-cli", program, doc]).strip().split(" ")
    if argv[0] != subcommand:
        return ["%s: expected the %r subcommand, got %r" % (program, subcommand, argv[0])]

    cli = parse_cli(run([cli_exe] + argv + ["--digits", "17"]))
    gui = parse_gui(run([gui_exe, "--selftest", program]), program)
    if not cli or not gui:
        return ["%s: one side produced no parseable rows" % program]
    return compare(program, cli, gui, rows)


def main():
    if len(sys.argv) != 3:
        sys.stderr.write(__doc__)
        return 2
    gui_exe, cli_exe = sys.argv[1], sys.argv[2]
    for exe in (gui_exe, cli_exe):
        if not os.path.isfile(exe):
            sys.stderr.write("not found: %s\n" % exe)
            return 1

    failures = []
    with tempfile.TemporaryDirectory() as tmpdir:
        failures += check(gui_exe, cli_exe, "P1", "derivs", P1_ROWS, tmpdir)
        failures += check(gui_exe, cli_exe, "P4", "lap", P4_ROWS, tmpdir)

    if failures:
        print("\nSS18.3 FAILED")
        for f in failures:
            print("  " + f)
        return 1
    print("\nSS18.3: the GUI and the CLI agree to 17 significant figures "
          "on every shared quantity.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
