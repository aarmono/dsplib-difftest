#!/bin/sh
# Logging compiler shim of tools/stampkeys.py (recipe recording; not meant to be called by hand).
# Installed by the recorder as gcc / g++ / cc / c++ (symlinks to this file) first in PATH of a job run in a private sandbox tree.
# It appends the REAL command line to $SK_LOG (one record per call: unit-separator-delimited stage, compiler name, cwd, arguments),
# runs the real compiler unchanged (so every later step of the script that needs the object sees a real one), and links every executable
# with $SK_DIR/exit0_<32|64>.o, a constructor that calls exit_group(0) before main: the test binaries are never really run while a recipe is
# recorded (their output is empty), only the compile steps matter.  cmp and diff (symlinks to this file) succeed silently, so the script follows the
# control flow of a passing run.
n=${0##*/}
case "$n" in cmp|diff) exit 0;; esac   # recording: the fake test binaries print nothing, so every output comparison is made to succeed (the control flow of a PASSING run)
real=/usr/bin/$n
[ -n "$SK_LOG" ] || exec "$real" "$@"
link=1; m=64
for a in "$@"; do
  case "$a" in
    -c|-S|-E|-fsyntax-only|-M|-MM|-shared|-r|-v|--version|-dumpmachine|-dumpversion|-print-*|-\#\#\#) link=;;
    -m32) m=32;;
  esac
done
printf '%s\n' "$(printf '\037%s' "$SK_STAGE" "$n" "$PWD" "$@")" >> "$SK_LOG"
if [ -n "$link" ]; then exec "$real" "$@" "$SK_DIR/exit0_$m.o"; fi
exec "$real" "$@"
