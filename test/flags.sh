# X1: The pinned compiler flag set of the library under test: DEFINED in the library's flags.mk (the Makefile of the library uses it too); this file only reads it (sourced by test/cross/h01_datasym.build_objs.sh and tools/run_changed.sh, exported to the
# test scripts through test/flagshim).  POSIX sh, no side effects.
#
# What the port is bit-exact for (every claim in NOTES.md names its flags; this is the definition of the words used there):
#   PLAIN   = the scripts' own flags: 32-bit -O0 and -O2 (g++ -fno-exceptions -fno-rtti for the C++ rows), no further -f flag.
#             Tested by stage 1 of tools/run_changed.sh (blob vs port, 32-bit) and, for the 37 plain-C tiers, stage 1b (32-bit -O2 vs the blob golden).
#   PINNED  = PLAIN + $PIN_FLAGS at -O2.  Tested by stage 1p (the same scripts with PORT_XFLAGS=$PIN_FLAGS) and stage 1b-pinned.
#   x86-64  = default SSE flags -O2 (stage 2, all 91 goldens), optional matrix (run_changed.sh --matrix): -O0, -O2 -mfpmath=387, one -O3.
# NOT claimed / never run: -O1, -Os, 32-bit -O3, 32-bit -O2 -mfpmath=sse (the 32-bit -O3 / -O1 / -Os results of the older chunks in NOTES.md are historical).
#
# PIN_FLAGS comes from the library (flags.mk, a symlink into ext/dsplib; the file explains why the flag is pinned).  Source this file from the repository root.
PIN_FLAGS=$(make -s -f flags.mk print-pin)
