#!/bin/sh
# Transcendental policy study of chunk P03 (see test/p03_transcendental_study.cpp and NOTES.md "Transcendental policy").
# x86-64, -O2, no blob needed.  Usage: sh test/p03_study.sh [samples-per-raw-sweep]   (default 2,000,000; ~3-4 minutes)
set -e
cd "$(dirname "$0")/.."
T="${TPREFIX:-test/p03study}"
g++ -m64 -O2 -Wall -Wextra -fno-exceptions -fno-rtti -Isrc -c src/dsp/dsp_window_fft_psd.cpp -o ${T}_port.o
g++ -m64 -O2 -Wall -Wextra -Wno-unused-parameter -fno-exceptions -fno-rtti -Isrc -c test/p03_transcendental_study.cpp -o ${T}_study.o
g++ -m64 -no-pie -o ${T}_bin ${T}_study.o ${T}_port.o -lm
case "$T" in /*) R="";; *) R="./";; esac
${R}${T}_bin "$@"
rc=$?
rm -f ${T}_port.o ${T}_study.o ${T}_bin
exit $rc
