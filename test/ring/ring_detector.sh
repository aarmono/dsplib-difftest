#!/bin/sh
# Builds both sides of the RingDetector_* differential test (one linked
# against the real dsplibs.o, one against the hand-cleaned src/) as 32-bit
# binaries, runs them, and diffs their output. Exit status is diff's: 0 means
# the port is behaviorally identical to the blob for this test's inputs.
#
# Usage: sh test/ring/ring_detector.sh   (job "rd"; run from slmodemd/dsplibs-port/)
# Build artifacts go to test/.build/ring/ (ignored by git); TPREFIX overrides the prefix.

set -e
cd "$(dirname "$0")/../.."
. test/blob.sh   # BLOB_O: the blob oracle (dsplibs.o), see test/blob.sh
T="${TPREFIX:-test/.build/ring/ring_detector}"
mkdir -p "$(dirname "$T")"

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_blob \
    test/ring/ring_detector.main.c test/stub_env.c $BLOB_O

gcc -m32 -Isrc -Wall -Wextra -O0 -g -o ${T}_test_port \
    test/ring/ring_detector.main.c test/stub_env.c src/ring/ring_detector.c

# X1: the port is also built at -O2 (and -O3): the strict-aliasing defect of ring_detector.c (the Process state read as int what Create/Process write
# as short) was invisible at -O0 and only showed at -O2.  Extra flags for the port come from the environment (PORT_XFLAGS via test/flagshim).
gcc -m32 -Isrc -Wall -Wextra -O2 -g -o ${T}_test_port_o2 \
    test/ring/ring_detector.main.c test/stub_env.c src/ring/ring_detector.c

gcc -m32 -Isrc -Wall -Wextra -O3 -g -o ${T}_test_port_o3 \
    test/ring/ring_detector.main.c test/stub_env.c src/ring/ring_detector.c

${T}_test_blob > ${T}_blob.out
${T}_test_port > ${T}_port.out
${T}_test_port_o2 > ${T}_port_o2.out
${T}_test_port_o3 > ${T}_port_o3.out

diff -u ${T}_blob.out ${T}_port_o2.out || { echo "MISMATCH: port at -O2"; exit 1; }
diff -u ${T}_blob.out ${T}_port_o3.out || { echo "MISMATCH: port at -O3"; exit 1; }
diff -u ${T}_blob.out ${T}_port.out && echo "MATCH: port is behaviorally identical to dsplibs.o for this test"
