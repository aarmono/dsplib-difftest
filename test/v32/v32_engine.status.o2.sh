#!/bin/sh
# Job n01_v32status_O2: the noisy-state V32FP_status test of test/v32/v32_engine.status.sh with the port built at -O2 (the shared script takes the optimisation level as $1).
cd "$(dirname "$0")/../.."
TPREFIX="${TPREFIX:-test/.build/v32/v32_engine.status.o2}" exec sh test/v32/v32_engine.status.sh -O2
