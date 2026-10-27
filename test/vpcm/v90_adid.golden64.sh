#!/bin/sh
# Row p18 of class V90AutoDigitalImpDetector: golden check on x86-64 without the blob, see test/vpcm/v90_adid.shared.golden64.sh (ROW=p18).
cd "$(dirname "$0")/../.."
ROW=p18 exec sh test/vpcm/v90_adid.shared.golden64.sh "$@"
