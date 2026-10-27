#!/bin/sh
# Row p19 of class V90AutoDigitalImpDetector: golden check on x86-64 without the blob, see test/vpcm/v90_adid.shared.golden64.sh (ROW=p19).
cd "$(dirname "$0")/../.."
ROW=p19 exec sh test/vpcm/v90_adid.shared.golden64.sh "$@"
