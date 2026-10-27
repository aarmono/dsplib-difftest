#!/bin/sh
# Row p18 of class V90AutoDigitalImpDetector: see test/vpcm/v90_adid.shared.sh (the shared script; ROW=p18).
cd "$(dirname "$0")/../.."
ROW=p18 exec sh test/vpcm/v90_adid.shared.sh "$@"
