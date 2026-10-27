#!/bin/sh
# Row p20 of class V90AutoDigitalImpDetector: see test/vpcm/v90_adid.shared.sh (the shared script; ROW=p20).
cd "$(dirname "$0")/../.."
ROW=p20 exec sh test/vpcm/v90_adid.shared.sh "$@"
