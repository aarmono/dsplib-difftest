#!/bin/sh
# Row p19 of class V90AutoDigitalImpDetector: see test/vpcm/v90_adid.shared.sh (the shared script; ROW=p19).
cd "$(dirname "$0")/../.."
ROW=p19 exec sh test/vpcm/v90_adid.shared.sh "$@"
