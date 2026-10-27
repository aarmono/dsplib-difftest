#!/bin/sh
# flavour A of the P29 test (scenarios 0, 1: V90BitsToSymbol / V92BitsToSymbol over recorder callees) under the names tools/mutate.py expects (see pcm_bits_to_symbol.main.sh)
cd "$(dirname "$0")/../.."
# the golden test/golden/vpcm/pcm_bits_to_symbol.main.blob32.out belongs to the job vpcm/pcm_bits_to_symbol.main (both flavours' scenarios); a flavour alone would overwrite it with half of them
[ -n "$GOLDEN_UPDATE" ] && { echo "GOLDEN_UPDATE=1 is refused for vpcm/pcm_bits_to_symbol.a: its golden is vpcm/pcm_bits_to_symbol.main's (both flavours); run GOLDEN_UPDATE=1 sh test/vpcm/pcm_bits_to_symbol.main.sh" >&2; exit 1; }
P29_FLAV=A exec sh test/vpcm/pcm_bits_to_symbol.main.sh "$@"
