/* P45: sizeof of the heap objects the V90Modem / V92Modem constructors allocate (the harness expects the i386 values and maps the x86-64 sizes to them in the malloc log) */
#include "vpcm/pcm_modem.h"
extern "C" const unsigned p45_sizes[] = {
  sizeof(V90Parameters), sizeof(V90Phase2Info), sizeof(V90Jd), sizeof(V92Jd), sizeof(V90Modulator), sizeof(V90Demodulator),
  sizeof(V92Parameters), sizeof(V92Phase2Info), sizeof(V92CP), sizeof(V92MappingParams), sizeof(V92Modulator) };
