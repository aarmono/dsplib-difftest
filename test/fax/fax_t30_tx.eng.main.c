/* R28 ENG build of the differential harness (the real FAXVMI layer and the real fax modems behind the T.30 transmit states): see test/fax/fax_t30_tx.main.c and test/fax/r28_eng.h.  A separate main so
 * that every build script / x86-64 golden run compiles the right binary without special -D flags. */
#define HARNESS_ENG 1
#include "fax_t30_tx.main.c"
