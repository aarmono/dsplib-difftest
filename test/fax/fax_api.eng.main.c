/* R30 ENG build of the differential harness (two FAX objects over the real class 1 layer, FAXVMI and fax modems of both sides): see test/fax/fax_api.main.c and test/fax/r30_eng.h.  A separate main so
 * that every build script / x86-64 golden run compiles the right binary without special -D flags. */
#define HARNESS_ENG 1
#include "fax_api.main.c"
