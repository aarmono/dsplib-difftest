/* R27 ENG build of the differential harness (the real engines behind the ops tables): see test/fax/fax_vmi.main.c.  A separate main so that every build script / x86-64 golden run compiles the right binary
 * without special -D flags. */
#define HARNESS_ENG 1
#include "fax_vmi.main.c"
