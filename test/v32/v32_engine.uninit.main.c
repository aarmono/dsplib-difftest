/* N02: uninitialised-memory probe for a constructor.  Usage: v32_engine.uninit_<blob|port> FILLBYTE  -> hex dump of every sysdep_malloc'ed block after
 * V32FP_create (both roles) and a few V32FP_modem calls.  Run twice with different FILLBYTEs under `setarch -R` (no ASLR); a byte that differs between
 * the two dumps was never initialised by the constructor (and, if the blob's own dump is stable, the port reads/keeps uninitialised heap where the
 * blob does not).  Found because live slmodemd (dirty heap) diverged from the blob where the differential tests (calloc-like clean heap) did not. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define MAXA 256
static void *ap[MAXA]; static unsigned asz[MAXA]; static int na; static int fill;
void *sysdep_malloc(unsigned int size)
{
  void *p = malloc(size);
  memset(p, fill, size);
  if (na < MAXA) { ap[na] = p; asz[na++] = size; }
  return p;
}
extern void *V32FP_create(int *cfg6, int param2);
extern int V32FP_modem(void *obj, int *a2, short *a3, short *a4, short a5);
int main(int argc, char **argv)
{
  int role, i, k;
  fill = argc > 1 ? atoi(argv[1]) : 0;
  for (role = 0; role < 2; role++) {
    int cfg[6];
    void *o;
    int start = na;
    memset(cfg, 0, sizeof cfg);
    cfg[0] = role; cfg[1] = 100; cfg[2] = 60000; cfg[3] = 700; cfg[4] = 1;
    *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16);
    o = V32FP_create(cfg, 0);
    printf("role %d: %d allocations\n", role, na - start);
    for (i = start; i < na; i++) {
      printf("A%d size %u:", i - start, asz[i]);
      for (k = 0; k < (int)asz[i]; k++) { if (k % 64 == 0) printf("\n%04x:", k); printf(" %02x", ((unsigned char *)ap[i])[k]); }
      printf("\n");
    }
  }
  return 0;
}
