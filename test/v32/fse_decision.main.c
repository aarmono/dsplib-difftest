/*
 * N04: differential test of the V.32bis FSE decision callbacks that write BOTH outputs (*angleInOut, *magInOut):
 * FSE_decision_16Tpt (7200, MAG9600[idx-1]), _32pt (9600 trellis), _64pt (12000), _128pt (14400), driven DIRECTLY
 * over the whole input range, comparing every output (return value, angle, mag, the smoothing/lock state in the
 * fse history object, the VTB path metrics / write index / history checksum).
 *
 * Why: the older tests compared only angle + the VTB result and treated the magnitude as dead, but FPM_FSE_receive
 * uses it after the callback (asStack_16[0] -> equaliser error), and the port had an all-zero "dead placeholder"
 * for FSE_decision_16Tpt (found by N03/N04: V.32bis 7200 failed live).  FSE_decision_16pt (4800) is NOT callable
 * here: the vendor's index (|I|+|Q|)>>1 reads up to 24 KB past DECv32_MAG9600 and crashes in a sparse process.
 *
 * Inputs per function: (a) every constellation point of the rate's own map +- {0, small offsets} (exact hits, so
 * every table entry is read), (b) a 40x40 grid over +-32767 (all quadrants, all regions), (c) 600 LCG points.
 * argv[1] = function index 0..3.  Output is one line per call (hashed state) + a mag histogram summary.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v32_hobj.h"

extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(char *obj);
extern void VTBv32_init(char *vtbSubObj, unsigned int rate, int param_3);
extern void VTB_decoder(char *vtbSubObj, short i, short q, short *outBits);
extern short FSE_decision_16Tpt(char *fseObj, short *angleInOut, short *magInOut);
extern short FSE_decision_32pt(char *fseObj, short *angleInOut, short *magInOut);
extern short FSE_decision_64pt(char *fseObj, short *angleInOut, short *magInOut);
extern short FSE_decision_128pt(char *fseObj, short *angleInOut, short *magInOut);
extern const short DECv32_IMAP16[16], DECv32_QMAP16[16];
extern const short DECv32_IMAP64[64], DECv32_QMAP64[64];

#define S16(o) (*(short *)(intptr_t)(o))
#define S32(o) (*(int *)(intptr_t)(o))

static unsigned int st;
static short lcg(void) { st = st * 1103515245u + 12345u; return (short)(st >> 16); }

struct fn { const char *name; short (*f)(char *, short *, short *); unsigned rate; };
static const struct fn FN[] = {
  { "16Tpt", FSE_decision_16Tpt, 3 }, { "32pt", FSE_decision_32pt, 2 },
  { "64pt", FSE_decision_64pt, 4 }, { "128pt", FSE_decision_128pt, 5 },
};

static unsigned hsh(const void *p, int n)
{
  const unsigned char *c = p; unsigned h = 2166136261u; int i;
  for (i = 0; i < n; i++) h = (h ^ c[i]) * 16777619u;
  return h;
}

int main(int argc, char **argv)
{
  int k = argc > 1 ? atoi(argv[1]) : 0;
  const struct fn *fn = &FN[k];
  int cfg[6]; void *p; char *obj, *engine, *vtbSub;
  /* x86-64: the FSE object (+0x204), its history / VTB object (+0x5020) and the address slots between them (+0x230 hist, +0x258 / +0x25c the I / Q sample
   * buffers, +0x5038 the VTB scratch buffer) live in ONE engine allocation (src/v32/v32.h): a private one here, the real object only donates its VTB state */
  unsigned char *eng = calloc(1, V32_ENG_ALLOC + 0x100);
  unsigned char *hist = eng + 0x5020;
  unsigned char *fse = eng + 0x204;
  short *iq = calloc(2, sizeof(short)); /* iq[0] I, iq[1] Q */
  short *pi = calloc(1, 2), *pq = calloc(1, 2);
  int calls = 0, magn = 0; short magseen[64]; int i, j, dx, dy;
  static const int off[5][2] = { {0,0}, {1,0}, {0,-1}, {37,-29}, {-300,411} };

  (void)iq;
  memset(cfg, 0, sizeof(cfg));
  cfg[1] = 100; cfg[2] = 60000; cfg[3] = 700; cfg[4] = 1;
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16);
  p = V32FP_create(cfg, 0);
  if (!p) { printf("create failed\n"); return 1; }
  obj = (char *)(p); engine = V32W_PG(char *, obj, 0x68); vtbSub = V32E_PG(char *, engine, 0x230) + 0x18;
  VTBv32_init(vtbSub, fn->rate, 0);
  memcpy(hist + 0x18, (void *)(intptr_t)vtbSub, 0x38);
  V32E_PS(void *, (char *)(eng), 0x5038, V32X_PG(void *, vtbSub, V32_VTB_OFF, 0)); /* the VTB scratch-buffer slot of the copy */
  V32E_PS(void *, (char *)(eng), 0x230, hist);
  V32E_PS(void *, (char *)(eng), 0x258, pi);
  V32E_PS(void *, (char *)(eng), 0x25c, pq);
  S16(fse + 0x5e) = 0;
  st = 0x1234567u + k;
  printf("== FSE_decision_%s ==\n", fn->name);

#define CALL(I, Q) do { \
    short ang = 0x5a5a, mag = 0x3c3c, ret; \
    pi[0] = (short)(I); pq[0] = (short)(Q); \
    ret = fn->f((char *)(fse), &ang, &mag); \
    printf("%s %5d %5d ret=%d ang=%d mag=%d hist=%08x vtb=%08x\n", fn->name, (int)pi[0], (int)pq[0], (int)ret, (int)ang, (int)mag, \
           hsh(hist + 0x50, 0x24), hsh(hist + 0x1c, 0x12) ^ hsh(hist + 0x48, 2) ^ hsh(V32E_PG(void *, (char *)(eng), 0x5038), 0x200)); \
    { int m; for (m = 0; m < magn; m++) if (magseen[m] == mag) break; if (m == magn && magn < 64) magseen[magn++] = mag; } \
    calls++; } while (0)

  /* (a) exact constellation points (+ offsets).  The 32pt map is the 16pt-style 9600 map; 64/128 use the larger maps: sweep a fine lattice too. */
  for (i = 0; i < 16; i++)
    for (j = 0; j < 5; j++) CALL(DECv32_IMAP16[i] + off[j][0], DECv32_QMAP16[i] + off[j][1]);
  for (i = 0; i < 64; i++)
    for (j = 0; j < 5; j++) CALL(DECv32_IMAP64[i] + off[j][0], DECv32_QMAP64[i] + off[j][1]);
  /* (b) grid */
  for (dx = 0; dx < 40; dx++)
    for (dy = 0; dy < 40; dy++) CALL(-32767 + dx * 1680 + (dy & 1) * 7, -32767 + dy * 1680 + (dx & 1) * 11);
  /* (c) random */
  for (i = 0; i < 600; i++) { short a = lcg(), b = lcg(); CALL(a, b); }
  printf("calls=%d distinct_mag=%d:", calls, magn);
  for (i = 0; i < magn; i++) printf(" %d", magseen[i]);
  printf("\n");
  V32FP_delete(obj);
  return 0;
}
