/*
 * X5: V32FP_status() on a REAL x86-64 engine (bugs.md V80).  The status word at status + 0xe (shorts 14 / 15) is "the AGC detect state differs from 1".
 * V32FP_status() used to read it from the flat engine image (V32Eng::agc[28]); on x86-64 the AGC object is in the tail behind the address slots
 * (V32E_AGC), so the flat bytes are never written and the word did not follow the AGC.  The i386 blob harnesses (v32, n01_v32status) cannot see that.
 * This test creates a real engine with V32FP_create() (the V.32 files and the FPM files of the engine, native), sets the detect state of the AGC object
 * through its named member (and poisons the flat image bytes with other values), calls V32FP_status() and checks the word, together with the AGC level
 * word (status[6] = FpmAgc::hist), which was already read through the accessor.
 * Built for x86-64 only by test/v32/v32_engine.status.native64.sh; the same source is compiled with -m32 -c as a syntax / static-assert check.
 * Exit 0 and "MATCH" iff every check holds.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "fpm/fpm.h"
#include "v32/v32.h"

extern V32Wrap* V32FP_create(int* args, int reserved);
extern void V32FP_delete(V32Wrap* obj);
extern int V32FP_status(V32Wrap* obj, short* status);

#ifdef __i386__
_Static_assert(offsetof(V32Eng, agc) + offsetof(FpmAgc, detect) == 0x1f4 && sizeof(FpmAgc) == 0x2c, "FpmAgc i386");
#else
_Static_assert(offsetof(V32Eng, agc) + 28 == 0x1f4, "the flat image offset the old code read");
_Static_assert(V32_AGC_TAIL + offsetof(FpmAgc, detect) == 0x5230 && sizeof(FpmAgc) == 0x38, "FpmAgc x86-64");
#endif

static int g_fail;
#define CHECK_EQ(a, b)                                                                                                                                       \
  do                                                                                                                                                         \
  {                                                                                                                                                          \
    long ga_ = (long)(a), gb_ = (long)(b);                                                                                                                   \
    if (ga_ != gb_)                                                                                                                                          \
    {                                                                                                                                                        \
      g_fail++;                                                                                                                                              \
      printf("FAIL %s:%d: %s = %ld, expected %ld (%s)\n", __FILE__, __LINE__, #a, ga_, gb_, ctx);                                                            \
    }                                                                                                                                                        \
  } while (0)

int main(void)
{
  static const int detects[] = {0, 1, 2, 5, -1};
  static const unsigned flats[] = {0u, 1u, 7u};
  int cfg[6];
  unsigned i, j;
  int calls = 0;

  for (i = 0; i < sizeof detects / sizeof detects[0]; i++)
    for (j = 0; j < sizeof flats / sizeof flats[0]; j++)
    {
      V32Wrap* obj;
      V32Eng* eng;
      short st[24];
      char ctx[80];
      memset(cfg, 0, sizeof cfg);
      cfg[1] = 100;
      cfg[2] = 60000;
      cfg[3] = 700;
      cfg[4] = 1;
      *(unsigned int*)&cfg[5] = 0x12c0u | (0x12c0u << 16);
      obj = V32FP_create(cfg, 0);
      eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
      snprintf(ctx, sizeof ctx, "detect=%d flat=%u", detects[i], flats[j]);

      /* the real AGC object: detect state and level; the flat image bytes of the i386 layout carry a different value (a read through them is wrong) */
      V32E_AGC(eng)->detect = detects[i];
      V32E_AGC(eng)->hist = (short)(0x1230 + i);
      memcpy(eng->agc + 28, &flats[j], 4);

      memset(st, 0x55, sizeof st);
      CHECK_EQ(V32FP_status(obj, st), 1);
      CHECK_EQ(*(int*)(st + 0xe), detects[i] != 1);
      CHECK_EQ(st[6], 0x1230 + (int)i);
      calls++;
      V32FP_delete(obj);
    }
  if (g_fail)
  {
    printf("MISMATCH: %d checks failed\n", g_fail);
    return 1;
  }
  printf("MATCH (%d engines)\n", calls);
  return 0;
}
