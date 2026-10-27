/*
 * Differential test harness for V.32/V.32bis's `VTBv32_init` (Viterbi-
 * decoder per-rate state initializer, src/v32/v32_datapath.c) - the first of
 * this subsystem's two functions to be hand-cleaned (`VTB_decoder`, the
 * per-symbol trellis decode step, remains a deliberate out-of-scope stub -
 * see NOTES.md's "V.32 VTB Viterbi decoder decompile-archaeology" and "V.32
 * VTB table extraction" entries for the two prior chunks this one builds
 * on).
 *
 * ==========================================================================
 * WHY A DIRECT CALL, NOT THROUGH SetRxModeV32
 * ==========================================================================
 * `VTBv32_init` is called for real from `SetRxModeV32`'s cases 3-6 (NOT
 * case 2 - see src/v32/v32_datapath.c's updated comment: case 2 wires up
 * `FSE_decision_16pt`, a non-trellis decision callback that never reaches
 * `VTBv32_init` at all - every OLDER comment in this tree that says "cases
 * 2-6" is imprecise, corrected as of this pass). Every V.32 differential
 * test built so far has deliberately steered clear of `SetRxModeV32`'s
 * modes 3-6 specifically BECAUSE `VTBv32_init` was an `abort()` stub until
 * now (see e.g. test/v32/v32_engine.org.main.c's own `SetTxModeV32`-only direct-
 * call sweep, which explicitly notes it does NOT drive `SetRxModeV32`'s
 * modes 2-6 for exactly this reason).
 *
 * This test follows the SAME "call the hand-cleaned function directly,
 * bypassing the not-fully-portable caller" precedent that direct-call sweep
 * already established for `SetTxModeV32` - `VTBv32_init` is called directly
 * with synthetic `(vtbSubObj, rate, param_3)` arguments, matching its real
 * signature exactly, sidestepping `SetRxModeV32`'s other still-real-but-
 * untested-together side effects. This is the primary (and, this pass,
 * ONLY completed) test approach - option (a) from this chunk's brief.
 * Driving `SetRxModeV32` end-to-end for modes 3-6 for real (option (b)) was
 * NOT attempted this pass; it remains a valid future bonus but is not
 * required and is not claimed here.
 *
 * `vtbSubObj` itself is obtained the SAME way the real object obtains it -
 * `*(int*)(engine+0x230)+0x18` off a real, fully-constructed `V32FP_create`
 * engine object (the FSE sub-object's own base address, already proven by
 * the prior `v32_equalizer.c` chunk's independent address-arithmetic proof to
 * equal `bigbuf+0x5038`) - so the scratch-buffer pointer `VTBv32_init`
 * reads/writes at `vtbSubObj+0x00` is a REAL, already-`sysdep_malloc`'d
 * 0x200-byte buffer (allocated by `V32FP_recreate`'s own inlined duplicate
 * of this exact logic at construction time), not a dangling/synthetic one -
 * this matters because `VTBv32_init`'s `param_3==0` path (the only one the
 * real `SetRxModeV32` call site ever uses) dereferences and rewrites
 * through that existing pointer without ever allocating it itself.
 *
 * ==========================================================================
 * WHAT IS COMPARED
 * ==========================================================================
 * For each of the 4 trellis-coded rate-family values `VTBv32_init`
 * dispatches on (rate==3 -> 7200bps/16T, rate==2 -> 9600bps/32pt,
 * rate==4 -> 12000bps/64pt, rate==5 -> 14400bps/128pt - NOTE the mapping is
 * NOT numerically monotonic, see src/v32/v32_datapath.c's header comment),
 * this test:
 *   1. Calls `VTBv32_init(vtbSubObj, rate, 0)` directly (the real call
 *      site's own `param_3` value).
 *   2. Dumps every scalar struct field the function writes
 *      (`vtbSubObj+0x04/0x14/0x24/0x2c/0x2e/0x30/0x32/0x34`).
 *   3. Dumps a checksum (+ first/last few elements) of the DEREFERENCED
 *      CONTENT (not the raw pointer value, which differs between the blob
 *      and port binaries by construction) of each of the 4 table pointers
 *      written (`+0x18`=IMAP, `+0x1c`=QMAP, `+0x20`=BOUND, `+0x28`=REGION),
 *      reading exactly the real, rate-family-correct element COUNT for
 *      each (128/416/960/1856 for BOUND, 8/32/72/128 for REGION, 16/32/64/
 *      128 for IMAP/QMAP, matching src/v32/v32_viterbi_tables.h's already-
 *      extracted, already-byte-exact-verified table sizes) - this proves
 *      `VTBv32_init` selects the CORRECT table for the CORRECT rate family
 *      (a wrong-table-selected bug would show as a checksum mismatch even
 *      though the individual tables were already separately verified
 *      byte-exact), not just that the tables themselves are correct.
 *   4. Dumps a checksum of the (always fully re-zeroed, per real call) 16-
 *      slot/0x200-byte scratch history buffer, as a sanity check that the
 *      RIGHT buffer got zeroed (not a stray/wrong address).
 *
 * `rate==2/3/4/5` are the 4 values `SetRxModeV32`'s own real call site
 * (cases 3-6) ever passes. As an HONEST bonus (clearly separated in the
 * output, not conflated with the real-call-site coverage above), this test
 * ALSO exercises `rate==1` (never reached in practice - `SetRxModeV32`
 * never calls `VTBv32_init` with a rate below 2 - but a real, reachable
 * branch of the function's own dispatch logic: `rateS<4 && rateS!=2`
 * defaults to the 14400bps/128pt family, same as `rate==5`/`rate>=6`) and
 * `param_3==1` (the `fresh`-malloc branch, ALSO never reached by the real
 * call site, which always passes literal `0` - exercised here on a
 * separate, freshly-allocated, zeroed local struct so it cannot disturb the
 * main `vtbSubObj` used for the real-call-site-matching coverage above).
 * Both are real, disassembly-confirmed-reachable code paths of the function
 * under test (not dead code in the FUNCTION - only dead at its one current
 * caller), so covering them is genuine extra confidence, not padding.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v32_hobj.h"

extern void *V32FP_create(int *cfg6, int param2);
extern void V32FP_delete(char *obj);
extern void VTBv32_init(char *vtbSubObj, unsigned int rate, int param3);

#define S16(o) (*(short *)(intptr_t)(o))
#define U16(o) (*(unsigned short *)(intptr_t)(o))
#define S32(o) (*(int *)(intptr_t)(o))

/* Rate-family table-size table, matching src/v32/v32_viterbi_tables.h's already-
 * extracted, already-verified-byte-exact sizes (element counts, not
 * bytes) - indexed by `VTBv32_init`'s own `rate` argument value (only
 * 2/3/4/5 are real dispatch values; 1 shares family 4's sizes since it
 * falls into the same default branch as 5). */
struct rate_info {
  unsigned int rate;
  const char *label;
  int imapQmapCount;
  int boundCount;
  int regionCount;
};

static const struct rate_info RATES[] = {
  { 3, "7200/16T",   16,  128,  8 },
  { 2, "9600/32pt",  32,  416,  32 },
  { 4, "12000/64pt", 64,  960,  72 },
  { 5, "14400/128pt",128, 1856, 128 },
  /* bonus: dead-at-real-call-site coverage of the default branch via a
   * DIFFERENT input value than 5 (rate==1 -> same family 4 tables) */
  { 1, "14400/128pt(rate=1,bonus)", 128, 1856, 128 },
};
#define NUM_RATES (int)(sizeof(RATES) / sizeof(RATES[0]))

/* Sum-of-unsigned-shorts checksum over a dereferenced table pointer's
 * content - a portable stand-in for "compare the pointee bytes" that
 * doesn't require printing hundreds/thousands of individual values
 * (VTB_BOUND_14400 alone is 1856 elements) while still being sensitive to
 * ANY byte difference anywhere in the table. */
static unsigned int table_checksum(const short *ptr, int count)
{
  unsigned int sum = 0;
  int i;
  for (i = 0; i < count; i++) {
    sum = sum * 131u + (unsigned short)ptr[i];
  }
  return sum;
}

static void dump_scalars(const char *label, char *vtbSubObj)
{
  printf("[%-28s] scalars(04,14,24,2c,2e,30,32,34)=%d,%d,%d,%d,%d,%d,%u,%d\n",
         label,
         (int)S16(vtbSubObj + 0x04), (int)S16(vtbSubObj + 0x14),
         (int)S16(vtbSubObj + 0x24), (int)S16(vtbSubObj + 0x2c),
         (int)S16(vtbSubObj + 0x2e), (int)S16(vtbSubObj + 0x30),
         (unsigned)U16(vtbSubObj + 0x32), (int)S16(vtbSubObj + 0x34));
}

static void dump_tables(const char *label, char *vtbSubObj, const struct rate_info *ri)
{
  /* the four table words are CFGP handles on x86-64 (src/core/dsp_table_handles.h; the address at -m32) */
  const short *imapPtr = (const short *)cfgp_ptr(S32(vtbSubObj + 0x18));
  const short *qmapPtr = (const short *)cfgp_ptr(S32(vtbSubObj + 0x1c));
  const short *boundPtr = (const short *)cfgp_ptr(S32(vtbSubObj + 0x20));
  const short *regionPtr = (const short *)cfgp_ptr(S32(vtbSubObj + 0x28));

  printf("[%-28s] imap(n=%d,csum=%u,first=%d,last=%d) qmap(csum=%u,first=%d,last=%d) "
         "bound(n=%d,csum=%u,first=%d,last=%d) region(n=%d,csum=%u,first=%d,last=%d)\n",
         label,
         ri->imapQmapCount, table_checksum(imapPtr, ri->imapQmapCount),
         (int)imapPtr[0], (int)imapPtr[ri->imapQmapCount - 1],
         table_checksum(qmapPtr, ri->imapQmapCount),
         (int)qmapPtr[0], (int)qmapPtr[ri->imapQmapCount - 1],
         ri->boundCount, table_checksum(boundPtr, ri->boundCount),
         (int)boundPtr[0], (int)boundPtr[ri->boundCount - 1],
         ri->regionCount, table_checksum(regionPtr, ri->regionCount),
         (int)regionPtr[0], (int)regionPtr[ri->regionCount - 1]);
}

static void dump_scratch(const char *label, char *vtbSubObj)
{
  const short *scratch = V32X_PG(const short *, vtbSubObj, V32_VTB_OFF, 0x00);
  unsigned int csum = table_checksum(scratch, 0x100 /* 0x200 bytes / 2 */);
  printf("[%-28s] scratch_nonnull=%d scratch_csum(should be 0)=%u\n",
         label, scratch != 0, csum);
}

int main(void)
{
  int cfg[6];
  void *p;
  char *obj, *engine, *vtbSubObj;
  int i;

  memset(cfg, 0, sizeof(cfg));
  cfg[0] = 0;              /* role: Originate */
  cfg[1] = 100;             /* look-ahead */
  cfg[2] = 60000;           /* timeout */
  cfg[3] = 700;             /* margin */
  cfg[4] = 1;               /* enable word, bit0 set */
  *(unsigned int *)&cfg[5] = 0x12c0u | (0x12c0u << 16); /* tx rate 4800, both halves */

  p = V32FP_create(cfg, 0);
  if (!p) {
    printf("create failed\n");
    return 1;
  }
  obj = (char *)(p);
  engine = V32W_PG(char *, obj, 0x68);
  vtbSubObj = V32E_PG(char *, engine, 0x230) + 0x18;

  printf("== VTBv32_init direct-call sweep (real call-site values: rate=2,3,4,5) ==\n");
  for (i = 0; i < NUM_RATES; i++) {
    const struct rate_info *ri = &RATES[i];
    char tag[64];

    VTBv32_init(vtbSubObj, ri->rate, 0);

    snprintf(tag, sizeof(tag), "rate=%u(%s)", ri->rate, ri->label);
    dump_scalars(tag, vtbSubObj);
    dump_tables(tag, vtbSubObj, ri);
    dump_scratch(tag, vtbSubObj);
  }

  /* ---- bonus: param_3==1 (fresh-malloc) branch, on a SEPARATE, harness-
   * owned, zeroed struct region so it cannot disturb vtbSubObj above. Only
   * fields up to +0x34 (inclusive, 2 bytes) are ever written -
   * 0x40 bytes is comfortably sized. ---- */
  {
    /* x86-64: the VTB object must sit at its blob offset in an engine allocation (its scratch-buffer slot is an engine-tail slot, src/v32/v32.h) */
    unsigned char *freshEng = (unsigned char *)calloc(1, V32_ENG_ALLOC);
    unsigned char *freshBuf = freshEng + V32_VTB_OFF;
    char *freshObj = (char *)(freshBuf);
    memset(freshBuf, 0, 0x40);

    printf("== VTBv32_init param_3=1 (fresh-malloc branch, bonus/dead-at-real-call-site) ==\n");
    for (i = 0; i < NUM_RATES; i++) {
      const struct rate_info *ri = &RATES[i];
      char tag[64];

      VTBv32_init(freshObj, ri->rate, 1);

      snprintf(tag, sizeof(tag), "fresh,rate=%u(%s)", ri->rate, ri->label);
      dump_scalars(tag, freshObj);
      dump_tables(tag, freshObj, ri);
      dump_scratch(tag, freshObj);
    }
  }

  V32FP_delete(obj);
  printf("== done ==\n");
  return 0;
}
