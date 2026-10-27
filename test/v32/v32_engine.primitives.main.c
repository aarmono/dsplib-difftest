/*
 * Differential test harness for the V.32/V.32bis primitive/DSP toolkit
 * hand-clean pass (2026-09-29): PPS/SRE/ECC/FSE subsystem init/free/per-
 * sample bodies, the FSE_decision_* callback family, the scrambler/
 * descrambler + SMC constellation encoders, FPM_atan/FPM_lmsupd, and the
 * mid-level V32 wrapper helpers (minus ModDataV32/DemodDataV32's full
 * multi-subsystem wiring - see NOTES.md for what's covered vs. deferred).
 *
 * Same pattern as test/fpm/fpm_toolkit.main.c: this ONE source is compiled into
 * two binaries - v32_primitives_test_blob (linked against the real
 * dsplibs.o, whose functions under test are all globally exported per
 * `nm ../dsplibs.o`) and v32_primitives_test_port (linked against
 * ../src/v32/v32_*.c). Per fpm_diff_main.c's own established precedent, config
 * arrays passed into the _init functions reference the TABLE SYMBOLS BY
 * EXTERN NAME (PPSv32_ICOFFS, SREv32_COFFS, ECCv32_IMAP, FSEv32_ICOFF,
 * MRFv32_COFFS, ...) rather than duplicating their content locally - these
 * are real GLOBAL symbols in dsplibs.o (confirmed via `nm`), so the BLOB
 * build resolves them directly from dsplibs.o itself (this file does NOT
 * link src/v32/v32_tables.c into the blob build - doing so would duplicate-
 * symbol-collide with dsplibs.o's own copies, exactly the concern
 * fpm_diff_main.c's header comment already documents for its own tables),
 * while the PORT build resolves them from src/v32/v32_tables.c/
 * src/v32/v32_viterbi_tables.c (linked in for that build only). The one exception
 * is the V.32 AGC alpha/beta pair, whose real ELF symbol is a bare,
 * ambiguous, LOCAL (file-static) "AGC_DEF_ALPHA"/"AGC_DEF_BETA" - not
 * externally linkable - so those 4 shorts are a small local literal copy
 * here instead (byte-exact per src/v32/v32_tables.c's own extraction).
 *
 * Scenario selection: unlike test/b103_diff_main.c/test/v23_diff_main.c
 * (which isolate each scenario into its own process because some B103/V23
 * internal state didn't tolerate several live instances in one process),
 * this harness follows test/fpm/fpm_toolkit.main.c's/test/v8_diff_main.c's own
 * precedent of running every test group in ONE process - PPS/SRE/ECC/FSE/
 * scrambler are all directly-called leaf-ish subsystems with no evidence
 * of the same cross-instance state fragility B103 showed.
 */

#include <stdint.h>
#include "core/dsp_table_handles.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/v32/v32.h"

/* U05: the engine / wrapper objects keep their address slots in a tail on x86-64 (src/v32/v32.h): a sub-object is carved out of a full engine allocation at its
 * blob offset (PPS +0x60, circular descriptor +0xb0, ECC +0xe0 ...), a slot is set / read with V32E_PS / V32E_PG (V32W_* for the wrapper); at -m32 these are the
 * plain 4-byte slot accesses of the old harness.  V32H_ALIGN: the wrapper buffer must be 8-aligned on x86-64 (it has a tail). */
#if UINTPTR_MAX == 0xffffffffu
#define V32H_ALIGN
#else
#define V32H_ALIGN __attribute__((aligned(16)))
#endif

/* ---- entry points under test (same signatures for both link targets) ---- */

/* Scrambler / SMC encoders (src/v32/v32_scrambler.c) */
extern void SDMv32_scrambler(short *sdmObj, unsigned short *data, short count);
extern void SDMv32_descrambler(short *sdmObj, unsigned short *data, short count);
extern void SMCv32_encoder_abs(char *smcObj, char *outBuf, unsigned short *rawBits, short count);
extern void SMCv32_encoder_dif(short *smcObj, char *outBuf, unsigned short *rawBits, short count);
extern void SMCv32_encoder_tcm(char *smcObj, char *outBuf, unsigned short *rawBits, short count);

/* PPS (src/v32/v32_pulse_shaping.c) */
extern int FPM_PPS_init(short *ppsObj, const int *cfg, int allocFlag);
extern short FPM_PPS_filter(short *ppsObj, int *ctx, short *out, unsigned short count);
extern void FPM_PPS_free(char *ppsObj);

/* SRE (src/v32/v32_timing.c) */
extern void FPM_SRE_init(void *sreObj, const int *cfg, int allocFlag);
extern unsigned int FPM_SRE_recover(short *sreObj, unsigned short *rawIn, char *outSamples, short count);
extern void FPM_SRE_free(char *sreObj);

/* ECC (src/v32/v32_echo_canceller.c) */
extern void FPM_ECC_init(void *eccObj, const int *cfg, int allocFlag);
extern short FPM_ECC_cancel(char *eccObjBytes, short *samples, unsigned short count);
extern void FPM_ECC_free(char *eccObj);
/* The REAL ECCv32_IMAP/QMAP tables (6-slot sub-table pointer arrays, see
 * src/v32/v32_tables.c's "RESOLVED (2026-09-29 follow-up)" header note) - used
 * by test_ecc_real_tables() below, which (unlike test_ecc()'s synthetic
 * override) exercises FPM_ECC_cancel's every-3rd-sample tap-injection step
 * against the SAME tables/pointers the real dsplibs.o and the real
 * V32FP_recreate/DemodDataV32 call chain actually use. For the BLOB build
 * this resolves to dsplibs.o's own global ECCv32_IMAP/QMAP symbols (whose
 * relocations the final link step resolves to real sub-table addresses);
 * for the PORT build it resolves to src/v32/v32_tables.c's now-fixed pointer
 * arrays. */
extern const short *const ECCv32_IMAP[6];
extern const short *const ECCv32_QMAP[6];

/* FSE (src/v32/v32_equalizer.c) */
extern void FPM_FSE_init(short *fseObj, const int *cfg, int allocFlag);
extern short FPM_FSE_receive(char *fseObj, short *rxSamples, short *outBits, unsigned short count);
extern void FPM_FSE_free(void *fseObj);
extern int FSE_getdiag(char *fseObj, int mode, int *outPairs, int maxCount);
extern unsigned int FSE_decision_AB(char *fseObj, short *angleInOut, short *magInOut);
extern unsigned char FSE_decision_CD(char *fseObj, short *angleInOut, short *magInOut);
extern unsigned int FSE_decision_trn(char *fseObj, short *angleInOut, short *magInOut);
extern unsigned int FSE_decision_4pt(char *fseObj, short *angleInOut, short *magInOut);
extern unsigned int FSE_decision_16pt(char *fseObj, short *angleInOut, short *magInOut);

/* Shared small leaves (src/v32/v32_math.c) */
extern void FPM_atan(short i, short q, unsigned short *angle_out);
extern void FPM_lmsupd(short *taps, char *historyBase, short histIdx, short tapCount, short gain);

/* Mid-level V32 wrapper helpers (src/v32/v32_datapath.c) */
extern void ScrambleDataV32(char *wrapperObj, void *data, unsigned short count);
extern void DescrambleDataV32(char *wrapperObj, void *data, unsigned short count);
extern unsigned short ModDataV32(char *wrapperObj, void *rawBits, void *out, unsigned short count);
extern unsigned short TxNoCarrierV32(char *wrapperObj, void *param_2, void *out, short count);
extern void SetTxModeV32(char *wrapperObj, short mode);
extern void SetRxModeV32(char *wrapperObj, short mode);
extern void SeedScramblerV32(char *wrapperObj, unsigned int seed);
extern unsigned int GetRateV32(char *wrapperObj);
extern void SetAdaptEqV32(char *wrapperObj, unsigned short mode);
extern void SetAdaptEcV32(char *wrapperObj, unsigned short mode);
extern void SetRxLoopsV32(char *wrapperObj, unsigned short mode);
extern void SetECRndTripDelayV32(char *wrapperObj, unsigned short param_2);
extern void TxClockSyncV32(void);
extern unsigned int EpochDetectV32(char *wrapperObj);
extern int RetrainDetectV32(char *wrapperObj);
extern int RenegotiateDetectV32(char *wrapperObj);
extern unsigned short RxClampV32(char *wrapperObj, void *param_2, unsigned short *out);

/* ---- real GLOBAL table symbols (resolved from dsplibs.o for the blob
 * build, from src/v32/v32_tables.c/.vtb.c for the port build - see file
 * header comment) ---- */
extern const short PPSv32_ICOFFS[120];
extern const short PPSv32_QCOFFS[120];
extern const short SMCv32_IMAP16[17];
extern const short SMCv32_QMAP16[17];
extern const short SREv32_COFFS[181];
extern const short SREv32_XB_COFFS[11];
extern const short SREv32_xCLOCK[3];
extern const short SREv32_yCLOCK[3];
extern const short SREv32_PLL_K1[3];
extern const short SREv32_PLL_K2[3];
extern const short FSEv32_ICOFF[103];
extern const short FSEv32_QCOFF[103];
extern const short CRRv32_CLK[5]; /* see src/v32/v32_tables.c's header comment on this array */
extern const short CRRv32_PLL_K1[3];
extern const short CRRv32_PLL_K2[3];
extern const short MRFv32_COFFS[360];

/* a full engine allocation (zeroed, 16-aligned): sub-objects are carved out of it at their blob offset */
/* a harness-owned sub-table pointer array as a cfg word: -m32 its address; x86-64 the handle of the (otherwise unused here) real table `sym`, re-pointed at the array */
#if UINTPTR_MAX == 0xffffffffu
#define ECCH_TABLE(sym, arr) ((int)(intptr_t)(arr))
#else
#define ECCH_TABLE(sym, arr) (cfgp_set(CFGP(sym), (arr)), CFGP(sym))
#endif
static unsigned char *v32h_eng(void)
{
  return (unsigned char *)calloc(1, V32_ENG_ALLOC);
}

/* ---- small helper: deterministic LCG for synthetic test data ---- */
static unsigned int g_lcg = 12345u;
static unsigned int lcg_next(void)
{
  g_lcg = g_lcg * 1103515245u + 12345u;
  return g_lcg;
}
static short lcg_short(int range)
{
  return (short)((lcg_next() >> 16) % (unsigned)range);
}

static void dump_shorts(const char *label, const short *buf, int count)
{
  int i;
  printf("%s:", label);
  for (i = 0; i < count; i++) {
    printf(" %d", (int)buf[i]);
  }
  printf("\n");
}

/* ===================== Scrambler / SMC encoders ===================== */

static void test_scrambler(void)
{
  int mode;
  int modes[] = {2, 3, 4, 5, 6};
  int m;
  printf("=== SDMv32_scrambler/descrambler ===\n");
  for (m = 0; m < (int)(sizeof(modes) / sizeof(modes[0])); m++) {
    short sdmObj[12];
    unsigned short data[64];
    int i;
    mode = modes[m];
    memset(sdmObj, 0, sizeof(sdmObj));
    sdmObj[0] = (short)mode;
    *(unsigned int *)(sdmObj + 4) = (mode == 6) ? 0x3f : 0xffff; /* output mask */
    *(unsigned int *)(sdmObj + 6) = 0x7fffffff;                  /* register mask */
    *(unsigned int *)(sdmObj + 8) = 0x12345678u;                 /* seed register */
    sdmObj[10] = 3;  /* tap1 */
    sdmObj[11] = 17; /* tap2 */
    for (i = 0; i < 64; i++) {
      data[i] = (unsigned short)lcg_short(mode == 6 ? 64 : 65536);
    }
    SDMv32_scrambler(sdmObj, data, 64);
    dump_shorts("scr", (short *)data, 64);
    printf("scr_reg mode=%d: %u\n", mode, *(unsigned int *)(sdmObj + 8));

    /* Descramble the just-scrambled output back through a freshly-seeded
     * descrambler with the SAME initial state - a real self-synchronizing
     * scrambler/descrambler pair should recover the original data. */
    memset(sdmObj, 0, sizeof(sdmObj));
    sdmObj[0] = (short)mode;
    *(unsigned int *)(sdmObj + 4) = (mode == 6) ? 0x3f : 0xffff;
    *(unsigned int *)(sdmObj + 6) = 0x7fffffff;
    *(unsigned int *)(sdmObj + 8) = 0x12345678u;
    sdmObj[10] = 3;
    sdmObj[11] = 17;
    SDMv32_descrambler(sdmObj, data, 64);
    dump_shorts("descr", (short *)data, 64);
    printf("descr_reg mode=%d: %u\n", mode, *(unsigned int *)(sdmObj + 8));
  }
}

static void test_smc_encoders(void)
{
  printf("=== SMCv32_encoder_abs/dif/tcm ===\n");
  {
    /* SMCv32_encoder_abs: smcObj is `char*` - byte0=mode tag, byte6=phase. */
    unsigned char smcObj[8];
    unsigned char *eng = v32h_eng(); /* the descriptor sits at +0xb0 of a full engine allocation */
    unsigned char *outBuf = eng + 0xb0; /* circular descriptor: +8 ptr, +0xc widx, +0x10 size */
    short codes[64];
    unsigned short rawBits[32];
    int i;
    memset(smcObj, 0, sizeof(smcObj));
    smcObj[0] = 0x02;
    memset(outBuf, 0, 32);
    V32E_PS(void *, (char *)(eng), 0xb8, codes);
    *(short *)(outBuf + 0xc) = 0;
    *(short *)(outBuf + 0x10) = 64;
    memset(codes, 0, sizeof(codes));
    for (i = 0; i < 32; i++) {
      rawBits[i] = (unsigned short)lcg_short(65536);
    }
    SMCv32_encoder_abs((char *)smcObj, (char *)(outBuf), rawBits, 32);
    dump_shorts("smc_abs_codes", codes, 32);
    printf("smc_abs_phase: %u widx: %d\n", *(unsigned short *)(smcObj + 6), *(short *)(outBuf + 0xc));
    free(eng);
  }
  {
    /* SMCv32_encoder_dif: smcObj is `short*` - [0]=mode(0/1), [2]=shift,
     * [3]=phase, [4]/[5]=accumulator slots. */
    short smcObj[8];
    unsigned char *eng = v32h_eng();
    unsigned char *outBuf = eng + 0xb0;
    short codes[64];
    unsigned short rawBits[32];
    int i, variant;
    for (variant = 0; variant < 2; variant++) {
      memset(smcObj, 0, sizeof(smcObj));
      smcObj[0] = (short)variant;
      smcObj[2] = 0; /* shift amount */
      memset(outBuf, 0, 32);
      V32E_PS(void *, (char *)(eng), 0xb8, codes);
      *(short *)(outBuf + 0xc) = 0;
      *(short *)(outBuf + 0x10) = 64;
      memset(codes, 0, sizeof(codes));
      for (i = 0; i < 32; i++) {
        rawBits[i] = (unsigned short)lcg_short(65536);
      }
      SMCv32_encoder_dif(smcObj, (char *)(outBuf), rawBits, 32);
      dump_shorts("smc_dif_codes", codes, 32);
      printf("smc_dif_phase(v=%d): %d accum: %d %d\n", variant, smcObj[3], smcObj[4], smcObj[5]);
    }
    free(eng);
  }
  {
    /* SMCv32_encoder_tcm: smcObj is `char*` but with a larger struct -
     * +0=mode, +6=phase, +0xe=trellis state, +0x10=2nd trellis value,
     * +0x14=bit-width constant (low byte). */
    unsigned char smcObj[24];
    unsigned char *eng = v32h_eng();
    unsigned char *outBuf = eng + 0xb0;
    short codes[64];
    unsigned short rawBits[32];
    int i;
    memset(smcObj, 0, sizeof(smcObj));
    smcObj[0] = 0x05;
    *(short *)(smcObj + 0x14) = 2; /* bVar6 = 2 uncoded bits */
    memset(outBuf, 0, 32);
    V32E_PS(void *, (char *)(eng), 0xb8, codes);
    *(short *)(outBuf + 0xc) = 0;
    *(short *)(outBuf + 0x10) = 64;
    memset(codes, 0, sizeof(codes));
    for (i = 0; i < 32; i++) {
      rawBits[i] = (unsigned short)lcg_short(16); /* small values - gets masked to payload width anyway */
    }
    SMCv32_encoder_tcm((char *)smcObj, (char *)(outBuf), rawBits, 32);
    dump_shorts("smc_tcm_codes", codes, 32);
    dump_shorts("smc_tcm_rawbits_after", (short *)rawBits, 32);
    printf("smc_tcm_state: phase=%u trellis=%d sVar2=%d\n",
           *(unsigned short *)(smcObj + 6), *(short *)(smcObj + 0xe), *(short *)(smcObj + 0x10));
    free(eng);
  }
}

/* ===================== PPS ===================== */

static void test_pps(void)
{
  unsigned char *eng = v32h_eng(); /* PPS at +0x60 and the circular descriptor at +0xb0 of one engine allocation */
  short *ppsObj = (short *)(eng + 0x60);
  int *ctx = (int *)(eng + 0xb0);
  /* NOTE: FPM_PPS_filter reads this byte array with a STRIDE-2 index
   * (`*(unsigned char *)(codeArray + idx*2)`, confirmed in the decompile -
   * see src/v32/v32_pulse_shaping.c) - the buffer must be sized wrap_bound*2, not
   * wrap_bound, to stay in bounds. */
  unsigned char codeBuf[512];
  /* FPM_PPS_filter's `count` is INPUT-symbol count to consume, not output
   * sample count - it's a 10/3 (stride/increment) UPSAMPLING interpolator,
   * so the actual number of output samples produced can be well over
   * `count` (confirmed empirically against dsplibs.o - the function's own
   * return value is the true output count, NOT necessarily == count, contra
   * this chunk's own decompile-archaeology write-up, which turns out to be
   * mistaken on this one point). Sized generously (10x margin) below. */
  short out[2000];
  int cfg[10];
  int i;

  printf("=== FPM_PPS_init/filter/free ===\n");
  memset(ppsObj, 0, 0x40);
  cfg[0] = 0x3000a;
  cfg[1] = 0x1;
  cfg[2] = 0x20000;
  cfg[3] = 0x0;
  cfg[4] = CFGP(SMCv32_IMAP16);
  cfg[5] = CFGP(SMCv32_QMAP16);
  cfg[6] = CFGP(PPSv32_ICOFFS);
  cfg[7] = CFGP(PPSv32_QCOFFS);
  cfg[8] = 0x78;
  cfg[9] = 0x0;
  FPM_PPS_init(ppsObj, cfg, 1);

  memset(ctx, 0, 5 * sizeof(int));
  memset(codeBuf, 0, sizeof(codeBuf));
  for (i = 0; i < 200; i++) {
    /* stride-2 layout - see codeBuf's declaration comment above */
    codeBuf[i * 2] = (unsigned char)lcg_short(16); /* valid SMCv32_IMAP16/QMAP16 index range */
  }
  V32E_PS(void *, (char *)(eng), 0xb8, codeBuf);
  *(short *)((char *)ctx + 0xc) = 0; /* write index (producer) */
  *(short *)((char *)ctx + 0xe) = 0; /* read index (PPS's own) */
  ctx[4] = 200; /* wrap bound */

  memset(out, 0, sizeof(out));
  {
    short ret = FPM_PPS_filter(ppsObj, ctx, out, 150);
    printf("pps_ret: %d\n", ret);
    dump_shorts("pps_out", out, ret > 0 && ret < 2000 ? ret : 0);
  }
  printf("pps_state: phase=%d needNew=%d widx=%d readIdx=%d\n",
         ppsObj[0x15], ppsObj[0x14], ppsObj[0x16], *(short *)((char *)ctx + 0xe));
  FPM_PPS_free((char *)(ppsObj));
  free(eng);
}

/* ===================== SRE ===================== */

static void test_sre(void)
{
  unsigned char *sreEng = v32h_eng(); /* the SRE object sits at +0x148 of a full engine allocation */
  unsigned char *sreObjBuf = sreEng + 0x148;
  int cfg[14];
  short *sreObj = (short *)sreObjBuf;
  unsigned short rawIn[4000];
  short outSamples[4000];
  int i;

  printf("=== FPM_SRE_init/recover/free ===\n");
  memset(sreObjBuf, 0, 0x100);
  cfg[0] = 0x30003;
  cfg[1] = 0x460010;
  cfg[2] = 0x40002000;
  cfg[3] = 0xb4;
  cfg[4] = CFGP(SREv32_COFFS);
  cfg[5] = CFGP(SREv32_XB_COFFS);
  cfg[6] = CFGP(SREv32_xCLOCK);
  cfg[7] = CFGP(SREv32_yCLOCK);
  cfg[8] = CFGP(SREv32_PLL_K1);
  cfg[9] = CFGP(SREv32_PLL_K2);
  cfg[10] = 0x10002;
  cfg[11] = 0xc82666;
  cfg[12] = 0x905dc;
  cfg[13] = 0x0;
  FPM_SRE_init(sreObj, cfg, 1);

  for (i = 0; i < 4000; i++) {
    rawIn[i] = (unsigned short)(lcg_short(2000) - 1000); /* small-amplitude synthetic signal */
  }
  memset(outSamples, 0, sizeof(outSamples));
  {
    unsigned int produced = FPM_SRE_recover(sreObj, rawIn, (char *)(outSamples), 4000);
    printf("sre_produced: %u\n", produced);
    dump_shorts("sre_out", outSamples, (int)(produced < 4000 ? produced : 4000));
  }
  printf("sre_state: acquireState=%d timing_ppm=%d\n", sreObj[0x1c], sreObj[0x41]);
  FPM_SRE_free((char *)(sreObj));
  free(sreEng);
}

/* ===================== ECC ===================== */

static void test_ecc(void)
{
  unsigned char *eccEng = v32h_eng(); /* the ECC object sits at +0xe0 of a full engine allocation (its buffer slots live in the engine tail) */
  unsigned char *eccObjBuf = eccEng + 0xe0;
  int cfg[6];
  short eccImapSub0[32];
  short eccQmapSub0[32];
  void *eccImapPtrs[6];
  void *eccQmapPtrs[6];
  short samples[3000];
  int i;

  printf("=== FPM_ECC_init/cancel/free ===\n");
  memset(eccObjBuf, 0, 0x100);
  /* Pre-seed fields FPM_ECC_init reads as INPUT before writing anything -
   * see src/v32/v32_echo_canceller.c's header comment (V32FP_recreate pre-seeds
   * engine+0x144/+0x146 == eccObj+0x64/+0x66 immediately before calling
   * FPM_ECC_init in the real object). */
  *(short *)(eccObjBuf + 0x64) = 0x30;
  *(short *)(eccObjBuf + 0x66) = 0x30;

  /* Synthetic, non-zero IMAP/QMAP sub-table-pointer arrays - see header
   * comment: the real ECCv32_IMAP/QMAP are byte-exact all-zero, which would
   * make the "every 3rd sample" training-tap injection dereference a NULL
   * sub-table pointer. Both blob and port runs use this SAME synthetic data
   * (harness-owned), so the comparison is meaningful either way. */
  for (i = 0; i < 32; i++) {
    eccImapSub0[i] = (short)(1000 + i * 7);
    eccQmapSub0[i] = (short)(2000 - i * 5);
  }
  memset(eccImapPtrs, 0, sizeof(eccImapPtrs));
  memset(eccQmapPtrs, 0, sizeof(eccQmapPtrs));
  eccImapPtrs[0] = eccImapSub0;
  eccQmapPtrs[0] = eccQmapSub0;

  cfg[0] = 0x2801e0;
  cfg[1] = 0x28;
  cfg[2] = ECCH_TABLE(ECCv32_IMAP, eccImapPtrs);
  cfg[3] = ECCH_TABLE(ECCv32_QMAP, eccQmapPtrs);
  cfg[4] = 0x10;
  cfg[5] = 0x0;
  FPM_ECC_init(eccObjBuf, cfg, 1);

  /* Both real call sites (V32FP_recreate, SetAdaptEcV32) unconditionally
   * overwrite these 3 fields right after calling FPM_ECC_init - see header
   * comment. Mirrored here so FPM_ECC_cancel's "every 3rd sample" indices
   * start from the same state real callers actually use. */
  *(short *)(eccObjBuf + 0x34) = 0x30;
  *(short *)(eccObjBuf + 0x30) = 0;
  *(short *)(eccObjBuf + 0x32) = 0;
  /* Enable adaptation so the LMS tap-update path is actually exercised. */
  *(int *)(eccObjBuf + 0x18) = 0;  /* energy-tracking enabled (0 == enabled, per decompile) */
  *(int *)(eccObjBuf + 0x24) = 0;  /* adaptation-master enabled */
  *(int *)(eccObjBuf + 0x28) = 1;  /* near-section enabled */
  *(int *)(eccObjBuf + 0x2c) = 1;  /* far-section enabled */

  for (i = 0; i < 3000; i++) {
    samples[i] = (short)(lcg_short(4000) - 2000);
  }
  {
    short ticks = FPM_ECC_cancel((char *)(eccObjBuf), samples, 3000);
    printf("ecc_ticks: %d\n", ticks);
  }
  dump_shorts("ecc_out", samples, 3000);
  printf("ecc_state: e20=%d e22=%d e48=%u e60=%d e62=%d e30=%d e3c=%d e32=%d\n",
         *(short *)(eccObjBuf + 0x20), *(short *)(eccObjBuf + 0x22),
         *(unsigned short *)(eccObjBuf + 0x48), *(short *)(eccObjBuf + 0x60),
         *(short *)(eccObjBuf + 0x62), *(short *)(eccObjBuf + 0x30),
         *(short *)(eccObjBuf + 0x3c), *(short *)(eccObjBuf + 0x32));
  FPM_ECC_free((char *)(eccObjBuf));
  free(eccEng);
}

/* ===================== ECC (real IMAP/QMAP tables - the targeted crash repro) =====================
 * Root-caused (2026-09-29 follow-up) the crash found empirically by the
 * LocLoop hand-clean chunk's differential test (see NOTES.md and
 * test/v32/v32_engine.loclop.main.c's "ECC LANDMINE" comment): the port previously
 * captured ECCv32_IMAP/ECCv32_QMAP as byte-exact ALL-ZERO flat data, but
 * `readelf -r --wide dsplibs.o` shows both tables' entire on-disk byte range
 * is actually a 6-slot array of R_386_32-relocated SUB-TABLE POINTERS
 * (confirmed independently via gdb reading `test/.build/v32/v32_engine.loclop_test_blob`'s
 * live memory) - see src/v32/v32_tables.c's header comment for the full
 * writeup. This test exercises exactly the code path that crashed: unlike
 * test_ecc() above (which deliberately uses a harness-owned SYNTHETIC
 * sub-table-pointer array, identical on both sides, chosen before this root
 * cause was known), this one uses the REAL ECCv32_IMAP/ECCv32_QMAP tables
 * and the REAL ECCv32_CFG_raw-equivalent config (cfg[4]=0x10, i.e. the
 * constant training-buffer fill value FPM_ECC_init actually uses in every
 * real call site), with count=9 (>=3, forcing the every-3rd-sample
 * injection branch to fire 3 times within a single FPM_ECC_cancel call -
 * strictly more than the 1 tick that was enough to crash the port before
 * this fix). Before the fix, the PORT build of this exact scenario
 * segfaults (near-NULL dereference through ECCv32_QMAP[0]==0); the BLOB
 * build has always completed cleanly (its ECCv32_QMAP[0] is a real,
 * non-null, linker-resolved pointer). After the fix, both sides must
 * produce byte-identical output. */
static void test_ecc_real_tables(void)
{
  unsigned char *eccEng = v32h_eng();
  unsigned char *eccObjBuf = eccEng + 0xe0;
  int cfg[6];
  short samples[9];
  int i;

  printf("=== FPM_ECC_init/cancel/free (REAL ECCv32_IMAP/QMAP tables) ===\n");
  memset(eccObjBuf, 0, 0x100);
  /* Same pre-seed as test_ecc() - see its comment. */
  *(short *)(eccObjBuf + 0x64) = 0x30;
  *(short *)(eccObjBuf + 0x66) = 0x30;

  /* Real ECCv32_CFG_raw layout (src/v32/v32_tables.c), using the REAL tables
   * instead of a synthetic override. */
  cfg[0] = 0x2801e0;
  cfg[1] = 0x28;
  cfg[2] = CFGP(ECCv32_IMAP);
  cfg[3] = CFGP(ECCv32_QMAP);
  cfg[4] = 0x10;
  cfg[5] = 0x0;
  FPM_ECC_init(eccObjBuf, cfg, 1);

  /* Same post-init overwrite both real call sites perform - see test_ecc(). */
  *(short *)(eccObjBuf + 0x34) = 0x30;
  *(short *)(eccObjBuf + 0x30) = 0;
  *(short *)(eccObjBuf + 0x32) = 0;
  *(int *)(eccObjBuf + 0x18) = 0;
  *(int *)(eccObjBuf + 0x24) = 0;
  *(int *)(eccObjBuf + 0x28) = 1;
  *(int *)(eccObjBuf + 0x2c) = 1;

  for (i = 0; i < 9; i++) {
    samples[i] = (short)(lcg_short(4000) - 2000);
  }
  {
    short ticks = FPM_ECC_cancel((char *)(eccObjBuf), samples, 9);
    printf("ecc_real_ticks: %d\n", ticks);
  }
  dump_shorts("ecc_real_out", samples, 9);
  printf("ecc_real_state: e20=%d e22=%d e48=%u e60=%d e62=%d e30=%d e3c=%d e32=%d\n",
         *(short *)(eccObjBuf + 0x20), *(short *)(eccObjBuf + 0x22),
         *(unsigned short *)(eccObjBuf + 0x48), *(short *)(eccObjBuf + 0x60),
         *(short *)(eccObjBuf + 0x62), *(short *)(eccObjBuf + 0x30),
         *(short *)(eccObjBuf + 0x3c), *(short *)(eccObjBuf + 0x32));
  FPM_ECC_free((char *)(eccObjBuf));
  free(eccEng);
}

/* ===================== FSE ===================== */

static short *g_fseCfgBuild(int cfg[14])
{
  cfg[0] = 0x30090;
  cfg[1] = CFGP(FSEv32_ICOFF);
  cfg[2] = CFGP(FSEv32_QCOFF);
  cfg[3] = 0xcab0067;
  cfg[4] = 0x361;
  cfg[5] = CFGP(CRRv32_CLK);
  cfg[6] = 0x10004;
  cfg[7] = 0x198805dc;
  cfg[8] = 0x666;
  cfg[9] = CFGP(CRRv32_PLL_K1);
  cfg[10] = CFGP(CRRv32_PLL_K2);
  cfg[11] = 0x0;
  cfg[12] = 0x0;
  cfg[13] = 0x0;
  return 0;
}

static void test_fse_receive(int installMode, int sampleCount, const char *label)
{
  unsigned char *fseEng = v32h_eng(); /* FSE at +0x204 (0x4e18 bytes), its history / VTB object at +0x5020 of one full engine allocation */
  short *fseObj = (short *)(fseEng + 0x204);
  unsigned char *fseHist = fseEng + 0x5020;
  int cfg[14];
  short *rxSamples = (short *)malloc(sizeof(short) * (size_t)sampleCount);
  short *outBits = (short *)calloc((size_t)sampleCount, sizeof(short));
  int i;

  printf("=== FPM_FSE_receive chain (%s) ===\n", label);
  g_fseCfgBuild(cfg);
  FPM_FSE_init(fseObj, cfg, 1);
  V32E_PS(void *, (char *)(fseEng), 0x230, fseHist); /* FSE +0x2c: the history object */
  /* Pre-seed the persistent symbol-timing fields (fseObj+0x74/+0x8a) to a
   * sane non-degenerate state - FPM_FSE_init itself does NOT initialize
   * these two (confirmed: they're outside its own zero-init list), and an
   * all-zero cold start (this test's first draft) makes the phase-modulus
   * reduction loop inside FPM_FSE_receive spiral away from zero forever
   * (short overflow) - CONFIRMED via gdb to crash the REAL dsplibs.o
   * identically, not a port bug, so real V32FP_recreate/SetRxModeV32 must
   * seed these two fields to something like this before ever calling
   * FPM_FSE_receive for the first time (that seeding logic itself is out of
   * this chunk's scope - v32_state.c/next chunk's job). Values chosen here
   * are synthetic but self-consistent (same values feed both blob and port
   * runs), per this project's established testing precedent. */
  *(short *)((char *)fseObj + 0x74) = 3;
  *(short *)((char *)fseObj + 0x8a) = 2;
  if (installMode == 0) {
    V32E_PS(void *, (char *)(fseEng), 0x234, (void *)FSE_decision_AB); /* FSE +0x30: the decision callback */
  } else {
    V32E_PS(void *, (char *)(fseEng), 0x234, (void *)FSE_decision_4pt); /* FSE +0x30: the decision callback */
  }
  for (i = 0; i < sampleCount; i++) {
    /* Deliberately low-amplitude synthetic RX noise - matches this
     * project's established B103/V23 precedent (full-range noise tends to
     * drive internal adaptive-filter/PLL state into corners neither
     * implementation handles robustly, not useful for a behavioral-
     * equivalence test). NOTE: driving this at ANY realistic amplitude
     * (including full-range, tried during this chunk's development) trips
     * FPM_phasor's negative-octant-selector table read - a genuine,
     * previously-undiscovered bug in the already-verified src/fpm/fpm_math.c/
     * fpm_tables.c FPM_sin_sign/FPM_cos_sign tables, found and fixed as
     * part of this chunk (see NOTES.md and those files' own header
     * comments) - not something amplitude tuning alone could have avoided. */
    rxSamples[i] = (short)(lcg_short(400) - 200);
  }
  {
    /* FPM_FSE_receive must be driven in small per-buffer CHUNKS, not one
     * giant call - confirmed via gdb watchpoint tracing (not guessed): its
     * local_28/local_2c output-tap pointers are read ONCE from fseObj+0x54/
     * +0x58 at function entry and walk forward (`local_28 = local_28 + 1`)
     * once per SYMBOL decision with NO wraparound anywhere in the function -
     * a real call must therefore never let one call's own symbol-decision
     * count exceed that buffer's size (~50 shorts, sized
     * `(fseObj[0]/fseObj[1])*2+4` by FPM_FSE_init), matching how a real
     * engine would call this once per small incoming audio buffer, not once
     * for an entire session's worth of samples. This project's first draft
     * fed the whole 12000-sample scenario as ONE call, which walks local_28/
     * local_2c far past their buffer and corrupts adjacent heap memory
     * (confirmed via gdb to crash the REAL dsplibs.o identically - not a
     * port bug, a test-harness misuse of the function's real calling
     * contract). Fixed by chunking into CHUNK-sized calls below. */
#define FSE_CHUNK 24
    int done = 0;
    int totalProduced = 0;
    while (done < sampleCount) {
      int thisChunk = sampleCount - done < FSE_CHUNK ? sampleCount - done : FSE_CHUNK;
      short produced = FPM_FSE_receive((char *)(fseObj), rxSamples + done, outBits + totalProduced,
                                        (unsigned short)thisChunk);
      totalProduced += produced;
      done += thisChunk;
    }
    printf("fse_produced(%s): %d\n", label, totalProduced);
    dump_shorts("fse_outbits", outBits, totalProduced < sampleCount ? totalProduced : sampleCount);
#undef FSE_CHUNK
  }
  printf("fse_state(%s): 5e=%d 50=%d 52=%d 70=%d hist37=%d hist62=%d hist6e=%d\n", label,
         *((short *)fseObj + 0x2f) /* fseObj+0x5e */,
         *(short *)((char *)fseObj + 0x50), *(short *)((char *)fseObj + 0x52),
         *(int *)((char *)fseObj + 0x70),
         *(short *)(fseHist + 0x6e), *(short *)(fseHist + 0x62), *(short *)(fseHist + 0x6c));

  {
    int diag[2 * 200];
    int n = FSE_getdiag((char *)(fseObj), 0, diag, 200);
    printf("fse_getdiag_count(%s): %d\n", label, n);
    if (n > 0) {
      int k;
      printf("fse_getdiag_vals(%s):", label);
      for (k = 0; k < n && k < 5; k++) {
        printf(" (%d,%d)", diag[k * 2], diag[k * 2 + 1]);
      }
      printf("\n");
    }
  }

  FPM_FSE_free(fseObj);
  free(fseEng);
  free(rxSamples);
  free(outBits);
}

static void test_fse_decisions_direct(void)
{
  unsigned char *fseEng = v32h_eng(); /* FSE at +0x204 (0x4e18 bytes), its history / VTB object at +0x5020 of one full engine allocation */
  short *fseObj = (short *)(fseEng + 0x204);
  unsigned char *fseHist = fseEng + 0x5020;
  int cfg[14];
  int i;
  short angle, mag;

  printf("=== FSE_decision_AB/CD/trn/4pt/16pt (direct) ===\n");
  g_fseCfgBuild(cfg);
  FPM_FSE_init(fseObj, cfg, 1);
  V32E_PS(void *, (char *)(fseEng), 0x230, fseHist); /* FSE +0x2c: the history object */
  *((short *)fseObj + 0x2f) = 0; /* fseObj+0x5e = 0, a valid tap-buffer index */

  for (i = 0; i < 400; i++) {
    unsigned int ret;
    angle = (short)(lcg_short(0x8000));
    mag = (short)(lcg_short(0x8000));
    ret = FSE_decision_AB((char *)(fseObj), &angle, &mag);
    if (i < 5 || i == 399) {
      printf("AB[%d]: ret=%u angle=%d hist37=%d\n", i, ret, angle, *(short *)(fseHist + 0x6e));
    }
  }
  dump_shorts("AB_hist_tail", (short *)(fseHist + 0x50), 16);

  memset(fseHist, 0, 0xbc); /* the history object ends at the engine end (+0x50dc): the tail behind it holds the address slots */
  for (i = 0; i < 20; i++) {
    unsigned int ret;
    angle = 0;
    mag = 0;
    ret = FSE_decision_CD((char *)(fseObj), &angle, &mag);
    printf("CD[%d]: ret=%u angle=%d hist38=%d\n", i, ret, angle, *(short *)((char *)fseObj + 0x38));
  }

  memset(fseHist, 0, 0xbc); /* the history object ends at the engine end (+0x50dc): the tail behind it holds the address slots */
  for (i = 0; i < 1400; i++) {
    unsigned int ret;
    angle = 0;
    mag = 0;
    ret = FSE_decision_trn((char *)(fseObj), &angle, &mag);
    if (i < 5 || i == 300 || i == 600 || i == 1399) {
      printf("trn[%d]: ret=%u angle=%d hist70=%d\n", i, ret, angle, *(int *)(fseHist + 0x70));
    }
  }

  memset(fseHist, 0, 0xbc); /* the history object ends at the engine end (+0x50dc): the tail behind it holds the address slots */
  for (i = 0; i < 200; i++) {
    unsigned int ret;
    angle = (short)(lcg_short(0x8000));
    mag = (short)(lcg_short(0x8000));
    ret = FSE_decision_4pt((char *)(fseObj), &angle, &mag);
    if (i < 5 || i == 199) {
      printf("4pt[%d]: ret=%u angle=%d hist2f=%d\n", i, ret, angle, *(short *)(fseHist + 0x5e));
    }
  }

  /* FSE_decision_16pt is deliberately NOT differentially tested by direct,
   * repeated calls here - see NOTES.md for the full writeup. Short version:
   * this function's own KNOWN VENDOR BUG (a `>>1` shift where its sibling
   * FSE_decision_16Tpt correctly uses `>>13` - see src/v32/v32_equalizer.c's header
   * comment) makes its dead magnitude-output computation read up to ~24KB
   * past an unidentified base address on every call, regardless of input.
   * Confirmed via repeated gdb runs (crashing 5/5 with ASLR active, 0/5
   * under gdb's own default ASLR-disabling or `setarch -R`) that this
   * reliably SEGFAULTS the REAL dsplibs.o when called directly in this
   * minimal test process (whose memory map has far less mapped/adjacent
   * memory around that static address than a full slmodemd process would) -
   * a genuine, real crash risk from the vendor's own bug interacting with
   * this harness's sparse address space, not a port bug or a test-harness
   * sizing mistake (every other function in this file was independently
   * ruled out first - see NOTES.md's blow-by-blow debugging history). The
   * port's own FSE_decision_16pt (src/v32/v32_equalizer.c) still faithfully
   * replicates the exact vendor shift-amount bug per this project's bit-
   * exact-behavior-matching rule; it is simply not safe to exercise
   * directly against the real blob in isolation the way AB/CD/trn/4pt are
   * above. */
  (void)mag;

  FPM_FSE_free(fseObj);
  free(fseEng);
}

/* ===================== Mid-level V32 helpers ===================== */

static void test_midlevel_helpers(void)
{
  unsigned char *engine = (unsigned char *)calloc(1, 0x6000);
  /* NOTE: fields are accessed up to byte offset 0x68/100(0x64) (each a
   * 4-byte pointer, so actually touching up to byte 0x6c/0x68) - a much earlier
   * draft sized this at 0x40 (64) bytes, a real stack-buffer-overflow bug
   * in this TEST HARNESS (not the port) that silently corrupted adjacent
   * stack memory and caused an ASLR-dependent intermittent segfault
   * (confirmed via careful offset auditing after the crash proved
   * non-reproducible under gdb/setarch -R, i.e. layout-dependent). Sized
   * generously here. */
  unsigned char wrapper[0x200] V32H_ALIGN;
  unsigned char clampSub[0x100];
  unsigned char rateSeqSub[0x100]; /* engine+0x230 sub-object */
  /* engine+0x118's target buffer - SetECRndTripDelayV32 fills up to
   * `clampSub[0x9c] + min(param_2,engine[0xe4],engine[0xe0]) +
   * V32_SYMBOL_LEN[wrapper[0x18]]*2` entries (computed, not a fixed size) -
   * sized generously here to stay safely within bounds for this test's
   * chosen inputs (computed worst case well under 256 shorts). */
  unsigned char ecRtBuf[1024];
  int cfg[6];
  int i;

  printf("=== Mid-level V32 helpers ===\n");
  memset(wrapper, 0, sizeof(wrapper));
  memset(clampSub, 0, sizeof(clampSub));
  memset(rateSeqSub, 0, sizeof(rateSeqSub));
  memset(ecRtBuf, 0, sizeof(ecRtBuf));
  V32W_PS(void *, (char *)(wrapper), 0x68, engine);
  V32W_PS(void *, (char *)(wrapper), 100, clampSub);
  V32E_PS(void *, (char *)(engine), 0x230, rateSeqSub);
  V32E_PS(void *, (char *)(engine), 0x118, ecRtBuf);

  /* ---- GetRateV32 (pure arithmetic on wrapper fields) ---- */
  {
    short rates[] = {0x3840, 12000, 0x2580, 0x1c20, 0x12c0, 999};
    int trellisFlags[] = {0, 0};
    int r;
    for (r = 0; r < (int)(sizeof(rates) / sizeof(rates[0])); r++) {
      *(short *)(wrapper + 4) = rates[r];
      *(int *)(wrapper + 0x1c) = 0;
      printf("GetRateV32(rate=%d,trellis=0): %u\n", rates[r], GetRateV32((char *)(wrapper)));
      *(int *)(wrapper + 0x1c) = 1;
      printf("GetRateV32(rate=%d,trellis=1): %u\n", rates[r], GetRateV32((char *)(wrapper)));
    }
    (void)trellisFlags;
  }

  /* ---- SeedScramblerV32 ---- */
  SeedScramblerV32((char *)(wrapper), 0xdeadbeefu);
  printf("SeedScramblerV32 -> engine+0x40: %u\n", *(unsigned int *)(engine + 0x40));

  /* ---- SetAdaptEqV32 ---- */
  {
    int mode;
    for (mode = 0; mode <= 3; mode++) {
      SetAdaptEqV32((char *)(wrapper), (unsigned short)mode);
      printf("SetAdaptEqV32(%d) -> e10=%u e23c=%u\n", mode,
             *(unsigned int *)(engine + 0x10), *(unsigned short *)(engine + 0x23c));
    }
  }

  /* ---- SetRxLoopsV32 ---- */
  {
    int mode;
    for (mode = 0; mode <= 4; mode++) {
      SetRxLoopsV32((char *)(wrapper), (unsigned short)mode);
      printf("SetRxLoopsV32(%d) -> %u %u %u %u\n", mode,
             *(unsigned int *)(engine + 0), *(unsigned int *)(engine + 4),
             *(unsigned int *)(engine + 8), *(unsigned int *)(engine + 0xc));
    }
  }

  /* ---- SetECRndTripDelayV32 ---- */
  *(short *)(clampSub + 0x9c) = 20;
  *(short *)(wrapper + 0x18) = 0;
  *(unsigned short *)(engine + 0xe4) = 30;
  *(unsigned short *)(engine + 0xe0) = 100;
  SetECRndTripDelayV32((char *)(wrapper), 10);
  printf("SetECRndTripDelayV32 -> e110=%d e112=%d e114=%d ebc=%d ebe=%d ec0=%d ecrt0=%d ecrt1=%d\n",
         *(short *)(engine + 0x110), *(short *)(engine + 0x112), *(short *)(engine + 0x114),
         *(short *)(engine + 0xbc), *(short *)(engine + 0xbe), *(short *)(engine + 0xc0),
         *(short *)(ecRtBuf + 0), *(short *)(ecRtBuf + 2));

  /* ---- TxClockSyncV32 (confirmed no-op) ---- */
  TxClockSyncV32();
  printf("TxClockSyncV32: called (no-op)\n");

  /* ---- EpochDetectV32 / RetrainDetectV32 / RenegotiateDetectV32 ---- */
  *(unsigned int *)(rateSeqSub + 100) = 0x77;
  printf("EpochDetectV32: %u\n", EpochDetectV32((char *)(wrapper)));
  *(unsigned short *)(rateSeqSub + 0x62) = 3; /* both bit0 and bit1 set */
  *(short *)(rateSeqSub + 0x74) = 5;
  printf("RetrainDetectV32(1st): %d flags=%u retries=%d\n",
         RetrainDetectV32((char *)(wrapper)), *(unsigned short *)(rateSeqSub + 0x62),
         *(short *)(rateSeqSub + 0x74));
  printf("RetrainDetectV32(2nd): %d flags=%u\n",
         RetrainDetectV32((char *)(wrapper)), *(unsigned short *)(rateSeqSub + 0x62));
  *(unsigned short *)(rateSeqSub + 0x62) = 2;
  printf("RenegotiateDetectV32(1st): %d flags=%u\n",
         RenegotiateDetectV32((char *)(wrapper)), *(unsigned short *)(rateSeqSub + 0x62));
  printf("RenegotiateDetectV32(2nd): %d flags=%u\n",
         RenegotiateDetectV32((char *)(wrapper)), *(unsigned short *)(rateSeqSub + 0x62));

  /* ---- RxClampV32 ---- */
  {
    unsigned short clampOut[8];
    memset(clampOut, 0xAA, sizeof(clampOut));
    *(short *)(clampSub + 0x9e) = 5;
    {
      unsigned short ret = RxClampV32((char *)(wrapper), 0, clampOut);
      printf("RxClampV32: ret=%u vals=%u %u %u %u %u %u\n", ret, clampOut[0], clampOut[1],
             clampOut[2], clampOut[3], clampOut[4], clampOut[5]);
    }
  }

  /* ---- SetTxModeV32 (all 8 branches - none touch VTBv32_init) ---- */
  {
    int m;
    for (m = 0; m <= 7; m++) {
      memset(engine, 0, 0x400); /* reset the low region SetTxModeV32 touches */
      *(short *)(engine + 0x32) = 200; /* symbol-length-ish fields SetTxModeV32 reads */
      *(short *)(engine + 0x34) = 200;
      SetTxModeV32((char *)(wrapper), (short)m);
      printf("SetTxModeV32(%d) -> e30=%u e48=%u e4a=%u e4c=%u e2c=%u ea4=%u e38=%u e3c=%u e44=%d e46=%d status30=%u status31=%u\n",
             m, *(unsigned short *)(engine + 0x30), *(unsigned short *)(engine + 0x48),
             *(unsigned short *)(engine + 0x4a), *(unsigned short *)(engine + 0x4c),
             *(unsigned short *)(engine + 0x2c), *(unsigned short *)(engine + 0xa4),
             *(unsigned int *)(engine + 0x38), *(unsigned int *)(engine + 0x3c),
             *(short *)(engine + 0x44), *(short *)(engine + 0x46),
             *(unsigned char *)(wrapper + 0x30), *(unsigned char *)(wrapper + 0x31));
    }
  }

  /* ---- SetRxModeV32 (modes 0/1/default only - modes 2-6 call the
   * out-of-scope VTBv32_init, see file header comment) ---- */
  {
    int modes[] = {0, 1, 99};
    int mi;
    for (mi = 0; mi < 3; mi++) {
      memset(engine + 0x2000, 0, 0x1000); /* reset the RX-mode region (engine+0x50xx et al) */
      memset(rateSeqSub, 0, sizeof(rateSeqSub));
      V32E_PS(void *, (char *)(engine), 0x230, rateSeqSub);
      SetRxModeV32((char *)(wrapper), (short)modes[mi]);
      printf("SetRxModeV32(%d) -> e50b0=%u e5098=%u e509c=%u e2e=%u e50b8=%u e50bc=%u e50c4=%d e50c6=%d status30=%u status31=%u hist0=%d hist4=%d hist6e=%d\n",
             modes[mi],
             *(unsigned short *)(engine + 0x50b0), *(unsigned short *)(engine + 0x5098),
             *(unsigned short *)(engine + 0x509c), *(unsigned short *)(engine + 0x2e),
             *(unsigned int *)(engine + 0x50b8), *(unsigned int *)(engine + 0x50bc),
             *(short *)(engine + 0x50c4), *(short *)(engine + 0x50c6),
             *(unsigned char *)(wrapper + 0x30), *(unsigned char *)(wrapper + 0x31),
             *(short *)(rateSeqSub + 0), *(short *)(rateSeqSub + 4), *(short *)(rateSeqSub + 0x6e));
    }
  }

  /* ---- ScrambleDataV32 / DescrambleDataV32 ---- */
  memset(engine + 0x30, 0, 0x20);
  *(short *)(engine + 0x30) = 4; /* mode */
  *(unsigned int *)(engine + 0x38) = 0xffff;       /* output mask */
  *(unsigned int *)(engine + 0x3c) = 0x7fffffff;   /* register mask */
  *(unsigned int *)(engine + 0x40) = 0x1357;       /* seed register */
  *(short *)(engine + 0x44) = 3;  /* tap1 */
  *(short *)(engine + 0x46) = 11; /* tap2 */
  {
    unsigned short scrData[32];
    for (i = 0; i < 32; i++) {
      scrData[i] = (unsigned short)lcg_short(65536);
    }
    ScrambleDataV32((char *)(wrapper), scrData, 32);
    dump_shorts("ScrambleDataV32_out", (short *)scrData, 32);
    printf("ScrambleDataV32_reg: %u\n", *(unsigned int *)(engine + 0x40));
  }

  memset(engine + 0x50b0, 0, 0x20);
  *(short *)(engine + 0x50b0) = 4;
  *(unsigned int *)(engine + 0x50b8) = 0xffff;
  *(unsigned int *)(engine + 0x50bc) = 0x7fffffff;
  *(unsigned int *)(engine + 0x50c0) = 0x2468;
  *(short *)(engine + 0x50c4) = 3;
  *(short *)(engine + 0x50c6) = 11;
  {
    unsigned short descrData[32];
    for (i = 0; i < 32; i++) {
      descrData[i] = (unsigned short)lcg_short(65536);
    }
    DescrambleDataV32((char *)(wrapper), descrData, 32);
    dump_shorts("DescrambleDataV32_out", (short *)descrData, 32);
    printf("DescrambleDataV32_reg: %u\n", *(unsigned int *)(engine + 0x50c0));
  }

  /* ---- SetAdaptEcV32 (needs a properly-initialized ECC object at
   * engine+0xe0 first - mirrors SetAdaptEcV32(mode 0)'s own "reset from
   * current config" pattern, see src/v32/v32_datapath.c's header comment). ---- */
  memset(engine + 0xe0, 0, 0x100);
  *(short *)(wrapper + 0x14) = 0x30;
  *(short *)(engine + 0xe0 + 0x64) = 0x30; /* pre-seed - see src/v32/v32_echo_canceller.c header comment */
  *(short *)(engine + 0xe0 + 0x66) = 0x30;
  cfg[0] = 0x2801e0;
  cfg[1] = 0x28;
  cfg[2] = 0; /* IMAP/QMAP left NULL here - SetAdaptEcV32's mode-0 path never
               * reaches the "every 3rd sample" injection branch within this
               * short test (only a handful of adaptation-mode-switch calls
               * are made, no FPM_ECC_cancel calls follow in this scenario) */
  cfg[3] = 0;
  cfg[4] = 0x10;
  cfg[5] = 0;
  FPM_ECC_init(engine + 0xe0, cfg, 1);
  *(short *)(engine + 0xe0 + 0x34) = 0x30;
  *(short *)(engine + 0xe0 + 0x30) = 0;
  *(short *)(engine + 0xe0 + 0x32) = 0;
  {
    int mode;
    for (mode = 0; mode <= 3; mode++) {
      if (mode == 2) {
        /* mode 2 zero-fills an array sized by e114 - give it a small,
         * consistent non-zero size and a matching buffer first. */
        *(short *)(engine + 0xe0 + 0x114) = 4;
        /* (a plain int store into the image at +0x1f8, as ever: the low 32 bits of the buffer address; nothing reads it) */
        *(int *)(engine + 0xe0 + 0x118) = (int)(intptr_t)((uintptr_t)ecRtBuf & 0xffffffffu);
      }
      SetAdaptEcV32((char *)(wrapper), (unsigned short)mode);
      printf("SetAdaptEcV32(%d) -> e108=%u e10c=%u e14=%u e18=%u e144=%u e146=%u e114=%d e110=%d e112=%d e140=%d\n",
             mode, *(unsigned int *)(engine + 0xe0 + 0x108), *(unsigned int *)(engine + 0xe0 + 0x10c),
             *(unsigned int *)(engine + 0xe0 + 0x14), *(unsigned int *)(engine + 0xe0 + 0x18),
             *(unsigned short *)(engine + 0xe0 + 0x144), *(unsigned short *)(engine + 0xe0 + 0x146),
             *(short *)(engine + 0xe0 + 0x114), *(short *)(engine + 0xe0 + 0x110),
             *(short *)(engine + 0xe0 + 0x112), *(short *)(engine + 0xe0 + 0x140));
    }
  }

  free(engine);
}

/*
 * Scenario selection: matches test/b103_diff_main.c's/test/v23_diff_main.c's
 * established one-scenario-per-process precedent - running several of these
 * test groups back-to-back in ONE process was found, during this chunk's
 * own development, to make the REAL dsplibs.o intermittently (ASLR-
 * dependent - confirmed via repeated runs, and via runs under gdb/
 * `setarch -R` never reproducing it, the classic signature of a stale/
 * leftover-state sensitivity rather than a deterministic bug) segfault
 * partway through a later group, while never failing when each group runs
 * alone in its own process - the same class of "some dsplibs.o-internal
 * state doesn't tolerate several live instances accumulating in one
 * process" finding already documented for B103/V.23 in NOTES.md. Every
 * group below is therefore isolated into its own process.
 */
static void test_fse_receive_trace(int callCount);

int main(int argc, char **argv)
{
  int scenario = argc > 1 ? atoi(argv[1]) : 0;

  switch (scenario) {
  case 0:
    test_scrambler();
    test_smc_encoders();
    break;
  case 1:
    test_pps();
    break;
  case 2:
    g_lcg = 999983u;
    test_sre();
    break;
  case 3:
    g_lcg = 424242u;
    test_ecc();
    break;
  case 4:
    g_lcg = 13131313u;
    test_fse_receive(0, 12000, "AB_chain");
    break;
  case 5:
    g_lcg = 24242424u;
    test_fse_receive(1, 800, "4pt_direct");
    break;
  case 6:
    g_lcg = 55555555u;
    test_fse_decisions_direct();
    break;
  case 7:
    g_lcg = 77777777u;
    test_midlevel_helpers();
    break;
  case 8:
    g_lcg = 13131313u;
    test_fse_receive_trace(120);
    break;
  case 9:
    g_lcg = 20260929u;
    test_ecc_real_tables();
    break;
  default:
    break;
  }
  return 0;
}

/* ===================== FSE receive fine-grained trace (debug) =====================
 * Dumps FPM_FSE_receive's own internal per-symbol state after every 1-sample
 * chunk call, to bisect exactly which call/field first diverges between
 * dsplibs.o and the port. Added during, and instrumental to, the follow-up
 * pass that root-caused the "diverges after 14 correct decisions" finding to
 * a real bug in FPM_atan (src/v32/v32_math.c) - see NOTES.md. Kept as scenario
 * 8 (NOT part of the main 0-7 scenario loop `build_and_diff_v32_primitives.sh`
 * runs) since it's a useful diagnostic for any future FSE-adjacent
 * regression - run manually via `./test/.build/v32/v32_engine.primitives_test_<blob|port> 8`.
 */
static void test_fse_receive_trace(int callCount)
{
  unsigned char *fseEng = v32h_eng(); /* FSE at +0x204 (0x4e18 bytes), its history / VTB object at +0x5020 of one full engine allocation */
  short *fseObj = (short *)(fseEng + 0x204);
  unsigned char *fseHist = fseEng + 0x5020;
  int cfg[14];
  int c;

  printf("=== FPM_FSE_receive trace ===\n");
  g_fseCfgBuild(cfg);
  FPM_FSE_init(fseObj, cfg, 1);
  V32E_PS(void *, (char *)(fseEng), 0x230, fseHist); /* FSE +0x2c: the history object */
  *(short *)((char *)fseObj + 0x74) = 3;
  *(short *)((char *)fseObj + 0x8a) = 2;
  V32E_PS(void *, (char *)(fseEng), 0x234, (void *)FSE_decision_AB); /* FSE +0x30: the decision callback */

  for (c = 0; c < callCount; c++) {
    short rx[1];
    short outb[1];
    short produced;
    rx[0] = (short)(lcg_short(400) - 200);
    outb[0] = 0;
    produced = FPM_FSE_receive((char *)(fseObj), rx, outb, 1);
    printf("call %d: produced=%d outb=%d "
           "f74=%d f8a=%d f6c=%u f50=%d f52=%d f70=%d f88=%d f3a=%d f76=%d "
           "f78=%d f7a=%d f7c=%d f7e=%d f80=%d f82=%d f84=%d f86=%d f5e=%d "
           "h28=%d h29=%d h2a=%d h2b=%d h2c=%d h2d=%d h2e=%d h2f=%d h30=%d h37=%d\n",
           c, produced, outb[0],
           *(short *)((char *)fseObj + 0x74), *(short *)((char *)fseObj + 0x8a),
           *(unsigned short *)((char *)fseObj + 0x6c),
           *(short *)((char *)fseObj + 0x50), *(short *)((char *)fseObj + 0x52),
           *(int *)((char *)fseObj + 0x70),
           *(short *)((char *)fseObj + 0x88), *(short *)((char *)fseObj + 0x3a),
           *(short *)((char *)fseObj + 0x76),
           *(short *)((char *)fseObj + 0x78), *(short *)((char *)fseObj + 0x7a),
           *(short *)((char *)fseObj + 0x7c), *(short *)((char *)fseObj + 0x7e),
           *(short *)((char *)fseObj + 0x80), *(short *)((char *)fseObj + 0x82),
           *(short *)((char *)fseObj + 0x84), *(short *)((char *)fseObj + 0x86),
           *(short *)((char *)fseObj + 0x5e),
           *(short *)(fseHist + 0x50), *(short *)(fseHist + 0x52),
           *(short *)(fseHist + 0x54), *(short *)(fseHist + 0x56),
           *(short *)(fseHist + 0x58), *(short *)(fseHist + 0x5a),
           *(short *)(fseHist + 0x5c), *(short *)(fseHist + 0x5e),
           *(short *)(fseHist + 0x60), *(short *)(fseHist + 0x6e));
  }

  FPM_FSE_free(fseObj);
  free(fseEng);
}
